#include "web_server.hpp"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <array>
#include <cassert>
#include <cstring>
#include <utility>
#include <nlohmann/json.hpp>

namespace
{
    constexpr char AP_SSID[] = "equilibot";
    constexpr char AP_PASSWORD[] = "equilibot123";
    constexpr uint8_t WIFI_CHANNEL = 1;
    constexpr uint8_t WIFI_MAX_CONNECTIONS = 4;

    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");

    bool is_valid_acce_odr(int value)
    {
        return value >= ICM42670_ACCE_ODR_1600HZ && value <= ICM42670_ACCE_ODR_1_5625HZ;
    }

    bool is_valid_gyro_odr(int value)
    {
        return value >= ICM42670_GYRO_ODR_1600HZ && value <= ICM42670_GYRO_ODR_12_5HZ;
    }

    bool is_valid_acce_fs(int value)
    {
        return value >= ICM42670_ACCE_FS_16G && value <= ICM42670_ACCE_FS_2G;
    }

    bool is_valid_gyro_fs(int value)
    {
        return value >= ICM42670_GYRO_FS_2000DPS && value <= ICM42670_GYRO_FS_250DPS;
    }

    bool is_valid_filter_bw(int value)
    {
        return value >= ICM42670_UI_FILT_BW_BYPASS && value <= ICM42670_UI_FILT_BW_16HZ;
    }

    esp_err_t parse_imu_config_json(const nlohmann::json &message,
                                    ICM42670Config &current_config)
    {
        const nlohmann::json &config = message.at("config");

        auto apply_if_valid = [&config, &current_config]<typename TEnum>(const char *key, bool (*valid)(int), TEnum &out)
        {
            int value = config[key];
            if (valid(value))
                out = static_cast<TEnum>(value);
        };

        apply_if_valid("acce_odr", is_valid_acce_odr, current_config.acce_odr);
        apply_if_valid("gyro_odr", is_valid_gyro_odr, current_config.gyro_odr);
        apply_if_valid("acce_fs", is_valid_acce_fs, current_config.acce_fs);
        apply_if_valid("gyro_fs", is_valid_gyro_fs, current_config.gyro_fs);
        apply_if_valid("acce_bw", is_valid_filter_bw, current_config.acce_bw);
        apply_if_valid("gyro_bw", is_valid_filter_bw, current_config.gyro_bw);
        return ESP_OK;
    }

    esp_err_t apply_imu_config_json(ICM42670Spi &imu, ICM42670Config &current_config,
                                    const nlohmann::json &message)
    {
        const esp_err_t parse_err = parse_imu_config_json(message, current_config);
        if (parse_err != ESP_OK)
        {
            return parse_err;
        }

        return imu.configure_sensor(current_config);
    }
} // namespace

WebServer::WebServer(ICM42670Spi &imu, const ICM42670Config &imu_config)
    : imu_(imu), imu_config_(imu_config)
{
}

esp_err_t WebServer::start()
{
    if (started_)
    {
        return ESP_OK;
    }

    esp_err_t err = init_nvs();
    if (err != ESP_OK)
    {
        return err;
    }

    err = init_softap();
    if (err != ESP_OK)
    {
        return err;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096 + 2048;
    err = httpd_start(&server_, &config);
    if (err != ESP_OK)
    {
        return err;
    }
    assert(server_ != nullptr);

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = this,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
    };
    err = httpd_register_uri_handler(server_, &root_uri);
    if (err != ESP_OK)
    {
        httpd_stop(server_);
        server_ = nullptr;
        return err;
    }

    const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_get_handler,
        .user_ctx = this,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
    };
    err = httpd_register_uri_handler(server_, &ws_uri);
    if (err != ESP_OK)
    {
        httpd_stop(server_);
        server_ = nullptr;
        return err;
    }

    const BaseType_t task_create_result = telemetry_sender_task_.create("web_tx",
                                                                        tskIDLE_PRIORITY + 2,
                                                                        telemetry_sender_task_entry,
                                                                        *this);
    assert(task_create_result == pdPASS);
    if (task_create_result != pdPASS)
    {
        httpd_stop(server_);
        server_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    started_ = true;
    ESP_LOGI(__FILE__, "SoftAP started. Open http://192.168.4.1/");
    return ESP_OK;
}

esp_err_t WebServer::queue_imu_data(const TelemetryData &sample)
{
    return telemetry_queue_.send(sample, 0) == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t WebServer::root_get_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    const size_t html_len = static_cast<size_t>(index_html_end - index_html_start);
    return httpd_resp_send(request, reinterpret_cast<const char *>(index_html_start), html_len);
}

esp_err_t WebServer::ws_get_handler(httpd_req_t *request)
{
    WebServer &server = *static_cast<WebServer *>(request->user_ctx);
    return server.handle_ws_request(request);
}

esp_err_t WebServer::handle_ws_request(httpd_req_t *request)
{
    if (request->method == HTTP_GET)
    {
        ESP_LOGI(__FILE__, "WebSocket handshake complete");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_frame = {};
    esp_err_t err = httpd_ws_recv_frame(request, &ws_frame, 0);
    if (err != ESP_OK)
    {
        ESP_LOGW(__FILE__, "ws recv length failed: %s", esp_err_to_name(err));
        return err;
    }

    if (ws_frame.len == 0)
    {
        return ESP_OK;
    }

    std::string payload(ws_frame.len, '\0');
    ws_frame.payload = reinterpret_cast<uint8_t *>(payload.data());
    err = httpd_ws_recv_frame(request, &ws_frame, ws_frame.len);
    if (err != ESP_OK)
    {
        return err;
    }

    const char *begin = payload.data();
    const char *end = begin + ws_frame.len;
    const nlohmann::json message = nlohmann::json::parse(begin, end, nullptr, false);
    if (message.is_discarded())
    {
        ESP_LOGW(__FILE__, "Invalid websocket JSON payload");
        return ESP_OK;
    }

    const auto type_it = message.find("type");
    if (type_it != message.end() && type_it->is_string() && type_it->get<std::string>() == "imu_config")
    {
        const esp_err_t apply_err = apply_imu_config_json(imu_, imu_config_, message);
        if (apply_err != ESP_OK)
        {
            ESP_LOGW(__FILE__, "Failed applying IMU config: %s", esp_err_to_name(apply_err));
        }
        else
        {
            ESP_LOGI(__FILE__, "Applied IMU config over websocket");
        }
    }

    return ESP_OK;
}

void WebServer::ws_broadcast_work(void *arg)
{
    WsBroadcastContext *context = static_cast<WsBroadcastContext *>(arg);
    std::array<int, CONFIG_LWIP_MAX_SOCKETS> client_fds = {};
    size_t fd_count = client_fds.size();
    esp_err_t list_err = httpd_get_client_list(context->server->server_, &fd_count, client_fds.data());
    if (list_err != ESP_OK)
    {
        ESP_LOGW(__FILE__, "httpd_get_client_list failed: %s", esp_err_to_name(list_err));
        delete context;
        return;
    }

    httpd_ws_frame_t ws_frame = {};
    ws_frame.type = HTTPD_WS_TYPE_TEXT;
    ws_frame.payload = reinterpret_cast<uint8_t *>(context->payload.data());
    ws_frame.len = context->payload.size();

    for (size_t i = 0; i < fd_count; ++i)
    {
        const int fd = client_fds[i];
        if (httpd_ws_get_fd_info(context->server->server_, fd) != HTTPD_WS_CLIENT_WEBSOCKET)
        {
            continue;
        }

        const esp_err_t send_err = httpd_ws_send_frame_async(context->server->server_, fd, &ws_frame);
        if (send_err != ESP_OK)
        {
            ESP_LOGW(__FILE__, "ws send failed on fd=%d: %s", fd, esp_err_to_name(send_err));
        }
    }

    delete context;
}

void WebServer::telemetry_sender_task_entry(WebServer &self)
{
    self.telemetry_sender_task();
}

void WebServer::telemetry_sender_task()
{
    while (true)
    {
        TelemetryData telemetry = {};
        if (telemetry_queue_.receive(telemetry, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        nlohmann::json payload = {
            {"acc", nlohmann::json::array()},
            {"gyro", nlohmann::json::array()},
        };
        payload["acc"].get_ref<nlohmann::json::array_t &>().reserve(kMaxSamplesPerTelemetryPayload);
        payload["gyro"].get_ref<nlohmann::json::array_t &>().reserve(kMaxSamplesPerTelemetryPayload);

        auto append_sample = [&payload](const ICM42670Sample &telemetry_sample)
        {
            payload["acc"].push_back({
                {"ts_us", telemetry_sample.ts_us},
                {"data", telemetry_sample.acc},
            });
            payload["gyro"].push_back({
                {"ts_us", telemetry_sample.ts_us},
                {"data", telemetry_sample.gyro},
            });
        };

        append_sample(telemetry.sample);
        size_t samples_in_payload = 1;
        while (samples_in_payload < kMaxSamplesPerTelemetryPayload &&
               telemetry_queue_.receive(telemetry, 0) == pdTRUE)
        {
            append_sample(telemetry.sample);
            ++samples_in_payload;
        }
        payload["theta_kalman"] = telemetry.theta_kalman_rad;
        payload["theta_int"] = telemetry.theta_int_rad;
        std::string serialized = payload.dump();
        publish_telemetry_payload(std::move(serialized));
    }
}

esp_err_t WebServer::publish_telemetry_payload(std::string &&serialized)
{
    auto *context = new WsBroadcastContext{
        .server = this,
        .payload = std::move(serialized),
    };

    const esp_err_t err = httpd_queue_work(server_, ws_broadcast_work, context);
    if (err != ESP_OK)
    {
        delete context;
    }
    return err;
}

esp_err_t WebServer::init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        err = nvs_flash_erase();
        if (err != ESP_OK)
        {
            return err;
        }
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t WebServer::init_softap()
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    ap_netif_ = esp_netif_create_default_wifi_ap();
    assert(ap_netif_ != nullptr);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    wifi_config_t ap_cfg = {};
    std::strncpy(reinterpret_cast<char *>(ap_cfg.ap.ssid), AP_SSID, sizeof(ap_cfg.ap.ssid));
    std::strncpy(reinterpret_cast<char *>(ap_cfg.ap.password), AP_PASSWORD, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = std::strlen(AP_SSID);
    ap_cfg.ap.channel = WIFI_CHANNEL;
    ap_cfg.ap.max_connection = WIFI_MAX_CONNECTIONS;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    return esp_wifi_start();
}
