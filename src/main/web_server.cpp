#include "web_server.hpp"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <nlohmann/json.hpp>

namespace
{
    constexpr char AP_SSID[] = "equilibot";
    constexpr char AP_PASSWORD[] = "equilibot123";
    constexpr uint8_t WIFI_CHANNEL = 1;
    constexpr uint8_t WIFI_MAX_CONNECTIONS = 4;

    httpd_handle_t g_server = nullptr;
    QueueHandle_t g_telemetry_queue = nullptr;
    TaskHandle_t g_telemetry_sender_task = nullptr;
    ICM42670Spi *g_imu = nullptr;
    ICM42670Config g_imu_config = {
        .acce_odr = ICM42670_ACCE_ODR_400HZ,
        .gyro_odr = ICM42670_GYRO_ODR_400HZ,
        .acce_fs = ICM42670_ACCE_FS_2G,
        .gyro_fs = ICM42670_GYRO_FS_250DPS,
        .acce_bw = ICM42670_UI_FILT_BW_BYPASS,
        .gyro_bw = ICM42670_UI_FILT_BW_BYPASS,
    };
    constexpr size_t kWebTelemetryQueueLength = 8;

    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");

    struct WsBroadcastContext
    {
        httpd_handle_t server;
        std::string payload;
    };

    void telemetry_sender_task(void *);

    bool is_valid_acce_odr(int value)
    {
        switch (value)
        {
        case ICM42670_ACCE_ODR_1600HZ:
        case ICM42670_ACCE_ODR_800HZ:
        case ICM42670_ACCE_ODR_400HZ:
        case ICM42670_ACCE_ODR_200HZ:
        case ICM42670_ACCE_ODR_100HZ:
        case ICM42670_ACCE_ODR_50HZ:
        case ICM42670_ACCE_ODR_25HZ:
        case ICM42670_ACCE_ODR_12_5HZ:
        case ICM42670_ACCE_ODR_6_25HZ:
        case ICM42670_ACCE_ODR_3_125HZ:
        case ICM42670_ACCE_ODR_1_5625HZ:
            return true;
        default:
            return false;
        }
    }

    bool is_valid_gyro_odr(int value)
    {
        switch (value)
        {
        case ICM42670_GYRO_ODR_1600HZ:
        case ICM42670_GYRO_ODR_800HZ:
        case ICM42670_GYRO_ODR_400HZ:
        case ICM42670_GYRO_ODR_200HZ:
        case ICM42670_GYRO_ODR_100HZ:
        case ICM42670_GYRO_ODR_50HZ:
        case ICM42670_GYRO_ODR_25HZ:
        case ICM42670_GYRO_ODR_12_5HZ:
            return true;
        default:
            return false;
        }
    }

    bool is_valid_acce_fs(int value)
    {
        switch (value)
        {
        case ICM42670_ACCE_FS_16G:
        case ICM42670_ACCE_FS_8G:
        case ICM42670_ACCE_FS_4G:
        case ICM42670_ACCE_FS_2G:
            return true;
        default:
            return false;
        }
    }

    bool is_valid_gyro_fs(int value)
    {
        switch (value)
        {
        case ICM42670_GYRO_FS_2000DPS:
        case ICM42670_GYRO_FS_1000DPS:
        case ICM42670_GYRO_FS_500DPS:
        case ICM42670_GYRO_FS_250DPS:
            return true;
        default:
            return false;
        }
    }

    bool is_valid_filter_bw(int value)
    {
        return value >= ICM42670_UI_FILT_BW_BYPASS && value <= ICM42670_UI_FILT_BW_16HZ;
    }

    esp_err_t apply_imu_config_json(const nlohmann::json &message)
    {
        if (g_imu == nullptr)
        {
            return ESP_ERR_INVALID_STATE;
        }

        const nlohmann::json *cfg = &message;
        const auto config_it = message.find("config");
        if (config_it != message.end())
        {
            if (!config_it->is_object())
            {
                return ESP_ERR_INVALID_ARG;
            }
            cfg = &(*config_it);
        }

        ICM42670Config updated = g_imu_config;
        int value = 0;

        const auto acce_odr_it = cfg->find("acce_odr");
        if (acce_odr_it != cfg->end())
        {
            if (!acce_odr_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = acce_odr_it->get<int>();
            if (!is_valid_acce_odr(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.acce_odr = static_cast<ICM42670AcceODR_t>(value);
        }

        const auto gyro_odr_it = cfg->find("gyro_odr");
        if (gyro_odr_it != cfg->end())
        {
            if (!gyro_odr_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = gyro_odr_it->get<int>();
            if (!is_valid_gyro_odr(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.gyro_odr = static_cast<ICM42670GyroODR_t>(value);
        }

        const auto acce_fs_it = cfg->find("acce_fs");
        if (acce_fs_it != cfg->end())
        {
            if (!acce_fs_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = acce_fs_it->get<int>();
            if (!is_valid_acce_fs(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.acce_fs = static_cast<ICM42670AcceFS_t>(value);
        }

        const auto gyro_fs_it = cfg->find("gyro_fs");
        if (gyro_fs_it != cfg->end())
        {
            if (!gyro_fs_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = gyro_fs_it->get<int>();
            if (!is_valid_gyro_fs(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.gyro_fs = static_cast<ICM42670GyroFS_t>(value);
        }

        const auto acce_bw_it = cfg->find("acce_bw");
        if (acce_bw_it != cfg->end())
        {
            if (!acce_bw_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = acce_bw_it->get<int>();
            if (!is_valid_filter_bw(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.acce_bw = static_cast<ICM42670LowpassBW_t>(value);
        }

        const auto gyro_bw_it = cfg->find("gyro_bw");
        if (gyro_bw_it != cfg->end())
        {
            if (!gyro_bw_it->is_number_integer())
            {
                return ESP_ERR_INVALID_ARG;
            }
            value = gyro_bw_it->get<int>();
            if (!is_valid_filter_bw(value))
            {
                return ESP_ERR_INVALID_ARG;
            }
            updated.gyro_bw = static_cast<ICM42670LowpassBW_t>(value);
        }

        const esp_err_t err = g_imu->configure_sensor(updated);
        if (err == ESP_OK)
        {
            g_imu_config = updated;
        }
        return err;
    }

    void ws_broadcast_work(void *arg)
    {
        WsBroadcastContext *context = (WsBroadcastContext *)arg;
        std::array<int, CONFIG_LWIP_MAX_SOCKETS> client_fds = {};
        size_t fd_count = client_fds.size();
        esp_err_t list_err = httpd_get_client_list(context->server, &fd_count, client_fds.data());
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
            int fd = client_fds[i];
            if (httpd_ws_get_fd_info(context->server, fd) != HTTPD_WS_CLIENT_WEBSOCKET)
            {
                continue;
            }

            esp_err_t send_err = httpd_ws_send_frame_async(context->server, fd, &ws_frame);
            if (send_err != ESP_OK)
            {
                ESP_LOGW(__FILE__, "ws send failed on fd=%d: %s", fd, esp_err_to_name(send_err));
            }
        }
        delete context;
    }

    esp_err_t ws_get_handler(httpd_req_t *request)
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

        uint8_t *payload = static_cast<uint8_t *>(calloc(1, ws_frame.len + 1));
        if (payload == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }

        ws_frame.payload = payload;
        err = httpd_ws_recv_frame(request, &ws_frame, ws_frame.len);
        if (err == ESP_OK)
        {
            const char *begin = reinterpret_cast<const char *>(payload);
            const char *end = begin + ws_frame.len;
            const nlohmann::json message = nlohmann::json::parse(begin, end, nullptr, false);
            if (message.is_discarded())
            {
                ESP_LOGW(__FILE__, "Invalid websocket JSON payload");
            }
            else
            {
                const auto type_it = message.find("type");
                if (type_it != message.end() && type_it->is_string() &&
                    type_it->get<std::string>() == "imu_config")
                {
                    const esp_err_t apply_err = apply_imu_config_json(message);
                    if (apply_err != ESP_OK)
                    {
                        ESP_LOGW(__FILE__, "Failed applying IMU config: %s", esp_err_to_name(apply_err));
                    }
                    else
                    {
                        ESP_LOGI(__FILE__, "Applied IMU config over websocket");
                    }
                }
            }
        }
        free(payload);
        return err;
    }

    esp_err_t root_get_handler(httpd_req_t *request)
    {
        httpd_resp_set_type(request, "text/html; charset=utf-8");
        size_t html_len = static_cast<size_t>(index_html_end - index_html_start);
        return httpd_resp_send(request, reinterpret_cast<const char *>(index_html_start), html_len);
    }

    esp_err_t init_nvs()
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

    esp_err_t init_softap()
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

        if (esp_netif_create_default_wifi_ap() == nullptr)
        {
            return ESP_FAIL;
        }

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
} // namespace

esp_err_t start_web_server(ICM42670Spi *imu, const ICM42670Config &imu_config)
{
    if (imu == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    g_imu = imu;
    g_imu_config = imu_config;

    if (g_server != nullptr)
    {
        return ESP_OK;
    }

    if (g_telemetry_queue == nullptr)
    {
        g_telemetry_queue = xQueueCreate(kWebTelemetryQueueLength, sizeof(ICM42670Sample));
        if (g_telemetry_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }
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
    err = httpd_start(&g_server, &config);
    if (err != ESP_OK)
    {
        return err;
    }

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr,
    };
    err = httpd_register_uri_handler(g_server, &root_uri);
    if (err != ESP_OK)
    {
        httpd_stop(g_server);
        g_server = nullptr;
        return err;
    }

    const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_get_handler,
        .user_ctx = nullptr,
        .is_websocket = true,
    };
    err = httpd_register_uri_handler(g_server, &ws_uri);
    if (err != ESP_OK)
    {
        httpd_stop(g_server);
        g_server = nullptr;
        return err;
    }

    if (g_telemetry_sender_task == nullptr)
    {
        BaseType_t ok = xTaskCreate(telemetry_sender_task,
                                    "web_tx",
                                    4096,
                                    nullptr,
                                    tskIDLE_PRIORITY + 2,
                                    &g_telemetry_sender_task);
        if (ok != pdPASS)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(__FILE__, "SoftAP started. Open http://192.168.4.1/");
    return ESP_OK;
}
namespace
{
    esp_err_t publish_telemetry_payload(std::string &&serialized)
    {
        if (!g_server)
        {
            return ESP_ERR_INVALID_STATE;
        }

        WsBroadcastContext *context = new WsBroadcastContext;
        if (!context)
            return ESP_ERR_NO_MEM;
        context->server = g_server;
        context->payload = std::move(serialized);

        esp_err_t err = httpd_queue_work(g_server, ws_broadcast_work, context);
        if (err != ESP_OK)
        {
            delete context;
        }
        return err;
    }

    void telemetry_sender_task(void *)
    {
        while (true)
        {
            ICM42670Sample sample = {};
            xQueueReceive(g_telemetry_queue, &sample, portMAX_DELAY);

            nlohmann::json payload = {
                {"acc", nlohmann::json::array()},
                {"gyro", nlohmann::json::array()},
            };

            auto append_sample = [&payload](const ICM42670Sample &telemetry_sample)
            {
                payload["acc"].push_back({
                    {"ts_ms", telemetry_sample.ts_ms},
                    {"data", telemetry_sample.acc},
                });
                payload["gyro"].push_back({
                    {"ts_ms", telemetry_sample.ts_ms},
                    {"data", telemetry_sample.gyro},
                });
            };

            append_sample(sample);
            while (xQueueReceive(g_telemetry_queue, &sample, 0) == pdTRUE)
            {
                append_sample(sample);
            }
            publish_telemetry_payload(payload.dump());
        }
    }
}

esp_err_t web_server_queue_imu_data(const ICM42670Sample &sample)
{
    return xQueueSend(g_telemetry_queue, &sample, 0);
}
