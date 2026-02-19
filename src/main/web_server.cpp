#include "web_server.hpp"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace
{
    constexpr char TAG[] = "web_server";
    constexpr char AP_SSID[] = "equilibot";
    constexpr char AP_PASSWORD[] = "equilibot123";
    constexpr uint8_t WIFI_CHANNEL = 1;
    constexpr uint8_t WIFI_MAX_CONNECTIONS = 4;

    httpd_handle_t g_server = nullptr;

    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");

    struct WsBroadcastContext
    {
        httpd_handle_t server;
        char payload[1024];
        size_t payload_len;
    };

    void ws_broadcast_work(void *arg)
    {
        WsBroadcastContext *context = (WsBroadcastContext *)arg;
        std::array<int, CONFIG_LWIP_MAX_SOCKETS> client_fds = {};
        size_t fd_count = client_fds.size();
        esp_err_t list_err = httpd_get_client_list(context->server, &fd_count, client_fds.data());
        if (list_err != ESP_OK)
        {
            ESP_LOGW(TAG, "httpd_get_client_list failed: %s", esp_err_to_name(list_err));
            delete context;
            return;
        }

        httpd_ws_frame_t ws_frame = {};
        ws_frame.type = HTTPD_WS_TYPE_TEXT;
        ws_frame.payload = reinterpret_cast<uint8_t *>(context->payload);
        ws_frame.len = context->payload_len;

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
                ESP_LOGW(TAG, "ws send failed on fd=%d: %s", fd, esp_err_to_name(send_err));
            }
        }
        delete context;
    }

    esp_err_t ws_get_handler(httpd_req_t *request)
    {
        if (request->method == HTTP_GET)
        {
            ESP_LOGI(TAG, "WebSocket handshake complete");
            return ESP_OK;
        }

        httpd_ws_frame_t ws_frame = {};
        esp_err_t err = httpd_ws_recv_frame(request, &ws_frame, 0);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "ws recv length failed: %s", esp_err_to_name(err));
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

esp_err_t start_web_server()
{
    if (g_server != nullptr)
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

    ESP_LOGI(TAG, "SoftAP started. Open http://192.168.4.1/");
    return ESP_OK;
}
int printxyz(char *buf, size_t sz, const std::array<float, 3> &xyz)
{
    return std::snprintf(buf, sz, R"("x":%.5f,"y":%.5f,"z":%.5f)", xyz[0], xyz[1], xyz[2]);
}
size_t write_telemetry_json(char *buf, size_t sz, const TelemetrySample &sample)
{
    size_t written = 0;
    written += std::snprintf(buf + written, sz - written, R"({"t_ms":%lld,"acc":{)", esp_timer_get_time() / 1000);
    written += printxyz(buf + written, sz - written, sample.acc);
    written += std::snprintf(buf + written, sz - written, R"(},"gyro":{)");
    written += printxyz(buf + written, sz - written, sample.gyro);
    written += std::snprintf(buf + written, sz - written, R"(},"f_acc":{)");
    written += printxyz(buf + written, sz - written, sample.f_acc);
    written += std::snprintf(buf + written, sz - written, R"(},"f_gyro":{)");
    written += printxyz(buf + written, sz - written, sample.f_gyro);
    written += std::snprintf(buf + written, sz - written, R"(}})");
    assert(written < sz);
    return written;
}
esp_err_t web_server_publish_telemetry(const TelemetrySample &sample)
{
    if (!g_server)
    {
        return ESP_ERR_INVALID_STATE;
    }
    WsBroadcastContext *context = new WsBroadcastContext;
    context->server = g_server;
    auto sz = write_telemetry_json(context->payload, sizeof(context->payload), sample);
    ESP_LOGI(__FILE__, "%s", context->payload);
    context->payload_len = sz;

    esp_err_t err = httpd_queue_work(g_server, ws_broadcast_work, context);
    if (err != ESP_OK)
    {
        delete context;
    }
    return err;
}
