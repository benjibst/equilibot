#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "freertos_wrappers.hpp"
#include "icm42670_spi.hpp"
#include <cstddef>
#include <cstdint>
#include <string>

class WebServer
{
public:
    WebServer(ICM42670Spi &imu, const ICM42670Config &imu_config);

    esp_err_t start();
    esp_err_t queue_imu_data(const ICM42670Sample &sample);

    WebServer(const WebServer &) = delete;
    WebServer &operator=(const WebServer &) = delete;
    WebServer(WebServer &&) = delete;
    WebServer &operator=(WebServer &&) = delete;

private:
    static constexpr size_t kTelemetryQueueLength = 8;
    static constexpr uint32_t kTelemetryTaskStackSizeBytes = 4096;

    struct WsBroadcastContext
    {
        WebServer *server;
        std::string payload;
    };

    static esp_err_t root_get_handler(httpd_req_t *request);
    static esp_err_t ws_get_handler(httpd_req_t *request);
    static void ws_broadcast_work(void *arg);
    static void telemetry_sender_task_entry(WebServer &self);

    esp_err_t init_nvs();
    esp_err_t init_softap();
    esp_err_t handle_ws_request(httpd_req_t *request);
    esp_err_t publish_telemetry_payload(std::string &&serialized);
    void telemetry_sender_task();

    ICM42670Spi &imu_;
    ICM42670Config imu_config_;

    httpd_handle_t server_ = nullptr;
    esp_netif_t *ap_netif_ = nullptr;
    bool started_ = false;

    StaticQueue<ICM42670Sample, kTelemetryQueueLength> telemetry_queue_;
    StaticTask<kTelemetryTaskStackSizeBytes, WebServer> telemetry_sender_task_;
};
