#pragma once

#include "esp_err.h"
#include "array"
struct TelemetrySample
{
    std::array<float, 3> acc;
    std::array<float, 3> gyro;
    std::array<float, 3> f_acc;
    std::array<float, 3> f_gyro;
};

esp_err_t start_web_server();
esp_err_t web_server_queue_telemetry(const TelemetrySample &sample);
