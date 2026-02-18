#pragma once

#include "esp_err.h"

struct TelemetrySample
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

esp_err_t start_web_server();
esp_err_t web_server_publish_telemetry(const TelemetrySample &sample);
