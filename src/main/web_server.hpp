#pragma once

#include "esp_err.h"
#include "icm42670_spi.hpp"

esp_err_t start_web_server(ICM42670Spi *imu, const ICM42670Config &imu_config);
esp_err_t web_server_queue_imu_data(const ICM42670Sample &sample);
