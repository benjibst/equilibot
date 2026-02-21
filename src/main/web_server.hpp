#pragma once

#include "esp_err.h"
#include "icm42670_spi.hpp"
esp_err_t start_web_server();
esp_err_t web_server_queue_imu_data(const ICM42670Sample &sample);
