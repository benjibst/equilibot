#include "spi_bus.hpp"
#include "common.hpp"
#include "icm42670_spi.hpp"
#include "led_strip.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

extern "C" void app_main(void)
{
    SpiBus spi_bus(SPI2_HOST, {.miso = GPIO_NUM_13, .mosi = GPIO_NUM_14, .sclk = GPIO_NUM_21}, NUM_DEVICES);
    ICM42670Config imu_config = {
        .config = {
            .acce_fs = ACCE_FS_2G,
            .acce_odr = ACCE_ODR_400HZ,
            .gyro_fs = GYRO_FS_1000DPS,
            .gyro_odr = GYRO_ODR_400HZ},
        .acce_pwr = ACCE_PWR_LOWNOISE,
        .gyro_pwr = GYRO_PWR_LOWNOISE,
        .acce_ui_filt_bw = ACCE_UI_FILT_BW_16HZ,
        .gyro_ui_filt_bw = GYRO_UI_FILT_BW_16HZ};

    ICM42670Spi imu(spi_bus, GPIO_NUM_41, imu_config);
    LedStripConfig led_strip_config = {
        .led_count = 31,
        .tilt_segment = {.start_index = 0, .led_count = 19},
        .battery_segment = {.start_index = 19, .led_count = 5},
        .motor1_segment = {.start_index = 24, .led_count = 3},
        .connection_segment = {.start_index = 27, .led_count = 1},
        .motor2_segment = {.start_index = 28, .led_count = 3},
    };
    EquilibotLedStrip led_strip(GPIO_NUM_38, led_strip_config);
    while (true)
    {
        ICM42670Sample sample = imu.read_sample();
        sample.acc.z -= 1.0f; // compensate for gravity
        ESP_LOGI(__FILE__, "acc: %f %f %f, gyro: %f %f %f", sample.acc.x, sample.acc.y, sample.acc.z, sample.gyro.x, sample.gyro.y, sample.gyro.z);
        float abs = std::sqrt(sample.acc.x * sample.acc.x + sample.acc.y * sample.acc.y + sample.acc.z * sample.acc.z);
        float angle = std::lerp(-90.0f, 90.0f, (abs));
        led_strip.set_tilt(angle);
    }
}
