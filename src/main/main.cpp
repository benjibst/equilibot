#include "spi_bus.hpp"
#include "common.hpp"
#include "icm42670_spi.hpp"
#include "led_strip.hpp"
#include "web_server.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmc5240.hpp"
#include "filter.hpp"
#include <cmath>

extern "C" void app_main(void)
{
    SpiBus spi_bus(SPI2_HOST, {.miso = GPIO_NUM_13, .mosi = GPIO_NUM_14, .sclk = GPIO_NUM_21}, NUM_DEVICES);
    ICM42670Config imu_config = {
        .config = {
            .acce_fs = ACCE_FS_2G,
            .acce_odr = ACCE_ODR_400HZ,
            .gyro_fs = GYRO_FS_250DPS,
            .gyro_odr = GYRO_ODR_400HZ},
        .acce_pwr = ACCE_PWR_LOWNOISE,
        .gyro_pwr = GYRO_PWR_LOWNOISE,
        .acce_ui_filt_bw = ACCE_UI_FILT_BW_BYPASS,
        .gyro_ui_filt_bw = GYRO_UI_FILT_BW_BYPASS};
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
    TMC5240 mot1(spi_bus, GPIO_NUM_12, MOT_1);
    TMC5240 mot2(spi_bus, GPIO_NUM_10, MOT_2);
    esp_err_t web_server_err = start_web_server();
    if (web_server_err != ESP_OK)
    {
        ESP_LOGE(__FILE__, "Failed to start web server: %s", esp_err_to_name(web_server_err));
    }
    // Tuned for stable web telemetry while keeping responsive motion tracking.
    LowPassIIR<3> acc_filter(8.0f);
    LowPassIIR<3> gyro_filter(10.0f);
    TickType_t last_telemetry_send = 0;
    int64_t last_sample_us = 0;
    while (true)
    {
        const int64_t now_us = esp_timer_get_time();
        float dt_seconds = 1.0f / 400.0f;
        if (last_sample_us > 0)
        {
            dt_seconds = static_cast<float>(now_us - last_sample_us) / 1'000'000.0f;
            if (dt_seconds < 0.0001f)
            {
                dt_seconds = 0.0001f;
            }
        }
        last_sample_us = now_us;

        ICM42670Sample sample = imu.read_sample();
        auto f_acc = acc_filter.process(sample.acc, dt_seconds);
        auto f_gyro = gyro_filter.process(sample.gyro, dt_seconds);
        TickType_t now = xTaskGetTickCount();
        if (now - last_telemetry_send >= pdMS_TO_TICKS(50))
        {
            TelemetrySample telemetry = {
                sample.acc,
                sample.gyro,
                f_acc, f_gyro};
            web_server_publish_telemetry(telemetry);
            last_telemetry_send = now;
        }
    }
}
