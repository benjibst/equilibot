#include "spi_bus.hpp"
#include "common.hpp"
#include "icm42670_spi.hpp"
#include "led_strip.hpp"
#include "web_server.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmc5240.hpp"
#include <cmath>

void teststepper(TMC5240 &drv, uint32_t data)
{
    uint8_t status = 12;
    ESP_LOGI(__FILE__, "TESTING DRIVER %d with data %x", drv.device_id(), data);
    if (drv.write(0x21, data, status) == ESP_OK)
    {
        ESP_LOGI(__FILE__, "Write with status %x", status);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t val;
    if (drv.read(0x21, val, status) == ESP_OK)
    {
        ESP_LOGI(__FILE__, "Read with status %x", status);
    }

    ESP_LOGI(__FILE__, "Sent: %x, received: %x", data, val);
}

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
    TMC5240 mot1(spi_bus, GPIO_NUM_12, MOT_1);
    TMC5240 mot2(spi_bus, GPIO_NUM_10, MOT_2);
    esp_err_t web_server_err = start_web_server();
    if (web_server_err != ESP_OK)
    {
        ESP_LOGE(__FILE__, "Failed to start web server: %s", esp_err_to_name(web_server_err));
    }

    TickType_t last_telemetry_send = 0;
    int cnt = 0;
    while (true)
    {
        ICM42670Sample sample = imu.read_sample();
        float abs = std::sqrt(sample.acc.x * sample.acc.x + sample.acc.y * sample.acc.y + sample.acc.z * sample.acc.z);
        TickType_t now = xTaskGetTickCount();
        if (now - last_telemetry_send >= pdMS_TO_TICKS(50))
        {
            TelemetrySample telemetry = {
                .acc_x = sample.acc.x,
                .acc_y = sample.acc.y,
                .acc_z = sample.acc.z,
                .gyro_x = sample.gyro.x,
                .gyro_y = sample.gyro.y,
                .gyro_z = sample.gyro.z,
            };
            web_server_publish_telemetry(telemetry);
            last_telemetry_send = now;
            teststepper(mot1, cnt++);
            teststepper(mot2, cnt++);
        }
    }
}
