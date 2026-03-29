#include "spi_bus.hpp"
#include "battery_monitor.hpp"
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
#include "kalman.hpp"
#include "pid_controller.hpp"
#include <cmath>

namespace
{
    constexpr char kTag[] = "app_main";

} // namespace

extern "C" void app_main(void)
{
    SpiBus spi_bus(SPI2_HOST, {.miso = GPIO_NUM_13, .mosi = GPIO_NUM_14, .sclk = GPIO_NUM_21}, NUM_DEVICES);
    ICM42670Config imu_config = {
        .acce_odr = ICM42670_ACCE_ODR_400HZ,
        .gyro_odr = ICM42670_GYRO_ODR_400HZ,
        .acce_fs = ICM42670_ACCE_FS_2G,
        .gyro_fs = ICM42670_GYRO_FS_250DPS,
        .acce_bw = ICM42670_UI_FILT_BW_180HZ,
        .gyro_bw = ICM42670_UI_FILT_BW_180HZ,
        .install = ICM42670InstallDirection{.remap = {2, 0, 1}},
    };
    ICM42670Spi imu(spi_bus, GPIO_NUM_41, imu_config, GPIO_NUM_40);
    LedStripConfig led_strip_config = {
        .led_count = 31,
        .tilt_segment = {.start_index = 0, .led_count = 19},
        .battery_segment = {.start_index = 19, .led_count = 5},
        .motor1_segment = {.start_index = 24, .led_count = 3},
        .connection_segment = {.start_index = 27, .led_count = 1},
        .motor2_segment = {.start_index = 28, .led_count = 3},
    };
    EquilibotLedStrip led_strip(GPIO_NUM_38, led_strip_config);
    TMC5240 mot1(spi_bus, GPIO_NUM_11, GPIO_NUM_12, MOT_1, 13.0f, 1.5f);
    TMC5240 mot2(spi_bus, GPIO_NUM_9, GPIO_NUM_10, MOT_2, 13.0f, 1.5f);
    BatteryMonitor battery_monitor{led_strip};
    PIDController pitch_controller{10.0f, 0.0f, 0.0f, esp_timer_get_time()};
    ICM42670Sample sample{};
    imu.receive_sample(sample, portMAX_DELAY);
    KalmanFilter attitude_filter(sample);
    WebServer web_server(imu, imu_config, mot1, mot2);

    while (true)
    {
        imu.receive_sample(sample, portMAX_DELAY);
        Quaternion orientation = attitude_filter.process(sample);
        float angle_deg = attitude_filter.pitch_filter_.angle() * 180 / M_PI;
        led_strip.set_tilt(angle_deg);
        led_strip.update();

        if (-30.0f < angle_deg && angle_deg < 30.0f)
        {
            float out = pitch_controller.update(0.0f, angle_deg);
            out = std::clamp(out, -50.0f, 50.0f);
            mot1.set_velocity(out);
            mot2.set_velocity(out);
        }
        else
        {
            mot1.set_velocity(0);
            mot2.set_velocity(0);
        }

        web_server.queue_imu_data(WebServer::TelemetryData{sample, orientation});
    }
}
