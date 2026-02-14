#include "led_strip.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <cmath>
[[maybe_unused]] void run_led_strip()
{
    EquilibotLedStrip led_strip(GPIO_NUM_38,
                                {
                                    .led_count = 31,
                                    .tilt_segment = {.start_index = 0, .led_count = 19},
                                    .battery_segment = {.start_index = 19, .led_count = 5},
                                    .motor1_segment = {.start_index = 24, .led_count = 3},
                                    .connection_segment = {.start_index = 27, .led_count = 1},
                                    .motor2_segment = {.start_index = 28, .led_count = 3},
                                });
    constexpr float kTwoPi = 6.28318530718f;
    constexpr float kTiltPeriodSec = 4.0f;
    constexpr float kBatteryPeriodSec = 6.0f;
    constexpr float kMotorPeriodSec = 2.0f;
    constexpr TickType_t kUpdatePeriod = pdMS_TO_TICKS(50);
    while (true)
    {
        const float t = static_cast<float>(esp_timer_get_time()) / 1000000.0f;

        const float tilt = 90.0f * std::sinf(kTwoPi * t / kTiltPeriodSec);
        const float battery_norm = 0.5f * (std::sinf(kTwoPi * t / kBatteryPeriodSec) + 1.0f);
        const int battery_percent = static_cast<int>(std::lround(100.0f * battery_norm));

        const float motor1 = std::sinf(kTwoPi * t / kMotorPeriodSec);
        const float motor2 = -motor1;

        const std::uint64_t seconds = static_cast<std::uint64_t>(t);
        const bool connected = (seconds % 2U) == 0U;

        led_strip.set_tilt(tilt);
        led_strip.set_battery_level(battery_percent);
        led_strip.set_motor_speed(motor1, motor2);
        led_strip.set_connection_status(connected);

        vTaskDelay(kUpdatePeriod);
    }
}