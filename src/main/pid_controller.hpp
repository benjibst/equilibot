
#pragma once
#include "esp_timer.h"
class PIDController
{
public:
    PIDController(float p, float i, float d, int64_t time_start) : p(p), i(i), d(d), last_time(time_start) {}
    float update(float target, float out_measured)
    {
        int64_t time = esp_timer_get_time();
        float dt_s = (time - last_time) / 1000000.0f;
        if (dt_s <= 0.0f)
        {
            return output;
        }
        float error = target - out_measured;
        float error_change = (error - last_error) / dt_s;

        error_i += error * dt_s;
        last_time = time;
        last_error = error;
        output = p * error + i * error_i + d * error_change;
        return output;
    }
    float target{0};
    float last_error{0};
    float output{0};
    float error_i{0};
    float p{0}, i{0}, d{0};
    int64_t last_time{0};
};