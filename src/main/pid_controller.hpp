#pragma once

#include <cmath>

struct PidGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

class PIDController
{
public:
    explicit PIDController(PidGains gains = {}) : gains_(gains) {}

    void set_gains(const PidGains &gains)
    {
        gains_ = gains;
    }

    const PidGains &gains() const
    {
        return gains_;
    }

    void reset()
    {
        last_error_ = 0.0f;
        output_ = 0.0f;
        error_integral_ = 0.0f;
        initialized_ = false;
    }

    float update(float target, float measured, float dt_s)
    {
        return update_from_error(target - measured, dt_s);
    }

    float update_from_error(float error, float dt_s)
    {
        if (!(dt_s > 0.0f) || !std::isfinite(dt_s))
        {
            return output_;
        }

        const float derivative = initialized_ ? (error - last_error_) / dt_s : 0.0f;
        error_integral_ += error * dt_s;
        last_error_ = error;
        initialized_ = true;
        output_ = gains_.kp * error + gains_.ki * error_integral_ + gains_.kd * derivative;
        return output_;
    }

private:
    PidGains gains_ = {};
    float last_error_ = 0.0f;
    float output_ = 0.0f;
    float error_integral_ = 0.0f;
    bool initialized_ = false;
};
