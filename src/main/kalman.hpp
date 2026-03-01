#pragma once

#include "icm42670_spi.hpp"
#include "vectors.hpp"
#include "esp_dsp.h"

#include <cmath>
#include <cstdint>

namespace kalman
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;

    class Angle1DKalman
    {
    public:
        // These defaults are conservative for a 400 Hz IMU stream.
        float q_angle = 0.001f;
        float q_bias = 0.003f;
        float r_measure = 0.03f;

        void set_angle(float angle_rad)
        {
            angle_ = angle_rad;
        }

        float angle() const
        {
            return angle_;
        }

        float predict(float gyro_rate_rad_s, float dt_s)
        {
            const float unbiased_rate = gyro_rate_rad_s - bias_;
            angle_ += dt_s * unbiased_rate;

            p00_ += dt_s * (dt_s * p11_ - p01_ - p10_ + q_angle);
            p01_ -= dt_s * p11_;
            p10_ -= dt_s * p11_;
            p11_ += q_bias * dt_s;
            return angle_;
        }

        float update(float measured_angle_rad)
        {
            const float innovation = measured_angle_rad - angle_;
            const float innovation_cov = p00_ + r_measure;
            if (innovation_cov <= 1e-9f)
            {
                return angle_;
            }

            const float k0 = p00_ / innovation_cov;
            const float k1 = p10_ / innovation_cov;

            angle_ += k0 * innovation;
            bias_ += k1 * innovation;

            const float p00_prev = p00_;
            const float p01_prev = p01_;

            p00_ -= k0 * p00_prev;
            p01_ -= k0 * p01_prev;
            p10_ -= k1 * p00_prev;
            p11_ -= k1 * p01_prev;
            return angle_;
        }

    private:
        float angle_ = 0.0f;
        float bias_ = 0.0f;

        float p00_ = 1.0f;
        float p01_ = 0.0f;
        float p10_ = 0.0f;
        float p11_ = 1.0f;
    };
} // namespace kalman

class KalmanFilter
{
public:
    explicit KalmanFilter(const ICM42670Sample &initial_sample)
        : last_ts_us_(initial_sample.ts_us)
    {
        initialize_from_accel(initial_sample.acc);
    }

    explicit KalmanFilter(uint64_t initial_ts_us)
        : last_ts_us_(initial_ts_us)
    {
    }

    Quaternion process(const ICM42670Sample &sample)
    {
        float dt = static_cast<float>(sample.ts_us - last_ts_us_) / 1'000'000.0f;
        last_ts_us_ = sample.ts_us;

        const Vec<3> gyro_rad = sample.gyro * kalman::kDegToRad;
        roll_filter_.predict(gyro_rad[0], dt);
        pitch_filter_.predict(gyro_rad[1], dt);

        const float acc_norm = sample.acc.abs();
        const bool accel_is_reasonable = (acc_norm > 0.2f) && (acc_norm < 3.0f);
        if (accel_is_reasonable)
        {
            const float ax = sample.acc[0];
            const float ay = sample.acc[1];
            const float az = sample.acc[2];

            const float measured_roll = std::atan2(ay, az);
            const float measured_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

            roll_filter_.update(measured_roll);
            pitch_filter_.update(measured_pitch);
        }

        orientation_ = euler_to_quaternion(roll_filter_.angle(), pitch_filter_.angle(), yaw_rad_);
        return orientation_;
    }

    const Quaternion &orientation() const
    {
        return orientation_;
    }

private:
    static Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
    {
        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);
        const float cy = std::cos(yaw * 0.5f);
        const float sy = std::sin(yaw * 0.5f);

        Quaternion q{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        };
        const float n = q.abs();
        if (n <= 1e-8f)
        {
            return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
        }
        const Vec<4> normalized = q / n;
        return Quaternion{normalized[0], normalized[1], normalized[2], normalized[3]};
    }

    void initialize_from_accel(const Vec<3> &acc)
    {
        const float ax = acc[0];
        const float ay = acc[1];
        const float az = acc[2];
        const float roll = std::atan2(ay, az);
        const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        roll_filter_.set_angle(roll);
        pitch_filter_.set_angle(pitch);
        yaw_rad_ = 0.0f;
        orientation_ = euler_to_quaternion(roll, pitch, yaw_rad_);
    }

    uint64_t last_ts_us_ = 0;
    kalman::Angle1DKalman roll_filter_;
    kalman::Angle1DKalman pitch_filter_;
    float yaw_rad_ = 0.0f;
    Quaternion orientation_{1.0f, 0.0f, 0.0f, 0.0f};
};
