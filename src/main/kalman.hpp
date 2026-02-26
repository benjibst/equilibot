#include "icm42670_spi.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include "vectors.hpp"

class GyroIntegrator
{
public:
    GyroIntegrator(uint64_t initial_ts_us) : last_ts_us(initial_ts_us) {}
    Quaternion process(const ICM42670Sample &sample)
    {
        float dt = (sample.ts_us - last_ts_us) / 1'000'000.0f;
        Vec3 gyro = Vec3(sample.gyro) * (M_PI / 180.0f);
        float theta = gyro.abs() * dt;
        Vec3 v = gyro / gyro.abs();
        Quaternion delta_q{std::cos(theta / 2), v.x() * std::sin(theta / 2), v.y() * std::sin(theta / 2), v.z() * std::sin(theta / 2)};
        orientation = delta_q * orientation;
        last_ts_us = sample.ts_us;
        return orientation;
    }
    uint64_t last_ts_us = 0;
    Quaternion orientation{1, 0, 0, 0};
};