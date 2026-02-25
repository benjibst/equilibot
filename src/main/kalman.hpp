#include "icm42670_spi.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

float degree_to_radian(float degree)
{
    return degree * M_PI / 180;
}
class KalmanFilter
{
public:
    float process(const ICM42670Sample &data)
    {
        uint64_t dt = data.ts_us - last_time_us;
        float ds = dt / 1000000.0f;

        omega_z = degree_to_radian(data.gyro[2]);

        theta_m = theta_m - omega_z * ds;
        theta_s = atan2f(data.acc[1], data.acc[0]);

        P_theta_n = P_theta_p + Q;
        K_theta = P_theta_n / (P_theta_n + R);
        theta_n = theta_p - ds * omega_z;
        theta_p = theta_n + K_theta * (theta_s - theta_n);
        P_theta_p = (1 - K_theta) * P_theta_n;
        return theta_p;
    }

private:
    uint64_t last_time_us = 0;
    float omega_z;       // angular velocity in z
    float theta_m;       // pitch angle model prediction
    float theta_s;       // pitch angle measurement
    float theta_n;       // pitch angle estimation a priori
    float theta_p = 0;   // pitch angle estimation a posteriori = 0
    float P_theta_n;     // a priori covariance of Theta
    float P_theta_p = 0; // a posteriori covariance of Theta
    float K_theta;       // Observer gain or Kalman gain for Theta
    float Q = 0.1;       // Covariance of disturbance (unknown disturbance affecting w_x and w_y)
    float R = 4;         // Covariance of noise (unknown noise affecting theta_s and phi_s)
};
class GyroIntegrator
{
public:
    float process(const ICM42670Sample &data)
    {
        uint64_t dt = data.ts_us - last_time_us;
        float ds = dt / 1000000.0f;
        float omega_z = degree_to_radian(data.gyro[2]);
        theta += omega_z * ds;
        return theta;
    }
    uint64_t last_time_us = 0;

    float theta = 0;
};