#pragma once

#include "controller.hpp"
#include "freertos_wrappers.hpp"
#include "pid_controller.hpp"

struct CascadedPidParameters
{
    PidGains position = {};
    PidGains pitch = {};
};

struct CascadedPidSnapshot
{
    CascadedPidParameters parameters = {};
    float target_position = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float pitch_deg = 0.0f;
    float pitch_rate_deg_s = 0.0f;
    float target_pitch_deg = 0.0f;
    float velocity_command = 0.0f;
};

class CascadedPidController : public Controller
{
public:
    CascadedPidController(CascadedPidParameters parameters,
                          float max_target_pitch_deg = 12.0f,
                          float max_velocity_command = 200.0f);

    ControllerType type() const override;
    float update(const ControllerState &state) override;
    void reset() override;
    void set_target_position(float target_position) override;
    float target_position() const override;

    CascadedPidParameters parameters() const;
    void set_parameters(const CascadedPidParameters &parameters);
    CascadedPidSnapshot snapshot() const;

private:
    void reset_unlocked();

    mutable StaticMutex mutex_;
    CascadedPidParameters parameters_ = {};
    PIDController position_controller_;
    PIDController pitch_controller_;
    float target_position_ = 0.0f;
    float max_target_pitch_deg_ = 12.0f;
    float max_velocity_command_ = 200.0f;
    uint64_t last_timestamp_us_ = 0;
    CascadedPidSnapshot snapshot_ = {};
};
