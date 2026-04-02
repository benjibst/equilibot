#include "cascaded_pid_controller.hpp"

#include <algorithm>

CascadedPidController::CascadedPidController(CascadedPidParameters parameters,
                                             float max_target_pitch_deg,
                                             float max_velocity_command)
    : parameters_(parameters),
      position_controller_(parameters.position),
      pitch_controller_(parameters.pitch),
      max_target_pitch_deg_(max_target_pitch_deg),
      max_velocity_command_(max_velocity_command)
{
    snapshot_.parameters = parameters_;
}

ControllerType CascadedPidController::type() const
{
    return ControllerType::CascadedPid;
}

float CascadedPidController::update(const ControllerState &state)
{
    LockGuard lock(mutex_);

    snapshot_.target_position = target_position_;
    snapshot_.position = state.position;
    snapshot_.velocity = state.velocity;
    snapshot_.pitch_deg = state.pitch_deg;
    snapshot_.pitch_rate_deg_s = state.pitch_rate_deg_s;

    if (last_timestamp_us_ == 0 || state.timestamp_us <= last_timestamp_us_)
    {
        last_timestamp_us_ = state.timestamp_us;
        snapshot_.target_pitch_deg = 0.0f;
        snapshot_.velocity_command = 0.0f;
        return 0.0f;
    }

    const float dt_s = static_cast<float>(state.timestamp_us - last_timestamp_us_) / 1'000'000.0f;
    last_timestamp_us_ = state.timestamp_us;

    // The current IMU pitch sign and motor position sign require the outer-loop
    // correction to request the opposite body lean.
    const float target_pitch_deg = std::clamp(-position_controller_.update(target_position_, state.position, dt_s),
                                              -max_target_pitch_deg_,
                                              max_target_pitch_deg_);
    const float velocity_command = std::clamp(pitch_controller_.update(target_pitch_deg, state.pitch_deg, dt_s),
                                              -max_velocity_command_,
                                              max_velocity_command_);

    snapshot_.target_pitch_deg = target_pitch_deg;
    snapshot_.velocity_command = velocity_command;
    return velocity_command;
}

void CascadedPidController::reset()
{
    LockGuard lock(mutex_);
    reset_unlocked();
}

void CascadedPidController::set_target_position(float target_position)
{
    LockGuard lock(mutex_);
    target_position_ = target_position;
    snapshot_.target_position = target_position_;
}

float CascadedPidController::target_position() const
{
    LockGuard lock(mutex_);
    return target_position_;
}

CascadedPidParameters CascadedPidController::parameters() const
{
    LockGuard lock(mutex_);
    return parameters_;
}

void CascadedPidController::set_parameters(const CascadedPidParameters &parameters)
{
    LockGuard lock(mutex_);
    parameters_ = parameters;
    position_controller_.set_gains(parameters_.position);
    pitch_controller_.set_gains(parameters_.pitch);
    snapshot_.parameters = parameters_;
    reset_unlocked();
}

CascadedPidSnapshot CascadedPidController::snapshot() const
{
    LockGuard lock(mutex_);
    return snapshot_;
}

void CascadedPidController::reset_unlocked()
{
    position_controller_.reset();
    pitch_controller_.reset();
    last_timestamp_us_ = 0;
    snapshot_.target_position = target_position_;
    snapshot_.target_pitch_deg = 0.0f;
    snapshot_.velocity_command = 0.0f;
}
