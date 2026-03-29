#pragma once

#include <cstdint>

enum class ControllerType
{
    CascadedPid,
    Lqr,
};

struct ControllerState
{
    float position = 0.0f;
    float velocity = 0.0f;
    float pitch_deg = 0.0f;
    float pitch_rate_deg_s = 0.0f;
    uint64_t timestamp_us = 0;
};

class Controller
{
public:
    virtual ~Controller() = default;

    virtual ControllerType type() const = 0;
    virtual float update(const ControllerState &state) = 0;
    virtual void reset() = 0;
    virtual void set_target_position(float target_position) = 0;
    virtual float target_position() const = 0;
};
