#pragma once

#include "driver/gpio.h"
#include "lock_guard.hpp"
#include "led_strip.h"

#include <cstdint>
#include <span>
#include <vector>

struct Pixel
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct LedSegment
{
    uint16_t start_index;
    uint16_t led_count;
};

struct LedStripConfig
{
    uint16_t led_count;
    LedSegment tilt_segment;
    LedSegment battery_segment;
    LedSegment motor1_segment;
    LedSegment connection_segment;
    LedSegment motor2_segment;
};

class EquilibotLedStrip
{
public:
    EquilibotLedStrip(gpio_num_t led_gpio, const LedStripConfig &config);
    ~EquilibotLedStrip();

    void set_tilt(float tilt);
    void set_battery_level(int level_percent);
    void set_motor_speed(float speed1, float speed2);
    void set_connection_status(bool connected);

    EquilibotLedStrip(const EquilibotLedStrip &) = delete;
    EquilibotLedStrip &operator=(const EquilibotLedStrip &) = delete;
    EquilibotLedStrip(EquilibotLedStrip &&) = delete;
    EquilibotLedStrip &operator=(EquilibotLedStrip &&) = delete;

private:
    std::span<Pixel> segment_pixels(const LedSegment &segment);
    void push_pixels();
    void push_pixels_locked();
    void set_motor_segment(std::span<Pixel> segment, float speed);

    LedStripConfig config{};
    led_strip_handle_t led_strip = nullptr;
    StaticMutex led_strip_mutex;
    std::vector<Pixel> pixels{};
};
