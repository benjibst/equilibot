#include "led_strip.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include <algorithm>
#include <cmath>

namespace
{

    constexpr char kTag[] = "EquilibotLedStrip";
    constexpr uint32_t kRmtResolutionHz = 10 * 1000 * 1000;
    constexpr size_t kRmtMemBlockSymbols = 64;

    constexpr Pixel kOff{0, 0, 0};
    constexpr Pixel kTiltColor{5, 5, 5};
    constexpr Pixel kBatteryColor{0, 5, 0};
    constexpr Pixel kMotorColor{5, 2, 0};
    constexpr Pixel kConnectionColor{0, 0, 5};

    bool segment_is_in_bounds(const LedSegment &segment, uint16_t strip_led_count)
    {
        return segment.start_index <= strip_led_count &&
               segment.led_count <= static_cast<uint16_t>(strip_led_count - segment.start_index);
    }

    bool segments_overlap(const LedSegment &a, const LedSegment &b)
    {
        if (a.led_count == 0 || b.led_count == 0)
        {
            return false;
        }

        const uint32_t a_start = a.start_index;
        const uint32_t a_end = a_start + a.led_count;
        const uint32_t b_start = b.start_index;
        const uint32_t b_end = b_start + b.led_count;

        return a_start < b_end && b_start < a_end;
    }

    bool config_has_overlapping_segments(const LedStripConfig &config)
    {
        const LedSegment segments[] = {
            config.tilt_segment,
            config.battery_segment,
            config.motor1_segment,
            config.connection_segment,
            config.motor2_segment,
        };
        constexpr size_t kSegmentCount = sizeof(segments) / sizeof(segments[0]);

        for (size_t i = 0; i < kSegmentCount; ++i)
        {
            for (size_t j = i + 1; j < kSegmentCount; ++j)
            {
                if (segments_overlap(segments[i], segments[j]))
                {
                    return true;
                }
            }
        }

        return false;
    }

    bool config_is_valid(const LedStripConfig &config)
    {
        if (config.led_count == 0)
        {
            return false;
        }

        const bool in_bounds =
            segment_is_in_bounds(config.tilt_segment, config.led_count) &&
            segment_is_in_bounds(config.battery_segment, config.led_count) &&
            segment_is_in_bounds(config.motor1_segment, config.led_count) &&
            segment_is_in_bounds(config.connection_segment, config.led_count) &&
            segment_is_in_bounds(config.motor2_segment, config.led_count);
        if (!in_bounds)
        {
            return false;
        }

        return !config_has_overlapping_segments(config);
    }

    void fill_segment(std::span<Pixel> segment, Pixel color)
    {
        std::fill(segment.begin(), segment.end(), color);
    }

    Pixel scale_pixel(Pixel color, float brightness)
    {
        const float clamped = std::clamp(brightness, 0.0f, 1.0f);
        return Pixel{
            static_cast<uint8_t>(std::lround(static_cast<float>(color.r) * clamped)),
            static_cast<uint8_t>(std::lround(static_cast<float>(color.g) * clamped)),
            static_cast<uint8_t>(std::lround(static_cast<float>(color.b) * clamped)),
        };
    }

} // namespace

EquilibotLedStrip::EquilibotLedStrip(gpio_num_t led_gpio, const LedStripConfig &config)
    : config(config), pixels(config.led_count, kOff)
{
    if (!config_is_valid(this->config))
    {
        ESP_LOGE(kTag, "Invalid LED strip configuration");
        return;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = led_gpio,
        .max_leds = this->config.led_count,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = kRmtResolutionHz,
        .mem_block_symbols = kRmtMemBlockSymbols,
        .flags = {
            .with_dma = true,
        },
    };

    const esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed to create LED strip: %s", esp_err_to_name(err));
        led_strip = nullptr;
        return;
    }
}

EquilibotLedStrip::~EquilibotLedStrip()
{
    if (led_strip != nullptr)
    {
        led_strip_del(led_strip);
    }
}

std::span<Pixel> EquilibotLedStrip::segment_pixels(const LedSegment &segment)
{
    if (segment.start_index >= pixels.size())
    {
        return {};
    }

    const size_t available = pixels.size() - segment.start_index;
    const size_t count = std::min<size_t>(segment.led_count, available);
    return {pixels.data() + segment.start_index, count};
}

void EquilibotLedStrip::update()
{
    LockGuard lock(led_strip_mutex);
    update_locked();
}

void EquilibotLedStrip::update_locked()
{
    if (led_strip == nullptr)
    {
        return;
    }

    for (size_t i = 0; i < pixels.size(); ++i)
    {
        const Pixel &pixel = pixels[i];
        const esp_err_t err = led_strip_set_pixel(led_strip, i, pixel.r, pixel.g, pixel.b);
        if (err != ESP_OK)
        {
            ESP_LOGE(kTag, "led_strip_set_pixel failed at %u: %s", static_cast<unsigned>(i),
                     esp_err_to_name(err));
            break;
        }
    }

    const esp_err_t refresh_err = led_strip_refresh(led_strip);
    if (refresh_err != ESP_OK)
    {
        ESP_LOGE(kTag, "led_strip_refresh failed: %s", esp_err_to_name(refresh_err));
    }
}

void EquilibotLedStrip::set_motor_segment(std::span<Pixel> segment, float speed)
{
    if (segment.empty())
    {
        return;
    }

    fill_segment(segment, kOff);
    const size_t center = segment.size() / 2;
    segment[center] = kMotorColor;

    const float clamped_speed = std::clamp(speed, -1.0f, 1.0f);
    const float magnitude = std::fabs(clamped_speed);
    if (magnitude == 0.0f)
    {
        return;
    }

    const size_t left_capacity = center;
    const size_t right_capacity = segment.size() - center - 1;

    if (clamped_speed > 0.0f && right_capacity > 0)
    {
        size_t extra = static_cast<size_t>(std::ceil(magnitude * right_capacity));
        extra = std::min(extra, right_capacity);
        for (size_t i = 1; i <= extra; ++i)
        {
            segment[center + i] = kMotorColor;
        }
    }
    else if (clamped_speed < 0.0f && left_capacity > 0)
    {
        size_t extra = static_cast<size_t>(std::ceil(magnitude * left_capacity));
        extra = std::min(extra, left_capacity);
        for (size_t i = 1; i <= extra; ++i)
        {
            segment[center - i] = kMotorColor;
        }
    }
}

void EquilibotLedStrip::set_tilt(float tilt)
{

    LockGuard lock(led_strip_mutex);

    const auto segment = segment_pixels(config.tilt_segment);
    if (segment.empty())
    {
        return;
    }

    fill_segment(segment, kOff);

    if (segment.size() == 1)
    {
        segment[0] = kTiltColor;
        return;
    }

    const float clamped_tilt = std::clamp(tilt, -90.0f, 90.0f);
    const float normalized = (clamped_tilt + 90.0f) / 180.0f;
    const float scaled = normalized * static_cast<float>(segment.size() - 1);

    const size_t lower_index = static_cast<size_t>(std::floor(scaled));
    const size_t upper_index = std::min(lower_index + 1, segment.size() - 1);
    const float upper_weight = scaled - static_cast<float>(lower_index);
    const float lower_weight = 1.0f - upper_weight;

    segment[lower_index] = scale_pixel(kTiltColor, lower_weight);
    segment[upper_index] = scale_pixel(kTiltColor, upper_weight);
}

void EquilibotLedStrip::set_battery_level(int level_percent)
{

    LockGuard lock(led_strip_mutex);

    const auto segment = segment_pixels(config.battery_segment);
    if (segment.empty())
    {
        return;
    }

    fill_segment(segment, kOff);

    const int clamped_percent = std::clamp(level_percent, 0, 100);
    size_t lit_count =
        static_cast<size_t>(std::lround(static_cast<float>(clamped_percent) *
                                        static_cast<float>(segment.size()) / 100.0f));
    lit_count = std::min(lit_count, segment.size());

    for (size_t i = 0; i < lit_count; ++i)
    {
        segment[i] = kBatteryColor;
    }
}

void EquilibotLedStrip::set_motor_speed(float speed1, float speed2)
{

    LockGuard lock(led_strip_mutex);

    set_motor_segment(segment_pixels(config.motor1_segment), speed1);
    set_motor_segment(segment_pixels(config.motor2_segment), speed2);
}

void EquilibotLedStrip::set_connection_status(bool connected)
{

    LockGuard lock(led_strip_mutex);

    const auto segment = segment_pixels(config.connection_segment);
    if (segment.empty())
    {
        return;
    }

    fill_segment(segment, connected ? kConnectionColor : kOff);
}
