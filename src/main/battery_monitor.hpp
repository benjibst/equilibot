#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "util.hpp"
#include "esp_log.h"
#include "led_strip.hpp"
class EquilibotLedStrip;

class BatteryMonitor
{
public:
    BatteryMonitor(EquilibotLedStrip &led_strip) : led_strip_(led_strip)
    {
        const esp_err_t battery_monitor_err = start();
        if (battery_monitor_err != ESP_OK)
        {
            LOG_E("Failed starting battery monitor: %s", esp_err_to_name(battery_monitor_err));
        }
    };
    ~BatteryMonitor();

    esp_err_t start();

    BatteryMonitor(const BatteryMonitor &) = delete;
    BatteryMonitor &operator=(const BatteryMonitor &) = delete;
    BatteryMonitor(BatteryMonitor &&) = delete;
    BatteryMonitor &operator=(BatteryMonitor &&) = delete;

private:
    bool create_adc_calibration();
    float read_battery_voltage(esp_err_t &err) const;
    void update_led_strip();
    void cleanup();
    static void timer_callback(void *arg);

    EquilibotLedStrip &led_strip_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_unit_t adc_unit_ = ADC_UNIT_1;
    adc_channel_t adc_channel_ = ADC_CHANNEL_0;
    adc_cali_handle_t cali_handle_ = nullptr;
    adc_cali_scheme_ver_t cali_scheme_ = static_cast<adc_cali_scheme_ver_t>(0);
    bool cali_enabled_ = false;
    esp_timer_handle_t timer_ = nullptr;
};
