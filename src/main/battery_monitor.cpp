#include "battery_monitor.hpp"

#include "led_strip.hpp"

#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include <algorithm>
#include <cmath>

namespace
{

    constexpr char kTag[] = "BatteryMonitor";
    constexpr gpio_num_t kBatteryAdcPin = GPIO_NUM_4;
    constexpr adc_atten_t kBatteryAdcAtten = ADC_ATTEN_DB_12;
    constexpr uint64_t kBatteryMeasurePeriodUs = 10ULL * 1000ULL * 1000ULL;

    // BAT_MON is derived from VM through a 12k / 3k divider, then filtered by 5.6k + 220n into IO4.
    constexpr float kBatteryDividerRatio = 5.0f;
    constexpr float kBatteryVoltageMinV = 9.0f;
    constexpr float kBatteryVoltageMaxV = 12.6f;

} // namespace

BatteryMonitor::~BatteryMonitor()
{
    cleanup();
}

bool BatteryMonitor::create_adc_calibration()
{
    adc_cali_scheme_ver_t scheme_mask = static_cast<adc_cali_scheme_ver_t>(0);
    if (adc_cali_check_scheme(&scheme_mask) != ESP_OK)
    {
        return false;
    }

    if ((scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) != 0)
    {
        const adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = adc_unit_,
            .chan = adc_channel_,
            .atten = kBatteryAdcAtten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_) == ESP_OK)
        {
            cali_scheme_ = ADC_CALI_SCHEME_VER_CURVE_FITTING;
            return true;
        }
    }

    return false;
}

float BatteryMonitor::read_battery_voltage(esp_err_t &err) const
{
    int voltage_mv = 0;
    if (cali_enabled_)
    {
        err = adc_oneshot_get_calibrated_result(adc_handle_, cali_handle_, adc_channel_, &voltage_mv);
    }
    else
    {
        int raw = 0;
        err = adc_oneshot_read(adc_handle_, adc_channel_, &raw);
        if (err == ESP_OK)
        {
            voltage_mv = static_cast<int>(std::lround(static_cast<float>(raw) * 3300.0f / 4095.0f));
        }
    }

    if (err != ESP_OK)
    {
        return 0.0f;
    }

    return (static_cast<float>(voltage_mv) / 1000.0f) * kBatteryDividerRatio;
}

void BatteryMonitor::update_led_strip()
{

    esp_err_t read_err = ESP_OK;
    const float battery_voltage = read_battery_voltage(read_err);
    if (read_err != ESP_OK)
    {
        ESP_LOGW(kTag, "Failed reading battery ADC: %s", esp_err_to_name(read_err));
        return;
    }

    const float span = kBatteryVoltageMaxV - kBatteryVoltageMinV;
    const float normalized = (span > 0.0f) ? ((battery_voltage - kBatteryVoltageMinV) / span) : 0.0f;
    const int battery_percent = static_cast<int>(std::lround(std::clamp(normalized, 0.0f, 1.0f) * 100.0f));

    led_strip_.set_battery_level(battery_percent);
    led_strip_.update();
    ESP_LOGI(kTag, "Battery %.2f V -> %d%%", battery_voltage, battery_percent);
}

void BatteryMonitor::cleanup()
{
    if (timer_ != nullptr)
    {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

    if (cali_enabled_ && cali_handle_ != nullptr)
    {
        if (cali_scheme_ == ADC_CALI_SCHEME_VER_CURVE_FITTING)
        {
            adc_cali_delete_scheme_curve_fitting(cali_handle_);
        }
    }
    cali_handle_ = nullptr;
    cali_scheme_ = static_cast<adc_cali_scheme_ver_t>(0);
    cali_enabled_ = false;

    if (adc_handle_ != nullptr)
    {
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
    }
}

void BatteryMonitor::timer_callback(void *arg)
{
    auto *self = static_cast<BatteryMonitor *>(arg);
    if (self != nullptr)
    {
        self->update_led_strip();
    }
}

esp_err_t BatteryMonitor::start()
{
    cleanup();

    esp_err_t err = adc_oneshot_io_to_channel(kBatteryAdcPin, &adc_unit_, &adc_channel_);
    if (err != ESP_OK)
    {
        cleanup();
        return err;
    }

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = adc_unit_,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&unit_config, &adc_handle_);
    if (err != ESP_OK)
    {
        cleanup();
        return err;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = kBatteryAdcAtten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle_, adc_channel_, &channel_config);
    if (err != ESP_OK)
    {
        cleanup();
        return err;
    }

    cali_enabled_ = create_adc_calibration();
    if (!cali_enabled_)
    {
        ESP_LOGW(kTag, "ADC calibration unavailable, using approximate scaling for battery monitor");
    }

    const esp_timer_create_args_t timer_args = {
        .callback = &BatteryMonitor::timer_callback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "battery_mon",
        .skip_unhandled_events = true,
    };
    err = esp_timer_create(&timer_args, &timer_);
    if (err != ESP_OK)
    {
        cleanup();
        return err;
    }

    update_led_strip();
    err = esp_timer_start_periodic(timer_, kBatteryMeasurePeriodUs);
    if (err != ESP_OK)
    {
        cleanup();
        return err;
    }

    return ESP_OK;
}
