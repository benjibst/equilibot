#include "tmc5240.hpp"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include <cmath>
#include <algorithm>
constexpr auto kTag = "tmc5240";

TMC5240::TMC5240(SpiBus &spi_bus, gpio_num_t en_pin, gpio_num_t cs_pin, int device_id, float rref_kohm, float motor_current_rms)
    : spi_bus(spi_bus), dev_id(device_id), rref_kohm(rref_kohm), motor_current_rms(motor_current_rms)
{
    // maximum ifs and rref according to datasheet
    assert(rref_kohm >= 12 && rref_kohm <= 60);
    assert(motor_current_rms <= 2.12f);

    SpiDeviceConfig devconfig = {
        .mode = SpiDeviceConfig::Mode::CPOL1_CPHA1,
        .cs = cs_pin,
        .clk_speed_hz = SPI_MASTER_FREQ_10M,
    };
    spi_bus.add_device(device_id, devconfig);
    gpio_config_t en_conf{
        .pin_bit_mask = (1ULL << static_cast<uint64_t>(en_pin)),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&en_conf));
    ESP_ERROR_CHECK(gpio_set_level(en_pin, 0));

    ESP_ERROR_CHECK(config_current());
    ESP_ERROR_CHECK(config_spreadcycle());
    ESP_ERROR_CHECK(config_motion_ctrl_vel_mode());
}

esp_err_t TMC5240::set_velocity(int32_t vel)
{
    using namespace tmc5240::registers;

    uint8_t status;
    uint32_t data;
    const uint32_t vel_abs = static_cast<uint32_t>(std::min<int64_t>(
        (vel >= 0) ? static_cast<int64_t>(vel) : -static_cast<int64_t>(vel),
        (1LL << 23) - 1));
    if (vel >= 0)
    {
        data = RAMPMODE::set_rampmode(0, 1);
        write(RAMPMODE::address, data, status);
    }
    else
    {
        data = RAMPMODE::set_rampmode(0, 2);
        write(RAMPMODE::address, data, status);
    }
    data = VMAX::set_vmax(0, vel_abs);
    write(VMAX::address, data, status);
    return ESP_OK;
}

// Rref = 13k
// Full scale current = Kifs / Rref
// DRV_CONF    Kifs    Ifs
// 00          11.75   0.639A
// 01          24      1.305A
// 10/11       36      1.958A

// Globalscaler: 0-255
// 0 -> full scale Ifs
// x -> x/256 * Ifs
// Small motors: 0.24A -> 0.24 = 0.639 * x/256 -> x = 0.24*256/0.639 = 96
constexpr float to_rms(float i)
{
    return i / M_SQRT2;
}
float ifs_rms(float kifs, float rref)
{
    return to_rms(kifs) / rref;
}
std::pair<uint32_t, float> min_drv_conf(float mot_curr_rms, float rref_kohm)
{
    float ifs_rms_;
    if (mot_curr_rms < (ifs_rms_ = ifs_rms(11.75, rref_kohm)))
        return {0b00, ifs_rms_};
    if (mot_curr_rms < (ifs_rms_ = ifs_rms(24, rref_kohm)))
        return {0b01, ifs_rms_};
    if (mot_curr_rms < (ifs_rms_ = ifs_rms(36, rref_kohm)))
        return {0b10, ifs_rms_};
    __builtin_unreachable();
}

esp_err_t TMC5240::config_current()
{
    using namespace tmc5240::registers;

    uint8_t status;
    auto [drv_conf, ifs_rms] = min_drv_conf(motor_current_rms, rref_kohm);

    uint32_t data = DRV_CONF::set_current_range(0, drv_conf);
    write(DRV_CONF::address, data, status);

    data = GLOBAL_SCALER::set_globalscaler(0, motor_current_rms * 256 / ifs_rms);
    write(GLOBAL_SCALER::address, data, status);

    data = GCONF::set_en_pwm_mode(0, 0);
    write(GCONF::address, data, status);
    return ESP_OK;
}

esp_err_t TMC5240::config_stealthchop2()
{
    using namespace tmc5240::registers;

    uint8_t status;

    uint32_t data = GCONF::set_en_pwm_mode(0, 1);
    write(GCONF::address, data, status);

    data = PWMCONF::set_pwm_autoscale(0, 1);
    data = PWMCONF::set_pwm_autograd(data, 1);
    data = PWMCONF::set_pwm_meas_sd_enable(data, 1);
    data = PWMCONF::set_pwm_freq(data, 0b00);
    write(PWMCONF::address, data, status);

    data = CHOPCONF::set_toff(0, 3);
    data = CHOPCONF::set_tbl(data, 2);
    data = CHOPCONF::set_hstrt_tfd210(data, 4);
    data = CHOPCONF::set_hend_offset(data, 0);
    write(CHOPCONF::address, data, status);

    return ESP_OK;
}

esp_err_t TMC5240::config_spreadcycle()
{
    uint8_t status;
    using namespace tmc5240::registers;
    uint32_t data = GCONF::set_en_pwm_mode(0, 0U);
    ESP_RETURN_ON_ERROR(write(GCONF::address, data, status), kTag, "Failed setting en_pwm_mode");
    return set_spreadcycle_config(5, 2, 0, 0);
}

esp_err_t TMC5240::set_spreadcycle_config(uint8_t toff, uint8_t tbl, uint8_t hstart, uint8_t hend)
{
    using namespace tmc5240::registers;

    uint8_t status = 0;
    uint32_t data = 0;

    // Datasheet-recommended spreadCycle tuning range:
    // TOFF [2..15], TBL [0..3], HSTRT [0..7], HEND [0..15]
    toff = static_cast<uint8_t>(std::clamp<uint8_t>(toff, 2, 15));
    tbl = static_cast<uint8_t>(std::min<uint8_t>(tbl, 3));
    hstart = static_cast<uint8_t>(std::min<uint8_t>(hstart, 7));
    hend = static_cast<uint8_t>(std::min<uint8_t>(hend, 15));
    ESP_LOGI(kTag,
             "Applying spreadcycle config on dev %d: TOFF=%u TBL=%u HSTART=%u HEND=%u",
             dev_id,
             static_cast<unsigned>(toff),
             static_cast<unsigned>(tbl),
             static_cast<unsigned>(hstart),
             static_cast<unsigned>(hend));

    data = CHOPCONF::set_toff(0, toff);
    data = CHOPCONF::set_tbl(data, tbl);
    data = CHOPCONF::set_hstrt_tfd210(data, hstart);
    data = CHOPCONF::set_hend_offset(data, hend);
    ESP_RETURN_ON_ERROR(write(CHOPCONF::address, data, status), kTag, "Failed writing CHOPCONF");
    return ESP_OK;
}

esp_err_t TMC5240::config_motion_ctrl_vel_mode()
{
    using namespace tmc5240::registers;

    uint8_t status;

    uint32_t data = AMAX::set_amax(0, 100);
    write(AMAX::address, data, status);
    return ESP_OK;
}

esp_err_t TMC5240::write(uint8_t addr, uint32_t data, uint8_t &spi_status)
{
    if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    {
        data = __builtin_bswap32(data);
    }
    uint8_t tx[5] = {(uint8_t)(addr | 0x80)};
    memcpy(tx + 1, &data, 4);
    uint8_t rx[5] = {0};
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag,
                        "Spi Write Failed");
    spi_status = rx[0];
    return ESP_OK;
}
esp_err_t TMC5240::read(uint8_t addr, uint32_t &val, uint8_t &spi_status)
{
    uint8_t tx[5] = {addr, 0, 0, 0, 0};
    uint8_t rx[5] = {0};
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag, "Failed Spi Read 1");
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag, "Failed Spi Read 2");
    spi_status = rx[0];
    val = rx[4] + (rx[3] << 8) + (rx[2] << 16) + (rx[1] << 24);
    return ESP_OK;
}
