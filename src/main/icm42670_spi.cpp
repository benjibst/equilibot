#include "icm42670_spi.hpp"
#include "common.hpp"

#include "esp_log.h"
#include "esp_rom_sys.h"

#include <array>

namespace
{

    constexpr char kTag[] = "ICM42670Spi";

    constexpr uint32_t kSpiClockHz = 1000 * 1000;
    constexpr uint8_t kReadMask = 0x80;

    constexpr uint8_t kWhoAmI = 0x75;
    constexpr uint8_t kPwrMgmt0 = 0x1F;
    constexpr uint8_t kGyroConfig0 = 0x20;
    constexpr uint8_t kAccelConfig0 = 0x21;
    constexpr uint8_t kGyroConfig1 = 0x23;
    constexpr uint8_t kAccelConfig1 = 0x24;
    constexpr uint8_t kAccelData = 0x0B;
    constexpr uint8_t kGyroData = 0x11;

    constexpr uint8_t kIcm42670Id = 0x67;

    constexpr float kGyroFs2000Sensitivity = 16.4f;
    constexpr float kGyroFs1000Sensitivity = 32.8f;
    constexpr float kGyroFs500Sensitivity = 65.5f;
    constexpr float kGyroFs250Sensitivity = 131.0f;

    constexpr float kAcceFs16gSensitivity = 2048.0f;
    constexpr float kAcceFs8gSensitivity = 4096.0f;
    constexpr float kAcceFs4gSensitivity = 8192.0f;
    constexpr float kAcceFs2gSensitivity = 16384.0f;

} // namespace

ICM42670Spi::ICM42670Spi(SpiBus &spi_bus, gpio_num_t cs_pin, const ICM42670Config &config)
    : spi_bus(spi_bus), config(config)
{
    const esp_err_t add_ret = spi_bus.add_device(EquiliBotSpiDevices::IMU,
                                                 {.mode = SpiDeviceConfig::Mode::CPOL1_CPHA1,
                                                  .cs = cs_pin,
                                                  .clk_speed_hz = kSpiClockHz});
    if (add_ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed creating ICM42670 SPI device");
        return;
    }

    const esp_err_t init_ret = initialize();
    if (init_ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed initializing ICM42670: %s", esp_err_to_name(init_ret));
        return;
    }

    initialized = true;
}

ICM42670Sample ICM42670Spi::read_sample()
{
    ICM42670Sample sample{};
    if (!initialized)
    {
        return sample;
    }

    esp_err_t ret = get_acce_value(sample.acc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed reading accelerometer value: %s", esp_err_to_name(ret));
    }

    ret = get_gyro_value(sample.gyro);
    if (ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed reading gyroscope value: %s", esp_err_to_name(ret));
    }

    return sample;
}

ICM42670Spi::~ICM42670Spi() {}

esp_err_t ICM42670Spi::initialize()
{
    esp_err_t ret = check_device_present();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = configure_sensor(config.config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = configure_low_pass_filters(config.acce_ui_filt_bw, config.gyro_ui_filt_bw);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t power_mgmt = 0;
    ret = read_register(kPwrMgmt0, power_mgmt);
    if (ret != ESP_OK)
    {
        return ret;
    }

    power_mgmt = static_cast<uint8_t>((power_mgmt & ~0x0F) |
                                      ((static_cast<uint8_t>(config.gyro_pwr) & 0x03) << 2) |
                                      (static_cast<uint8_t>(config.acce_pwr) & 0x03));
    ret = write_register(kPwrMgmt0, power_mgmt);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Datasheet: after transitioning Accel/Gyro from OFF to any active mode, avoid
    // issuing register writes for at least 200 us.
    esp_rom_delay_us(200);
    return ESP_OK;
}

esp_err_t ICM42670Spi::check_device_present()
{
    uint8_t device_id = 0;
    esp_err_t ret = ESP_FAIL;
    for (int i = 0; i < 5 && ret != ESP_OK; ++i)
    {
        ret = read_register(kWhoAmI, device_id);
    }
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (device_id != kIcm42670Id)
    {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t ICM42670Spi::configure_sensor(const icm42670_cfg_t &cfg)
{
    uint8_t config_data[2] = {
        static_cast<uint8_t>(((cfg.gyro_fs & 0x03) << 5) | (cfg.gyro_odr & 0x0F)),
        static_cast<uint8_t>(((cfg.acce_fs & 0x03) << 5) | (cfg.acce_odr & 0x0F)),
    };

    return write_registers(kGyroConfig0, config_data, sizeof(config_data));
}

esp_err_t ICM42670Spi::configure_low_pass_filters(icm42670_acce_ui_filt_bw_t acce_bw,
                                                  icm42670_gyro_ui_filt_bw_t gyro_bw)
{
    if ((static_cast<uint8_t>(acce_bw) > 0x07) || (static_cast<uint8_t>(gyro_bw) > 0x07))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_value = 0;
    esp_err_t ret = read_register(kGyroConfig1, reg_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    reg_value = static_cast<uint8_t>((reg_value & ~0x07) |
                                     (static_cast<uint8_t>(gyro_bw) & 0x07));
    ret = write_register(kGyroConfig1, reg_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register(kAccelConfig1, reg_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Preserve ACCEL_UI_AVG[6:4] so low-power moving-average behavior is unchanged.
    reg_value = static_cast<uint8_t>((reg_value & ~0x07) |
                                     (static_cast<uint8_t>(acce_bw) & 0x07));
    return write_register(kAccelConfig1, reg_value);
}

esp_err_t ICM42670Spi::set_acce_power(icm42670_acce_pwr_t state)
{
    uint8_t value = 0;
    esp_err_t ret = read_register(kPwrMgmt0, value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value = static_cast<uint8_t>((value & ~0x03) |
                                 (static_cast<uint8_t>(state) & 0x03));
    return write_register(kPwrMgmt0, value);
}

esp_err_t ICM42670Spi::set_gyro_power(icm42670_gyro_pwr_t state)
{
    uint8_t value = 0;
    esp_err_t ret = read_register(kPwrMgmt0, value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value = static_cast<uint8_t>((value & ~(0x03 << 2)) |
                                 ((static_cast<uint8_t>(state) & 0x03) << 2));
    return write_register(kPwrMgmt0, value);
}

esp_err_t ICM42670Spi::write_register(uint8_t reg, uint8_t value)
{
    return write_registers(reg, &value, 1);
}

esp_err_t ICM42670Spi::write_registers(uint8_t reg, const uint8_t *values, size_t count)
{
    if (values == nullptr || count == 0 || count > 4)
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, 5> tx{};
    tx[0] = reg;
    for (size_t i = 0; i < count; ++i)
    {
        tx[i + 1] = values[i];
    }

    return spi_bus.transfer(EquiliBotSpiDevices::IMU, tx.data(), nullptr, count + 1);
}

esp_err_t ICM42670Spi::read_register(uint8_t reg, uint8_t &value)
{
    return read_registers(reg, &value, 1);
}

esp_err_t ICM42670Spi::read_registers(uint8_t reg, uint8_t *values, size_t count)
{
    if (values == nullptr || count == 0 || count > 6)
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, 7> tx{};
    std::array<uint8_t, 7> rx{};
    tx[0] = static_cast<uint8_t>(reg | kReadMask);

    const esp_err_t ret = spi_bus.transfer(EquiliBotSpiDevices::IMU, tx.data(), rx.data(), count + 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    for (size_t i = 0; i < count; ++i)
    {
        values[i] = rx[i + 1];
    }
    return ESP_OK;
}

esp_err_t ICM42670Spi::get_acce_sensitivity(float &sensitivity)
{
    sensitivity = 0.0f;
    uint8_t acce_fs = 0;
    esp_err_t ret = read_register(kAccelConfig0, acce_fs);
    if (ret != ESP_OK)
    {
        return ret;
    }

    acce_fs = (acce_fs >> 5) & 0x03;
    switch (acce_fs)
    {
    case ACCE_FS_16G:
        sensitivity = kAcceFs16gSensitivity;
        break;
    case ACCE_FS_8G:
        sensitivity = kAcceFs8gSensitivity;
        break;
    case ACCE_FS_4G:
        sensitivity = kAcceFs4gSensitivity;
        break;
    case ACCE_FS_2G:
        sensitivity = kAcceFs2gSensitivity;
        break;
    default:
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t ICM42670Spi::get_gyro_sensitivity(float &sensitivity)
{
    sensitivity = 0.0f;
    uint8_t gyro_fs = 0;
    esp_err_t ret = read_register(kGyroConfig0, gyro_fs);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gyro_fs = (gyro_fs >> 5) & 0x03;
    switch (gyro_fs)
    {
    case GYRO_FS_2000DPS:
        sensitivity = kGyroFs2000Sensitivity;
        break;
    case GYRO_FS_1000DPS:
        sensitivity = kGyroFs1000Sensitivity;
        break;
    case GYRO_FS_500DPS:
        sensitivity = kGyroFs500Sensitivity;
        break;
    case GYRO_FS_250DPS:
        sensitivity = kGyroFs250Sensitivity;
        break;
    default:
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t ICM42670Spi::get_raw_value(uint8_t reg, icm42670_raw_value_t &value)
{
    value = {};
    uint8_t data[6] = {0};
    const esp_err_t ret = read_registers(reg, data, sizeof(data));
    if (ret != ESP_OK)
    {
        return ret;
    }

    value[0] = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    value[1] = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    value[2] = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);

    return ESP_OK;
}

esp_err_t ICM42670Spi::get_acce_value(icm42670_value_t &value)
{
    value = {};

    float sensitivity = 0.0f;
    esp_err_t ret = get_acce_sensitivity(sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    if (sensitivity == 0.0f)
    {
        return ESP_ERR_INVALID_STATE;
    }

    icm42670_raw_value_t raw_value{};
    ret = get_raw_value(kAccelData, raw_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value[0] = raw_value[0] / sensitivity;
    value[1] = raw_value[1] / sensitivity;
    value[2] = raw_value[2] / sensitivity;
    return ESP_OK;
}

esp_err_t ICM42670Spi::get_gyro_value(icm42670_value_t &value)
{
    value = {};

    float sensitivity = 0.0f;
    esp_err_t ret = get_gyro_sensitivity(sensitivity);
    if (ret != ESP_OK)
    {
        return ret;
    }
    if (sensitivity == 0.0f)
    {
        return ESP_ERR_INVALID_STATE;
    }

    icm42670_raw_value_t raw_value{};
    ret = get_raw_value(kGyroData, raw_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value[0] = raw_value[0] / sensitivity;
    value[1] = raw_value[1] / sensitivity;
    value[2] = raw_value[2] / sensitivity;
    return ESP_OK;
}
