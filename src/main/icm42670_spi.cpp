#include "icm42670_spi.hpp"
#include "common.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include <array>

namespace
{

    constexpr char kTag[] = "ICM42670Spi";

    constexpr uint32_t kSpiClockHz = 1000 * 1000;
    constexpr uint8_t kReadMask = 0x80;

    constexpr uint8_t kIntConfig = 0x06;
    constexpr uint8_t kWhoAmI = 0x75;
    constexpr uint8_t kPwrMgmt0 = 0x1F;
    constexpr uint8_t kGyroConfig0 = 0x20;
    constexpr uint8_t kAccelConfig0 = 0x21;
    constexpr uint8_t kGyroConfig1 = 0x23;
    constexpr uint8_t kAccelConfig1 = 0x24;
    constexpr uint8_t kIntSource0 = 0x2B;
    constexpr uint8_t kIntStatusDrdy = 0x39;
    constexpr uint8_t kAccelData = 0x0B;
    constexpr uint8_t kGyroData = 0x11;

    constexpr uint8_t kIcm42670Id = 0x67;

    constexpr uint8_t kInt1ModePulsed = 0;
    constexpr uint8_t kInt1DrivePushPull = 1;
    constexpr uint8_t kInt1PolarityActiveHigh = 1;
    constexpr uint8_t kDrdyInt1EnableMask = 1u << 3;

    constexpr UBaseType_t kDataReadyTaskPriority = configMAX_PRIORITIES - 2;
    constexpr uint32_t kSampleQueueLength = 2;

    constexpr float kGyroFs2000Sensitivity = 16.4f;
    constexpr float kGyroFs1000Sensitivity = 32.8f;
    constexpr float kGyroFs500Sensitivity = 65.5f;
    constexpr float kGyroFs250Sensitivity = 131.0f;

    constexpr float kAcceFs16gSensitivity = 2048.0f;
    constexpr float kAcceFs8gSensitivity = 4096.0f;
    constexpr float kAcceFs4gSensitivity = 8192.0f;
    constexpr float kAcceFs2gSensitivity = 16384.0f;

} // namespace

ICM42670Spi::ICM42670Spi(SpiBus &spi_bus, gpio_num_t cs_pin, const ICM42670Config &config,
                         gpio_num_t interrupt_pin)
    : spi_bus(spi_bus), config(config), interrupt_pin(interrupt_pin)
{
    initialized = false;
    sample_queue = xQueueCreate(kSampleQueueLength, sizeof(ICM42670Sample));
    assert(sample_queue);

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

    const esp_err_t task_ret = start_data_ready_task();
    if (task_ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed starting data-ready task: %s", esp_err_to_name(task_ret));
        return;
    }
    initialized = true;
}

esp_err_t ICM42670Spi::read_sample(ICM42670Sample &sample)
{
    uint64_t start_time = esp_timer_get_time();
    esp_err_t ret = get_acce_value(sample.acc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed reading accelerometer value: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ret = get_gyro_value(sample.gyro);
    if (ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed reading gyroscope value: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    sample.ts_ms = (esp_timer_get_time() + start_time) / 2000;
    return ESP_OK;
}

bool ICM42670Spi::receive_sample(ICM42670Sample &sample, TickType_t timeout_ticks)
{
    return xQueueReceive(sample_queue, &sample, timeout_ticks) == pdTRUE;
}

void ICM42670Spi::data_ready_task_entry(void *arg)
{
    auto *self = static_cast<ICM42670Spi *>(arg);
    self->data_ready_task();
}

void IRAM_ATTR ICM42670Spi::gpio_isr_handler(void *arg)
{
    auto *self = static_cast<ICM42670Spi *>(arg);

    BaseType_t high_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(self->data_ready_task_handle, &high_priority_task_woken);
    if (high_priority_task_woken == pdTRUE)
        portYIELD_FROM_ISR();
}

void ICM42670Spi::data_ready_task()
{
    ICM42670Sample sample;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t drdy_status = 0;
        const esp_err_t status_ret = read_register(kIntStatusDrdy, drdy_status);
        if (status_ret != ESP_OK)
        {
            ESP_LOGW(kTag, "Failed to read INT_STATUS_DRDY: %s", esp_err_to_name(status_ret));
            continue;
        }
        if ((drdy_status & 0x01u) == 0)
        {
            continue;
        }
        if (read_sample(sample) == ESP_OK)
            xQueueSend(sample_queue, &sample, 0);
    }
}

ICM42670Spi::~ICM42670Spi()
{
    if (interrupt_pin != GPIO_NUM_NC)
    {
        gpio_isr_handler_remove(interrupt_pin);
    }

    if (data_ready_task_handle != nullptr)
    {
        vTaskDelete(data_ready_task_handle);
        data_ready_task_handle = nullptr;
    }

    if (sample_queue != nullptr)
    {
        vQueueDelete(sample_queue);
        sample_queue = nullptr;
    }
}

esp_err_t ICM42670Spi::initialize()
{
    esp_err_t ret = check_device_present();
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (configure_sensor(config) != ESP_OK)
        return ESP_FAIL;

    uint8_t data = ICM42670_GYRO_PWR_LOWNOISE << 2 | ICM42670_ACCE_PWR_LOWNOISE;
    if (write_registers(kPwrMgmt0, &data, 1) != ESP_OK)
        return ESP_FAIL;

    // Datasheet: after transitioning Accel/Gyro from OFF to any active mode, avoid
    // issuing register writes for at least 200 us.
    esp_rom_delay_us(200);

    assert(interrupt_pin != GPIO_NUM_NC);
    if (configure_data_ready_interrupt() != ESP_OK)
        return ESP_FAIL;

    if (setup_interrupt_gpio() != ESP_OK)
        return ESP_FAIL;

    return ESP_OK;
}

esp_err_t ICM42670Spi::configure_data_ready_interrupt()
{
    uint8_t int_config = 0;
    esp_err_t ret = read_register(kIntConfig, int_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int_config = static_cast<uint8_t>((int_config & ~0x07u) |
                                      ((kInt1ModePulsed & 0x01u) << 2) |
                                      ((kInt1DrivePushPull & 0x01u) << 1) |
                                      (kInt1PolarityActiveHigh & 0x01u));
    ret = write_register(kIntConfig, int_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t int_source0 = 0;
    ret = read_register(kIntSource0, int_source0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int_source0 = static_cast<uint8_t>(int_source0 | kDrdyInt1EnableMask);
    ret = write_register(kIntSource0, int_source0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t drdy_status = 0;
    return read_register(kIntStatusDrdy, drdy_status);
}

esp_err_t ICM42670Spi::setup_interrupt_gpio()
{
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << interrupt_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    esp_err_t ret = gpio_config(&io_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        return ret;
    }

    ret = gpio_isr_handler_add(interrupt_pin, gpio_isr_handler, this);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

esp_err_t ICM42670Spi::start_data_ready_task()
{
    if (interrupt_pin == GPIO_NUM_NC)
    {
        return ESP_OK;
    }

    const BaseType_t created = xTaskCreate(data_ready_task_entry,
                                           "icm42670_drdy",
                                           4096,
                                           this,
                                           kDataReadyTaskPriority,
                                           &data_ready_task_handle);
    if (created != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

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

esp_err_t ICM42670Spi::configure_sensor(const ICM42670Config &cfg)
{
    if (!memcmp(&cfg, &config, sizeof(ICM42670Config)))
        return ESP_OK;
    uint8_t config_data[2] = {
        (uint8_t)(((cfg.gyro_fs & 0x03) << 5) | (cfg.gyro_odr & 0x0F)),
        (uint8_t)(((cfg.acce_fs & 0x03) << 5) | (cfg.acce_odr & 0x0F)),
    };
    if (write_registers(kGyroConfig0, config_data, sizeof(config_data)) != ESP_OK)
        return ESP_FAIL;
    uint8_t data = cfg.gyro_bw & 0x7;
    if (write_registers(kGyroConfig1, &data, 1) != ESP_OK)
        return ESP_FAIL;
    data = cfg.acce_bw & 0x7;
    if (write_registers(kAccelConfig1, &data, 1) != ESP_OK)
        return ESP_FAIL;
    config = cfg;
    return ESP_OK;
}

esp_err_t ICM42670Spi::set_acce_power(ICM42670AccePwr_t state)
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

esp_err_t ICM42670Spi::set_gyro_power(ICM42670GyroPwr_t state)
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
    case ICM42670_ACCE_FS_16G:
        sensitivity = kAcceFs16gSensitivity;
        break;
    case ICM42670_ACCE_FS_8G:
        sensitivity = kAcceFs8gSensitivity;
        break;
    case ICM42670_ACCE_FS_4G:
        sensitivity = kAcceFs4gSensitivity;
        break;
    case ICM42670_ACCE_FS_2G:
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
    case ICM42670_GYRO_FS_2000DPS:
        sensitivity = kGyroFs2000Sensitivity;
        break;
    case ICM42670_GYRO_FS_1000DPS:
        sensitivity = kGyroFs1000Sensitivity;
        break;
    case ICM42670_GYRO_FS_500DPS:
        sensitivity = kGyroFs500Sensitivity;
        break;
    case ICM42670_GYRO_FS_250DPS:
        sensitivity = kGyroFs250Sensitivity;
        break;
    default:
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t ICM42670Spi::get_raw_value(uint8_t reg, ICM42670RawVal_t &value)
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

esp_err_t ICM42670Spi::get_acce_value(ICM42670Val_t &value)
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

    ICM42670RawVal_t raw_value{};
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

esp_err_t ICM42670Spi::get_gyro_value(ICM42670Val_t &value)
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

    ICM42670RawVal_t raw_value{};
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
