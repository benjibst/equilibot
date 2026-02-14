#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm42670.h"

namespace
{

    constexpr const char *kTag = "icm42670_test";

    // Update these pins to match your hardware wiring.
    constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
    constexpr gpio_num_t kI2cSdaPin = GPIO_NUM_14;
    constexpr gpio_num_t kI2cSclPin = GPIO_NUM_21;
    constexpr gpio_num_t kIcmAd0Pin = GPIO_NUM_13;

    constexpr TickType_t kReadPeriod = pdMS_TO_TICKS(200);

    i2c_master_bus_handle_t s_i2c_bus = nullptr;
    icm42670_handle_t s_icm42670 = nullptr;
    bool s_spi_bus_initialized = false;
    void cleanup()
    {
        if (s_icm42670 != nullptr)
        {
            icm42670_delete(s_icm42670);
            s_icm42670 = nullptr;
        }
        if (s_i2c_bus != nullptr)
        {
            i2c_del_master_bus(s_i2c_bus);
            s_i2c_bus = nullptr;
        }
        if (s_spi_bus_initialized)
        {
            spi_bus_free(SPI2_HOST);
            s_spi_bus_initialized = false;
        }
    }

    esp_err_t init_i2c_bus()
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = kI2cPort;
        bus_cfg.sda_io_num = kI2cSdaPin;
        bus_cfg.scl_io_num = kI2cSclPin;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;

        return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    }

    esp_err_t force_icm_address_0x68()
    {
        gpio_config_t ad0_cfg = {};
        ad0_cfg.pin_bit_mask = (1ULL << kIcmAd0Pin);
        ad0_cfg.mode = GPIO_MODE_OUTPUT;
        ad0_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        ad0_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        ad0_cfg.intr_type = GPIO_INTR_DISABLE;

        esp_err_t ret = gpio_config(&ad0_cfg);
        if (ret != ESP_OK)
        {
            return ret;
        }

        return gpio_set_level(kIcmAd0Pin, 0);
    }
    esp_err_t init_i2c()
    {
        esp_err_t ret;
        if ((ret = force_icm_address_0x68()) != ESP_OK)
        {
            ESP_LOGE(kTag, "Failed to force AD0 low on GPIO%d: %s", kIcmAd0Pin, esp_err_to_name(ret));
            cleanup();
            return ret;
        }
        ESP_LOGI(kTag, "GPIO%d forced low, using ICM42670 address 0x68", kIcmAd0Pin);

        if ((ret = init_i2c_bus()) != ESP_OK)
        {
            ESP_LOGE(kTag, "Failed to create I2C bus: %s", esp_err_to_name(ret));
            cleanup();
            return ret;
        }
        ESP_LOGI(kTag, "I2C bus created (SDA=%d, SCL=%d)", kI2cSdaPin, kI2cSclPin);
        return ESP_OK;
    }
    esp_err_t init_spi()
    {
        spi_bus_config_t bus_cfg = {};
        bus_cfg.mosi_io_num = GPIO_NUM_14;
        bus_cfg.miso_io_num = GPIO_NUM_13;
        bus_cfg.sclk_io_num = GPIO_NUM_21;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.data4_io_num = -1;
        bus_cfg.data5_io_num = -1;
        bus_cfg.data6_io_num = -1;
        bus_cfg.data7_io_num = -1;
        bus_cfg.max_transfer_sz = 16;

        esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK)
        {
            ESP_LOGE(kTag, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            cleanup();
            return ret;
        }
        s_spi_bus_initialized = true;
        ESP_LOGI(kTag, "SPI bus initialized");
        return ESP_OK;
    }
    esp_err_t init_icm42670(bool spi)
    {
        if (spi)
        {
            if (init_spi() != ESP_OK)
            {
                return ESP_FAIL;
            }
            if (icm42670_create_spi(SPI2_HOST, GPIO_NUM_41, &s_icm42670) != ESP_OK)
            {
                return ESP_FAIL;
            }
        }
        else
        {
            if (init_i2c() != ESP_OK)
            {
                return ESP_FAIL;
            }
            if (icm42670_create_i2c(s_i2c_bus, ICM42670_I2C_ADDRESS, &s_icm42670) != ESP_OK)
            {
                return ESP_FAIL;
            }
        }

        icm42670_cfg_t cfg = {};
        cfg.acce_fs = ACCE_FS_2G;
        cfg.acce_odr = ACCE_ODR_400HZ;
        cfg.gyro_fs = GYRO_FS_2000DPS;
        cfg.gyro_odr = GYRO_ODR_400HZ;

        esp_err_t ret = icm42670_config(s_icm42670, &cfg);
        if (ret != ESP_OK)
        {
            return ret;
        }

        ret = icm42670_acce_set_pwr(s_icm42670, ACCE_PWR_LOWNOISE);
        if (ret != ESP_OK)
        {
            return ret;
        }

        return icm42670_gyro_set_pwr(s_icm42670, GYRO_PWR_LOWNOISE);
    }
} // namespace

uint64_t us_avg = 0;
uint64_t cnt = 0;
extern "C" void app_main(void)
{
    bool spi = true; // Set to true to test SPI interface instead of I2C
    esp_err_t ret = init_icm42670(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(kTag, "Failed to init ICM42670: %s", esp_err_to_name(ret));
        cleanup();
        return;
    }
    ESP_LOGI(kTag, "ICM42670 initialized");

    while (true)
    {
        icm42670_value_t acc = {};
        icm42670_value_t gyro = {};
        float temp_c = 0.0f;

        uint64_t start_time = esp_timer_get_time();
        const esp_err_t acc_ret = icm42670_get_acce_value(s_icm42670, &acc);
        const esp_err_t gyro_ret = icm42670_get_gyro_value(s_icm42670, &gyro);
        const esp_err_t temp_ret = icm42670_get_temp_value(s_icm42670, &temp_c);
        uint64_t end_time = esp_timer_get_time();
        us_avg += (end_time - start_time);
        cnt++; 
        ESP_LOGI(kTag, "Read time: %lld us (avg: %lld us over %lld reads) using %s", end_time - start_time, us_avg / cnt, cnt, spi ? "SPI" : "I2C");

        if (acc_ret == ESP_OK && gyro_ret == ESP_OK && temp_ret == ESP_OK)
        {
            ESP_LOGI(kTag,
                     "acc[g] x=%.3f y=%.3f z=%.3f | gyro[dps] x=%.3f y=%.3f z=%.3f | temp=%.2fC",
                     acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, temp_c);
        }
        else
        {
            ESP_LOGW(kTag, "Read failed: acc=%s gyro=%s temp=%s",
                     esp_err_to_name(acc_ret),
                     esp_err_to_name(gyro_ret),
                     esp_err_to_name(temp_ret));
        }

        vTaskDelay(kReadPeriod);
    }
}
