#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#include "spi_bus.hpp"
#include <cstddef>
#include <array>
#include <cstdint>

typedef enum
{
    ACCE_FS_16G = 0, /*!< Accelerometer full scale range is +/- 16g */
    ACCE_FS_8G = 1,  /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_4G = 2,  /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_2G = 3,  /*!< Accelerometer full scale range is +/- 2g */
} icm42670_acce_fs_t;

typedef enum
{
    ACCE_PWR_OFF = 0,      /*!< Accelerometer power off state */
    ACCE_PWR_ON = 1,       /*!< Accelerometer power on state */
    ACCE_PWR_LOWPOWER = 2, /*!< Accelerometer low-power mode */
    ACCE_PWR_LOWNOISE = 3, /*!< Accelerometer low noise state */
} icm42670_acce_pwr_t;

typedef enum
{
    ACCE_ODR_1600HZ = 5,    /*!< Accelerometer ODR 1.6 kHz */
    ACCE_ODR_800HZ = 6,     /*!< Accelerometer ODR 800 Hz */
    ACCE_ODR_400HZ = 7,     /*!< Accelerometer ODR 400 Hz */
    ACCE_ODR_200HZ = 8,     /*!< Accelerometer ODR 200 Hz */
    ACCE_ODR_100HZ = 9,     /*!< Accelerometer ODR 100 Hz */
    ACCE_ODR_50HZ = 10,     /*!< Accelerometer ODR 50 Hz */
    ACCE_ODR_25HZ = 11,     /*!< Accelerometer ODR 25 Hz */
    ACCE_ODR_12_5HZ = 12,   /*!< Accelerometer ODR 12.5 Hz */
    ACCE_ODR_6_25HZ = 13,   /*!< Accelerometer ODR 6.25 Hz */
    ACCE_ODR_3_125HZ = 14,  /*!< Accelerometer ODR 3.125 Hz */
    ACCE_ODR_1_5625HZ = 15, /*!< Accelerometer ODR 1.5625 Hz */
} icm42670_acce_odr_t;

typedef enum
{
    GYRO_FS_2000DPS = 0, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    GYRO_FS_1000DPS = 1, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_500DPS = 2,  /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_250DPS = 3,  /*!< Gyroscope full scale range is +/- 250 degree per sencond */
} icm42670_gyro_fs_t;

typedef enum
{
    GYRO_PWR_OFF = 0,      /*!< Gyroscope power off state */
    GYRO_PWR_STANDBY = 1,  /*!< Gyroscope power standby state */
    GYRO_PWR_LOWNOISE = 3, /*!< Gyroscope power low noise state */
} icm42670_gyro_pwr_t;

typedef enum
{
    GYRO_ODR_1600HZ = 5,  /*!< Gyroscope ODR 1.6 kHz */
    GYRO_ODR_800HZ = 6,   /*!< Gyroscope ODR 800 Hz */
    GYRO_ODR_400HZ = 7,   /*!< Gyroscope ODR 400 Hz */
    GYRO_ODR_200HZ = 8,   /*!< Gyroscope ODR 200 Hz */
    GYRO_ODR_100HZ = 9,   /*!< Gyroscope ODR 100 Hz */
    GYRO_ODR_50HZ = 10,   /*!< Gyroscope ODR 50 Hz */
    GYRO_ODR_25HZ = 11,   /*!< Gyroscope ODR 25 Hz */
    GYRO_ODR_12_5HZ = 12, /*!< Gyroscope ODR 12.5 Hz */
} icm42670_gyro_odr_t;

typedef enum
{
    GYRO_UI_FILT_BW_BYPASS = 0, /*!< LPF bypassed */
    GYRO_UI_FILT_BW_180HZ = 1,  /*!< LPF bandwidth 180 Hz */
    GYRO_UI_FILT_BW_121HZ = 2,  /*!< LPF bandwidth 121 Hz */
    GYRO_UI_FILT_BW_73HZ = 3,   /*!< LPF bandwidth 73 Hz */
    GYRO_UI_FILT_BW_53HZ = 4,   /*!< LPF bandwidth 53 Hz */
    GYRO_UI_FILT_BW_34HZ = 5,   /*!< LPF bandwidth 34 Hz */
    GYRO_UI_FILT_BW_25HZ = 6,   /*!< LPF bandwidth 25 Hz */
    GYRO_UI_FILT_BW_16HZ = 7,   /*!< LPF bandwidth 16 Hz */
} icm42670_gyro_ui_filt_bw_t;

typedef enum
{
    ACCE_UI_FILT_BW_BYPASS = 0, /*!< LPF bypassed */
    ACCE_UI_FILT_BW_180HZ = 1,  /*!< LPF bandwidth 180 Hz */
    ACCE_UI_FILT_BW_121HZ = 2,  /*!< LPF bandwidth 121 Hz */
    ACCE_UI_FILT_BW_73HZ = 3,   /*!< LPF bandwidth 73 Hz */
    ACCE_UI_FILT_BW_53HZ = 4,   /*!< LPF bandwidth 53 Hz */
    ACCE_UI_FILT_BW_34HZ = 5,   /*!< LPF bandwidth 34 Hz */
    ACCE_UI_FILT_BW_25HZ = 6,   /*!< LPF bandwidth 25 Hz */
    ACCE_UI_FILT_BW_16HZ = 7,   /*!< LPF bandwidth 16 Hz */
} icm42670_acce_ui_filt_bw_t;

typedef struct
{
    icm42670_acce_fs_t acce_fs;   /*!< Accelerometer full scale range */
    icm42670_acce_odr_t acce_odr; /*!< Accelerometer ODR selection */
    icm42670_gyro_fs_t gyro_fs;   /*!< Gyroscope full scale range */
    icm42670_gyro_odr_t gyro_odr; /*!< Gyroscope ODR selection */
} icm42670_cfg_t;

struct ICM42670Config
{
    icm42670_cfg_t config;
    icm42670_acce_pwr_t acce_pwr;
    icm42670_gyro_pwr_t gyro_pwr;
    icm42670_acce_ui_filt_bw_t acce_ui_filt_bw = ACCE_UI_FILT_BW_BYPASS;
    icm42670_gyro_ui_filt_bw_t gyro_ui_filt_bw = GYRO_UI_FILT_BW_BYPASS;
};
using icm42670_raw_value_t = std::array<int16_t, 3>;
using icm42670_value_t = std::array<float, 3>;

struct ICM42670Sample
{
    icm42670_value_t acc;
    icm42670_value_t gyro;
};

class ICM42670Spi
{
public:
    ICM42670Spi(SpiBus &spi_bus, gpio_num_t cs_pin, const ICM42670Config &config);
    ~ICM42670Spi();
    ICM42670Sample read_sample();

    ICM42670Spi(const ICM42670Spi &) = delete;
    ICM42670Spi &operator=(const ICM42670Spi &) = delete;
    ICM42670Spi(ICM42670Spi &&) = delete;
    ICM42670Spi &operator=(ICM42670Spi &&) = delete;

private:
    esp_err_t initialize();
    esp_err_t check_device_present();

    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t write_registers(uint8_t reg, const uint8_t *values, size_t count);
    esp_err_t read_register(uint8_t reg, uint8_t &value);
    esp_err_t read_registers(uint8_t reg, uint8_t *values, size_t count);

    esp_err_t set_acce_power(icm42670_acce_pwr_t state);
    esp_err_t set_gyro_power(icm42670_gyro_pwr_t state);
    esp_err_t configure_sensor(const icm42670_cfg_t &cfg);
    esp_err_t configure_low_pass_filters(icm42670_acce_ui_filt_bw_t acce_bw,
                                         icm42670_gyro_ui_filt_bw_t gyro_bw);

    esp_err_t get_acce_sensitivity(float &sensitivity);
    esp_err_t get_gyro_sensitivity(float &sensitivity);
    esp_err_t get_raw_value(uint8_t reg, icm42670_raw_value_t &value);
    esp_err_t get_acce_value(icm42670_value_t &value);
    esp_err_t get_gyro_value(icm42670_value_t &value);

    SpiBus &spi_bus;
    ICM42670Config config;
    bool initialized = false;
};
