#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#include "spi_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <cstddef>
#include <array>
#include <cstdint>

typedef enum
{
    ICM42670_ACCE_PWR_OFF = 0,      /*!< Accelerometer power off state */
    ICM42670_ACCE_PWR_OFF2 = 1,     /*!< Accelerometer power off state */
    ICM42670_ACCE_PWR_LOWPOWER = 2, /*!< Accelerometer low-power mode */
    ICM42670_ACCE_PWR_LOWNOISE = 3, /*!< Accelerometer low noise state */
} ICM42670AccePwr_t;
typedef enum
{
    ICM42670_GYRO_PWR_OFF = 0,      /*!< Gyroscope power off state */
    ICM42670_GYRO_PWR_STANDBY = 1,  /*!< Gyroscope power standby state */
    ICM42670_GYRO_PWR_LOWNOISE = 3, /*!< Gyroscope power low noise state */
} ICM42670GyroPwr_t;

typedef enum
{
    ICM42670_ACCE_ODR_1600HZ = 5,    /*!< Accelerometer ODR 1.6 kHz */
    ICM42670_ACCE_ODR_800HZ = 6,     /*!< Accelerometer ODR 800 Hz */
    ICM42670_ACCE_ODR_400HZ = 7,     /*!< Accelerometer ODR 400 Hz */
    ICM42670_ACCE_ODR_200HZ = 8,     /*!< Accelerometer ODR 200 Hz */
    ICM42670_ACCE_ODR_100HZ = 9,     /*!< Accelerometer ODR 100 Hz */
    ICM42670_ACCE_ODR_50HZ = 10,     /*!< Accelerometer ODR 50 Hz */
    ICM42670_ACCE_ODR_25HZ = 11,     /*!< Accelerometer ODR 25 Hz */
    ICM42670_ACCE_ODR_12_5HZ = 12,   /*!< Accelerometer ODR 12.5 Hz */
    ICM42670_ACCE_ODR_6_25HZ = 13,   /*!< Accelerometer ODR 6.25 Hz */
    ICM42670_ACCE_ODR_3_125HZ = 14,  /*!< Accelerometer ODR 3.125 Hz */
    ICM42670_ACCE_ODR_1_5625HZ = 15, /*!< Accelerometer ODR 1.5625 Hz */
} ICM42670AcceODR_t;
typedef enum
{
    ICM42670_GYRO_ODR_1600HZ = 5,  /*!<Gyroscope ODR 1.6 kHz */
    ICM42670_GYRO_ODR_800HZ = 6,   /*!<Gyroscope ODR 800 Hz */
    ICM42670_GYRO_ODR_400HZ = 7,   /*!<Gyroscope ODR 400 Hz */
    ICM42670_GYRO_ODR_200HZ = 8,   /*!<Gyroscope ODR 200 Hz */
    ICM42670_GYRO_ODR_100HZ = 9,   /*!<Gyroscope ODR 100 Hz */
    ICM42670_GYRO_ODR_50HZ = 10,   /*!<Gyroscope ODR 50 Hz */
    ICM42670_GYRO_ODR_25HZ = 11,   /*!<Gyroscope ODR 25 Hz */
    ICM42670_GYRO_ODR_12_5HZ = 12, /*!<Gyroscope ODR 12.5 Hz */
} ICM42670GyroODR_t;

typedef enum
{
    ICM42670_ACCE_FS_16G = 0, /*!< Accelerometer full scale range is +/- 16g */
    ICM42670_ACCE_FS_8G = 1,  /*!< Accelerometer full scale range is +/- 8g */
    ICM42670_ACCE_FS_4G = 2,  /*!< Accelerometer full scale range is +/- 4g */
    ICM42670_ACCE_FS_2G = 3,  /*!< Accelerometer full scale range is +/- 2g */
} ICM42670AcceFS_t;
typedef enum
{
    ICM42670_GYRO_FS_2000DPS = 0, /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    ICM42670_GYRO_FS_1000DPS = 1, /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    ICM42670_GYRO_FS_500DPS = 2,  /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    ICM42670_GYRO_FS_250DPS = 3,  /*!< Gyroscope full scale range is +/- 250 degree per sencond */
} ICM42670GyroFS_t;

typedef enum
{
    ICM42670_UI_FILT_BW_BYPASS = 0,
    ICM42670_UI_FILT_BW_180HZ = 0b001,
    ICM42670_UI_FILT_BW_121HZ = 0b010,
    ICM42670_UI_FILT_BW_73HZ = 0b011,
    ICM42670_UI_FILT_BW_53HZ = 0b100,
    ICM42670_UI_FILT_BW_34HZ = 0b101,
    ICM42670_UI_FILT_BW_25HZ = 0b110,
    ICM42670_UI_FILT_BW_16HZ = 0b111,
} ICM42670LowpassBW_t;

struct ICM42670Config
{
    ICM42670AcceODR_t acce_odr;
    ICM42670GyroODR_t gyro_odr;
    ICM42670AcceFS_t acce_fs;
    ICM42670GyroFS_t gyro_fs;
    ICM42670LowpassBW_t acce_bw;
    ICM42670LowpassBW_t gyro_bw;
};
using ICM42670RawVal_t = std::array<int16_t, 3>;
using ICM42670Val_t = std::array<float, 3>;

struct ICM42670Sample
{
    ICM42670Val_t acc;
    ICM42670Val_t gyro;
    uint64_t ts_ms;
};

class ICM42670Spi
{
public:
    ICM42670Spi(SpiBus &spi_bus, gpio_num_t cs_pin, const ICM42670Config &config,
                gpio_num_t interrupt_pin = GPIO_NUM_NC);
    ~ICM42670Spi();
    esp_err_t read_sample(ICM42670Sample &sample);
    bool receive_sample(ICM42670Sample &sample, TickType_t timeout_ticks);
    esp_err_t configure_sensor(const ICM42670Config &config);

    ICM42670Spi(const ICM42670Spi &) = delete;
    ICM42670Spi &operator=(const ICM42670Spi &) = delete;
    ICM42670Spi(ICM42670Spi &&) = delete;
    ICM42670Spi &operator=(ICM42670Spi &&) = delete;

private:
    static void data_ready_task_entry(void *arg);
    static void IRAM_ATTR gpio_isr_handler(void *arg);
    void data_ready_task();

    esp_err_t initialize();
    esp_err_t check_device_present();
    esp_err_t configure_data_ready_interrupt();
    esp_err_t setup_interrupt_gpio();
    esp_err_t start_data_ready_task();

    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t write_registers(uint8_t reg, const uint8_t *values, size_t count);
    esp_err_t read_register(uint8_t reg, uint8_t &value);
    esp_err_t read_registers(uint8_t reg, uint8_t *values, size_t count);

    esp_err_t set_acce_power(ICM42670AccePwr_t state);
    esp_err_t set_gyro_power(ICM42670GyroPwr_t state);

    esp_err_t get_acce_sensitivity(float &sensitivity);
    esp_err_t get_gyro_sensitivity(float &sensitivity);
    esp_err_t get_raw_value(uint8_t reg, ICM42670RawVal_t &value);
    esp_err_t get_acce_value(ICM42670Val_t &value);
    esp_err_t get_gyro_value(ICM42670Val_t &value);

    SpiBus &spi_bus;
    ICM42670Config config;
    gpio_num_t interrupt_pin;
    QueueHandle_t sample_queue = nullptr;
    TaskHandle_t data_ready_task_handle = nullptr;
    bool initialized = false;
};
