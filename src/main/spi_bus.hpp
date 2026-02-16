#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <vector>
#include "lock_guard.hpp"

struct SpiBusConfig
{
    gpio_num_t miso, mosi, sclk;
};

struct SpiDeviceConfig
{
    enum class Mode
    {
        CPOL0_CPHA0,
        CPOL0_CPHA1,
        CPOL1_CPHA0,
        CPOL1_CPHA1
    } mode;
    gpio_num_t cs;
    uint32_t clk_speed_hz;
};
class SpiBus
{
public:
    SpiBus(spi_host_device_t device, SpiBusConfig busconfig, int num_devices);
    esp_err_t add_device(int device_id, SpiDeviceConfig config);
    esp_err_t transfer(int device_id, void *tx, void *rx, size_t len_bytes);

private:
    StaticMutex mutex;
    std::vector<spi_device_handle_t> devices;
    spi_host_device_t spi_host;
};
