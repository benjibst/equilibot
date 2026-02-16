#include "spi_bus.hpp"

#include "esp_log.h"

#include <cassert>

SpiBus::SpiBus(spi_host_device_t device, SpiBusConfig busconfig, int num_devices)
    : devices(num_devices, 0), spi_host(device)
{
    LockGuard lg(mutex);

    spi_bus_config_t config = {
        .mosi_io_num = busconfig.mosi,
        .miso_io_num = busconfig.miso,
        .sclk_io_num = busconfig.sclk,
    };

    esp_err_t ret = spi_bus_initialize(device, &config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__FILE__, "Failed initializing SPI bus");
    }
}

esp_err_t SpiBus::add_device(int device_id, SpiDeviceConfig config)
{
    LockGuard lg(mutex);

    assert(device_id >= 0 && static_cast<size_t>(device_id) < devices.size());
    assert(!devices[device_id]);

    spi_device_interface_config_t dev_config = {
        .mode = static_cast<uint8_t>(config.mode),
        .clock_speed_hz = static_cast<int>(config.clk_speed_hz),
        .spics_io_num = config.cs,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(spi_host, &dev_config, &devices[device_id]);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__FILE__, "Failed adding device %d to SPI bus", device_id);
    }
    return ret;
}

esp_err_t SpiBus::transfer(int device_id, void *tx, void *rx, size_t len_bytes)
{
    LockGuard lg(mutex);

    assert(device_id >= 0 && static_cast<size_t>(device_id) < devices.size());
    assert(devices[device_id]);

    spi_transaction_t trans = {
        .length = len_bytes * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(devices[device_id], &trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__FILE__, "Failed transferring data to device %d", device_id);
    }
    return ret;
}
