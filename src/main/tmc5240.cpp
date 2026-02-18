#include "tmc5240.hpp"
#include "esp_log.h"
#include "esp_check.h"

constexpr auto kTag = "tmc5240";

TMC5240::TMC5240(SpiBus &spi_bus, gpio_num_t cs_pin, int device_id)
    : spi_bus(spi_bus), dev_id(device_id)
{
    SpiDeviceConfig devconfig = {
        .mode = SpiDeviceConfig::Mode::CPOL1_CPHA1,
        .cs = cs_pin,
        .clk_speed_hz = SPI_MASTER_FREQ_10M,
    };
    spi_bus.add_device(device_id, devconfig);
}

esp_err_t TMC5240::write(uint8_t addr, uint32_t data, uint8_t &spi_status)
{
    uint8_t tx[5] = {(uint8_t)(addr | 0x80),
                     (uint8_t)((data & 0xFF000000) >> 24),
                     (uint8_t)((data & 0x00FF0000) >> 16),
                     (uint8_t)((data & 0x0000FF00) >> 8),
                     (uint8_t)((data & 0x000000FF) >> 0)};
    uint8_t rx[5] = {0};
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag, "Spi Write Failed");
    spi_status = rx[0];
    return ESP_OK;
}
esp_err_t TMC5240::read(uint8_t addr, uint32_t &val, uint8_t &spi_status)
{
    uint8_t tx[5] = {addr, 0, 0, 0, 0};
    uint8_t rx[5];
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag, "Failed Spi Read 1");
    ESP_RETURN_ON_ERROR(spi_bus.transfer(dev_id, tx, rx, 5), kTag, "Failed Spi Read 2");
    spi_status = rx[0];
    val = rx[4] + (rx[3] << 8) + (rx[2] << 16) + (rx[1] << 24);
    return ESP_OK;
}