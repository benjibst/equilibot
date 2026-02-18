#pragma once
#include "spi_bus.hpp"

class TMC5240
{
public:
    TMC5240(SpiBus &spi_bus, gpio_num_t cs_pin, int device_id);
    esp_err_t write(uint8_t addr, uint32_t data, uint8_t &spi_status);
    esp_err_t read(uint8_t addr, uint32_t &val, uint8_t &spi_status);
    int device_id() { return this->dev_id; };

private:
    SpiBus &spi_bus;
    int dev_id;
    uint8_t last_read_addr;
};