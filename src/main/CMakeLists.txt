idf_component_register(
    SRCS "main.cpp"
         "led_strip.cpp"
         "spi_bus.cpp"
         "icm42670_spi.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi
)
