idf_component_register(
    SRCS "main.cpp" "led_strip.cpp"
    INCLUDE_DIRS "."
    REQUIRES icm42670 esp_driver_gpio esp_driver_i2c
)
