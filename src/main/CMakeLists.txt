idf_component_register(
    SRCS "main.cpp"
         "battery_monitor.cpp"
         "cascaded_pid_controller.cpp"
         "led_strip.cpp"
         "spi_bus.cpp"
         "icm42670_spi.cpp"
         "web_server.cpp"
         "tmc5240.cpp"
    EMBED_TXTFILES "index.html"
                   "controller.html"
                   "motor.html"
    INCLUDE_DIRS "."
                 "../external/nlohmann_json/include"
    REQUIRES esp_driver_gpio
             esp_adc
             esp_driver_spi
             esp_event
             esp_http_server
             esp_netif
             esp_timer
             esp_wifi
             nvs_flash
)
