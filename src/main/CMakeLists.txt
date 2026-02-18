idf_component_register(
    SRCS "main.cpp"
         "led_strip.cpp"
         "spi_bus.cpp"
         "icm42670_spi.cpp"
         "web_server.cpp"
         "tmc5240.cpp"
    EMBED_TXTFILES "index.html"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
             esp_driver_spi
             esp_event
             esp_http_server
             esp_netif
             esp_timer
             esp_wifi
             nvs_flash
)
