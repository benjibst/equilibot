/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"

#define LED_STRIP_LED_COUNT 31
#define LED_STRIP_MEMORY_BLOCK_WORDS 1024 // this determines the DMA block size

constexpr int kWaveStartLed = 1;
constexpr int kWaveEndLed = 24;
constexpr float kSpatialFrequency = 0.45f;
constexpr float kTemporalStep = 0.22f;

constexpr gpio_num_t led_gpio = GPIO_NUM_38;

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "example";

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = led_gpio, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = 1,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

extern "C" void app_main(void)
{
    led_strip_handle_t led_strip = configure_led();
    float phase = 0.0f;

    ESP_LOGI(TAG, "Start sine wave animation on LED strip");
    while (1) {
        for (int i = 0; i < 19; i++) {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;

            if (i >= kWaveStartLed && i <= kWaveEndLed) {
                const float x = (float)(i - kWaveStartLed);
                const float wave = 0.5f * (sinf((x * kSpatialFrequency) + phase) + 1.0f); // 0..1
                b = (uint8_t)(wave * 10.0f);
            }

            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, r, g, b));
        }
        for (size_t i = 19; i < 24; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 10, 0, 0));
        }
        for (size_t i = 24; i < LED_STRIP_LED_COUNT; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 10,0));
        }
        

        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        phase += kTemporalStep;
        vTaskDelay(pdMS_TO_TICKS(100)); // ~30 FPS
    }
}
