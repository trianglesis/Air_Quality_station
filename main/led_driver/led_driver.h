#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "led_strip.h"

// Do not used
#define LED_STRIP_USE_DMA 0

// LED OPTS
#define TIME_TICK_MS 500

#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically

#define LED_STRIP_MODEL LED_MODEL_WS2812 // WS2812B-0807
#define LED_STRIP_GPIO_PIN 8
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

int random_int_range(int, int);
void led_control_pixel_random(void);
void led_control_hsv_random(void);
void led_control_pixel(led_strip_handle_t strip,
    uint32_t index,
    uint32_t red,
    uint32_t green,
    uint32_t blue);
void led_control_hsv(led_strip_handle_t strip,
    uint32_t index,
    uint16_t hue,
    uint8_t saturation,
    uint8_t value);
void led_co2_severity(int co2_ppm);
void led_init(void);