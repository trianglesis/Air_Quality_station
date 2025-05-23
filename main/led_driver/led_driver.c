#include "led_driver.h"

static const char *TAG = "LED-WS2812B";

static led_strip_handle_t led_strip;

int random_int_range(int min, int max) {    
    // Return random int from the range.
    // Use to blink led with random values.
    return min + (esp_random() % (max - min + 1));
}

void led_control_pixel_random(void) {
    // Simple colour
    int RED = random_int_range(1, 255);
    int GREEN = random_int_range(1, 255);
    int BLUE = random_int_range(1, 255);
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, LED_STRIP_LED_COUNT, RED, GREEN, BLUE));
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

void led_control_hsv_random(void) {
    // Color with saturation and brightness
    int hue = random_int_range(1, 360);  // MAX 360
    int saturation = random_int_range(1, 255); // MAX 255
    int brightLvl = random_int_range(1, 255);  // MAX 255
    ESP_ERROR_CHECK(led_strip_set_pixel_hsv(led_strip, LED_STRIP_LED_COUNT, hue, saturation, brightLvl));
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

void led_control_pixel(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue) {
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, index, red, green, blue));
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

void led_control_hsv(led_strip_handle_t strip, uint32_t index, uint16_t hue, uint8_t saturation, uint8_t value) {
    ESP_ERROR_CHECK(led_strip_set_pixel_hsv(strip, index, hue, saturation, value));
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

/*
See the color ordering of the LED: RGB is working fine.
Led count is 1
Model WS2812B-0807
Pin 8
*/
void led_init(void) {
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_COUNT,
        .led_model = LED_STRIP_MODEL,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
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
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
    // No need to return
    // led_strip_handle_t led_strip = led_strip;
    // return led_strip;
}


/*
Tested doc\test\co2_interpolation_test.py
Calculate LED HUE based on the level of CO2
Using atmospheric level substraction to proper indication.
Max level 2000 after which any CO2 level is always RED

Generate LED colour based on CO2 severity levels:
    Hue - colour
    Saturation - MAX
    Value - 200 of 255, to make color vibrant, not darker

Example
    (800 - 440) / (1560) = 0.2307
    (1 - 0.2307) * 96 = 73.8528 is HUE

*/
void led_co2_severity(int co2_ppm) {
    float co2_ENV = 427.0;  // Outdoors level
    float co2_MAX = 2000.0; // Max level, all RED
    float co2 = MIN(co2_MAX, co2_ppm);
    float t = (co2 - co2_ENV) / (co2_MAX - co2_ENV);
    float hue_calc = (1 - t) * 96;
    // https://cplusplus.com/reference/cstdio/printf/
    // ESP_LOGI(TAG, "CO2 lvl = %d co2 min = %4.2f HUE = %4.2f t = %4.2f", co2_ppm, co2, hue_calc, t);
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
        led_control_hsv(led_strip, i, hue_calc, 255, 200);
    }
}

