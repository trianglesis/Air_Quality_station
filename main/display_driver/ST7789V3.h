#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
// Also include LVGL setup here
#include "lvgl_driver.h"
/* LCD size */
#define DISP_HOR_RES        172
#define DISP_VER_RES        320
/* LCD settings */
#define DISP_DRAW_BUFF_HEIGHT 50
/* LCD pins */
// From example https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47
#define DISP_SPI_NUM         SPI2_HOST
// 
#define LCD_PIXEL_CLOCK_HZ     (12 * 1000 * 1000)
// Setup as DataSheet shows
#define DISP_GPIO_SCLK       GPIO_NUM_7     // GPIO_NUM_7
#define DISP_GPIO_MOSI       GPIO_NUM_6     // GPIO_NUM_6
#define DISP_GPIO_RST        GPIO_NUM_21    // GPIO_NUM_21
#define DISP_GPIO_DC         GPIO_NUM_15    // GPIO_NUM_15
#define DISP_GPIO_CS         GPIO_NUM_14    // GPIO_NUM_14
#define DISP_GPIO_BL         GPIO_NUM_22    // GPIO_NUM_22
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
// 
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO       EXAMPLE_PIN_NUM_BK_LIGHT
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_DUTY         (4000)
#define LEDC_ResolutionRatio   LEDC_TIMER_13_BIT
#define LEDC_MAX_Duty          ((1 << LEDC_ResolutionRatio) - 1)


// Glob
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_io_handle_t io_handle;

// Initialize the LCD backlight, which has been called in the LCD_Init function
void BK_Init(void);                                                         
// Call this function to adjust the brightness of the backlight. The value of the parameter Light ranges from 0 to 100
void BK_Light(uint8_t Light);
// Call this function to initialize the screen (must be called in the main function)
esp_err_t display_init(void);
