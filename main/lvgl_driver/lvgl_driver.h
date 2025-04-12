#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
// Use display
#include "ST7789V3.h"

// Rotate 90deg and compensate buffer change
// TODO: Move it to the main file for better visibility
#define Offset_X 0 // 0 IF NOT ROTATED 270deg
#define Offset_Y 34  // 34 IF ROTATED 270deg

// Save display when init and use it all over the project
extern lv_disp_t *display; 

// 
#define LVGL_TICK_PERIOD_MS    2
// Display buffer use
#define BUFFER_SIZE            (DISP_HOR_RES * DISP_VER_RES * 2 / 10)

bool notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
// Set resolution enforce
void set_resolution(lv_display_t* disp);
/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
// void rotate_display_and_touch(lv_disp_drv_t *drv);

void lvgl_tick_increment(void *arg);
// Call this function to initialize the screen (must be called in the main function)
esp_err_t lvgl_init(void);