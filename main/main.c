#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "ST7789V3.h"
#include "lvgl_driver.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
// LVGL locally installed
#include "lvgl.h"
// For SQ Line Studio
#include "ui/ui.h"

static const char *TAG = "co2station";


// 
static void lvgl_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_log_level_set("lcd_panel", ESP_LOG_VERBOSE);
    esp_log_level_set("lcd_panel.st7789", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_err_t ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST7789 failed to initilize");
        while (1);
    }
    ret = lvgl_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL Display failed to initialize");
        while (1);
    }

    long curtime = esp_timer_get_time()/1000;
    int counter = 0;
    // NOTE: Always init UI from SquareLine Studio export!
    ui_init();

    // Handle LVGL tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();

        if (esp_timer_get_time()/1000 - curtime > 1000) {
            curtime = esp_timer_get_time()/1000;

            // Init SQ Line Studio elements
            lv_arc_set_value(ui_Arc1, counter);
            lv_label_set_text_fmt(ui_Label1, "%d", counter);
            lv_label_set_text(ui_Label2, "CO2 ppm");

            // Make up and down
            if (counter == 2500) {
                counter = 0;
            } else {
                counter++;
            }
        }
    }
}

void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    TaskHandle_t taskHandle = NULL;
    BaseType_t res = xTaskCreatePinnedToCore(lvgl_task, "LVGL task", 8192, NULL, 4, &taskHandle, 0); // stack, params, prio, handle, core
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 