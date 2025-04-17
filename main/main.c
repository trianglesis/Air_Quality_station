#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "co2_sensor.h"
#include "BME680.h"

#include "ST7789V3.h"
#include "card_driver.h"
#include "lvgl_driver.h"
#include "led_driver.h"
#include "local_flash.h"
#include "wifi_ap.h"
#include "webserver.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_mac.h"

// LVGL locally installed
#include "lvgl.h"
// For SQ Line Studio
#include "ui/ui.h"


static const char *TAG = "co2station";

#define SPIN_ITER   350000  //actual CPU cycles consumed will depend on compiler optimization
#define CORE0       0
// only define xCoreID CORE1 as 1 if this is a multiple core processor target, else define it as tskNO_AFFINITY
#define CORE1       ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)


/*
Drawing LVGL graphics, run task in between all sensors are collected

Drawing UI between the measurements, CAN? be faster, that measurement itself.
    Set 1 second as default now.

Read the queue message in DESTRICTIVE manner! 
    OR not, then the co2 measure task should clean the queue if it's near full.
    xQueueReceive - destroy the message
    xQueuePeek - read the message, not destroying

*/
static void lvgl_task(void * pvParameters) {
    esp_err_t ret;

    int to_wait_ms = 10;
    int co2_ppm; // data type should be same as queue item type
    struct BMESensor bme680_readings; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(to_wait_ms);

    esp_log_level_set("lcd_panel", ESP_LOG_VERBOSE);
    esp_log_level_set("lcd_panel.st7789", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    // Moved display init in the main
    ret = lvgl_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL Display failed to initialize");
        while (1);
    }
    long curtime = esp_timer_get_time()/1000;
    ui_init(); // NOTE: Always init UI from SquareLine Studio export!
    // Handle LVGL tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));  // idle between cycles
        lv_task_handler();
        if (esp_timer_get_time()/1000 - curtime > 1000) {
            curtime = esp_timer_get_time()/1000;
            // mq_co2 is a pointer now, do not check its len, always peek
            xQueuePeek(mq_co2, (void *)&co2_ppm, xTicksToWait);
            xQueuePeek(mq_bme680, (void *)&bme680_readings, xTicksToWait);
        
            // Init SQ Line Studio elements

            // CO2 Arc SDC41
            lv_arc_set_value(ui_ArcCO2, co2_ppm);
            lv_label_set_text_fmt(ui_LabelCo2Count, "%d", co2_ppm);
            lv_label_set_text(ui_LabelCo2, "CO2");
            lv_label_set_text(ui_LabelCo2Ppm, "ppm");

            // Temerature, humidity etc: BME680
            ESP_LOGI(TAG, "BME680 Received: t: %4.0f, hum: %4.0f, press: %4.0f, res: %4.0f", bme680_readings.temperature, bme680_readings.humidity, bme680_readings.pressure, bme680_readings.resistance);
            // lv_obj_remove_flag(ui_ImageTemperature, LV_OBJ_FLAG_HIDDEN);
            // lv_obj_remove_flag(ui_ImageHumidity, LV_OBJ_FLAG_HIDDEN);
            // lv_obj_remove_flag(ui_ImagePressure, LV_OBJ_FLAG_HIDDEN);

            int32_t temperature;
            int32_t humidity;
            temperature = bme680_readings.temperature;
            humidity = bme680_readings.humidity;
            lv_bar_set_value(ui_BarTemperature, temperature, LV_ANIM_ON);
            lv_bar_set_value(ui_BarHumidity, humidity, LV_ANIM_ON);

            lv_label_set_text_fmt(ui_LabelTemperature, "%.0f", bme680_readings.temperature);
            lv_label_set_text_fmt(ui_LabelHumidity, "%.0f", bme680_readings.humidity);
            lv_label_set_text_fmt(ui_LabelPressure, "%.0f", bme680_readings.pressure);
            lv_label_set_text_fmt(ui_LabelAirQualityIndx, "AQI %.0f", bme680_readings.resistance);

            // Storage info
            // lv_label_set_text_fmt(ui_Label4, "SD: %ld GB", SDCard_Size);
            lv_label_set_text_fmt(ui_LabelSdFree, "SD: %.0fGB/%.0fGB free", sd_free, sd_total);
            // Hardcode total as string 2Mb and save LCD space
            // lv_label_set_text_fmt(ui_Label5, "LFS: %.0fKB/%.0fKB used", littlefs_used, littlefs_total);
            lv_label_set_text_fmt(ui_LabelLfsUsed, "LFS: %.0fKB/2MB used", littlefs_used);
            
            // Network info
            if (wifi_ap_mode == true && found_wifi == false) {
                // Show Wifi AP icon if it's active and users connected count
                lv_label_set_text_fmt(ui_LabelApUsers, "%d", connected_users);
                lv_obj_remove_flag(ui_ImageAPMode, LV_OBJ_FLAG_HIDDEN);
                lv_obj_remove_flag(ui_LabelApUsers, LV_OBJ_FLAG_HIDDEN);
            } else if (found_wifi == true) {
                // Show WiFi local Icon and IP
                lv_label_set_text_fmt(ui_LabelipAdress, "%s", ip_string);
                lv_obj_remove_flag(ui_ImageLocalWiFI, LV_OBJ_FLAG_HIDDEN);
                lv_obj_remove_flag(ui_LabelipAdress, LV_OBJ_FLAG_HIDDEN);
                // Hide AP mode icons
                lv_obj_add_flag(ui_LabelApUsers, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_ImageAPMode, LV_OBJ_FLAG_HIDDEN);
            } else {
                // Lables and icons are hidden by default!
                // Hide all other connection icons
                lv_obj_add_flag(ui_LabelApUsers, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_ImageAPMode, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_LabelipAdress, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_ImageLocalWiFI, LV_OBJ_FLAG_HIDDEN);
                // Show no WiFi Icon
                lv_obj_remove_flag(ui_ImageNoWiFi, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    // Debug
}


void app_main() {
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Create internal objects for sensors and queues before everything else
    create_mq_co2();
    create_mq_bme680();
    
    // Early init
    led_init();
    // SPI (local) flash partition mount and check:
    ESP_ERROR_CHECK(fs_setup());
    // SD Card before display
    ESP_ERROR_CHECK(card_init());
    // Display after SD, before LVGL:
    ESP_ERROR_CHECK(display_init());
    
    // Create a set of tasks to read sensors and update LCD, LED and other elements
    xTaskCreatePinnedToCore(lvgl_task, "LVGL task", 8192, NULL, 9, NULL, tskNO_AFFINITY);

    /*
    Old example setup for LVGL, now moving forward and create multiple tasks with message queue.
    Post all sensor readings into the queue
    Create a one bulky function to read all sensors and put readings into the queue,
    later read from the queue and fill data into LVGL objects, LED colours, webserver API endpoints, whatever.

    TaskHandle_t taskHandle = NULL;
    BaseType_t res = xTaskCreatePinnedToCore(lvgl_task, "LVGL task", 8192, NULL, 4, &taskHandle, 0);
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    */

    // NVS SET by Wifi module externally
    // Start Wifi AP
    ESP_LOGI(TAG, "Start Wifi and try to connect");
    ESP_ERROR_CHECK(wifi_setup());
    // Simple webserver
    ESP_ERROR_CHECK(start_webserver());

    // delay to let tasks finish the last loop
    vTaskDelay(pdMS_TO_TICKS(500));
}
// 