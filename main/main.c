#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "ST7789V3.h"
#include "lvgl_driver.h"
#include "led_driver.h"

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

static QueueHandle_t msg_queue;
static const uint8_t msg_queue_len = 40;
static volatile bool timed_out;

static int fake_co2_counter = 0;     // Faking CO2 levels by simple counter

/*
Drawing LVGL graphics, run task in between all sensors are collected

Drawing UI between the measurements, CAN? be faster, that measurement itself.
    Set 1 second as default now.
*/
static void lvgl_task(void * pvParameters) {
    int to_wait_ms = 250;
    int co2_counter; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(to_wait_ms);

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
    ui_init(); // NOTE: Always init UI from SquareLine Studio export!
    // Handle LVGL tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));  // idle between cycles
        lv_task_handler();
        if (esp_timer_get_time()/1000 - curtime > 1000) {
            curtime = esp_timer_get_time()/1000;
            // Queue recieve DESTRICTIVE, the value will no longer be in the queue!
            if (xQueueReceive(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
                // ESP_LOGI(TAG, "received data = %d", co2_counter);
            } else {
                ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
            }
            // Init SQ Line Studio elements
            lv_arc_set_value(ui_Arc1, co2_counter);
            lv_label_set_text_fmt(ui_Label1, "%d", co2_counter);
            lv_label_set_text(ui_Label2, "CO2 ppm");
        }
    }
}

// Led HUE based on CO2 levels as task
static void led_co2(void * pvParameters) {
    int to_wait_ms = 3000;
    int co2_counter; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(to_wait_ms);
    while (1) {
        // Queue recieve, non destructive!
        if (xQueuePeek(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
            // ESP_LOGI(TAG, "received data = %d", co2_counter);
        } else {
            ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
        }
        led_co2_severity(co2_counter);
    }
}

/*
Generating fake CO2 data
Reuse this function later, adding SDC41 readings

Queue fill will hang the process you must recieve from the queue!
Example: https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/system/freertos/basic_freertos_smp_usage/main/queue_example.c#L33

Send to queue should not be faster, than consuming from it!
    1 second should be alsways enought for any type of measurements!
*/
static void co2_reading(void * pvParameters) {
    int to_wait_ms = 1000;
    while (1) {
        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG, "sent data = %d", fake_co2_counter);
        if (xQueueGenericSend(msg_queue, (void *)&fake_co2_counter, portMAX_DELAY, queueSEND_TO_BACK) != pdTRUE) {
            ESP_LOGE(TAG, "Queue full\n");
        }
        // Make up and down
        if (fake_co2_counter == 2500) {
            fake_co2_counter = 0;
        } else {
            fake_co2_counter++;
        }
    }
    // Always should end, when taking measurements if not in loop: https://stackoverflow.com/a/63635154
    // vTaskDelete(NULL);
}


/*
Use xQueueReceive to destroy the message after reading!
*/
static void clean_queue() {
    int to_wait_ms = 1000;
    int co2_counter; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(to_wait_ms);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));
        // Queue recieve
        if (xQueueReceive(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
            ESP_LOGI(TAG, "Removing from queue received data = %d", co2_counter);
        } else {
            ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
        }
}
}


void app_main() {
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(10));
    // Early init
    led_init();
    // Message Queue
    msg_queue = xQueueGenericCreate(msg_queue_len, sizeof(int), queueQUEUE_TYPE_SET);
    if (msg_queue == NULL) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    // Create a set of tasks to read sensors and update LCD, LED and other elements
    xTaskCreatePinnedToCore(co2_reading, "co2_reading", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(led_co2, "led_co2", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(lvgl_task, "LVGL task", 8192, NULL, 9, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(clean_queue, "clean_queue", 512, NULL, 10, NULL, tskNO_AFFINITY);

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
    // delay to let tasks finish the last loop
    vTaskDelay(pdMS_TO_TICKS(500));
}
// 