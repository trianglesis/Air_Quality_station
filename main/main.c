#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "ST7789V3.h"
#include "card_driver.h"
#include "lvgl_driver.h"
#include "led_driver.h"
#include "local_flash.h"
#include "wifi_ap.h"
#include "webserver.h"
// #include "file_server.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_mac.h"

// LVGL locally installed
#include "lvgl.h"
// For SQ Line Studio
#include "ui/ui.h"


static const char *TAG = "co2station";

#define wait_co2_next_measure 1000; // How often to measure CO2 level in ms
#define wait_co2_to_led 1000; // How often change LED colour for CO2 measurements in ms

#define SPIN_ITER   350000  //actual CPU cycles consumed will depend on compiler optimization
#define CORE0       0
// only define xCoreID CORE1 as 1 if this is a multiple core processor target, else define it as tskNO_AFFINITY
#define CORE1       ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)

static QueueHandle_t msg_queue;
// Standard queue with len > 1, can consume any of multiple messages
// static const uint8_t msg_queue_len = 5;
// Or overwittable queue
static const uint8_t msg_queue_len = 1;
static volatile bool timed_out;

static int fake_co2_counter = 0;     // Faking CO2 levels by simple counter

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
    int co2_counter; // data type should be same as queue item type
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
            if (msg_queue_len > 1) {
                // Destructive read
                if (xQueueReceive(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
                    // ESP_LOGI(TAG, "received data = %d", co2_counter);
                } else {
                    // Skip drawing if there is no mesages left
                    // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
                }
            } else {
                // Queue recieve, non destructive! Always with xQueueOverwrite
                if (xQueuePeek(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
                    // ESP_LOGI(TAG, "received data = %d", co2_counter);
                } else {
                    // Skip drawing if there is no mesages left
                    // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
                }
            }
            // Init SQ Line Studio elements
            lv_arc_set_value(ui_Arc1, co2_counter);
            lv_label_set_text_fmt(ui_Label1, "%d", co2_counter);
            lv_label_set_text(ui_Label2, "CO2");
            lv_label_set_text(ui_Label3, "ppm");
            // lv_label_set_text_fmt(ui_Label4, "SD: %ld GB", SDCard_Size);
            lv_label_set_text_fmt(ui_Label4, "SD: %.0fGB/%.0fGB free", sd_free, sd_total);
            // Hardcode total as string 2Mb and save LCD space
            lv_label_set_text_fmt(ui_Label5, "LFS: %.0fKB/%.0fKB used", littlefs_used, littlefs_total);
            if (wifi_ap_mode == true && found_wifi == false) {
                // Show Wifi AP icon if it's active and users connected count
                lv_label_set_text_fmt(ui_Label6, "%d", connected_users);
                lv_obj_remove_flag(ui_Image1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_remove_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);
            } else if (found_wifi == true) {
                // Show WiFi local Icon and IP
                lv_label_set_text_fmt(ui_Label7, "%s", ip_string);
                lv_obj_remove_flag(ui_Image2, LV_OBJ_FLAG_HIDDEN);
                lv_obj_remove_flag(ui_Label7, LV_OBJ_FLAG_HIDDEN);
                // Hide AP mode icons
                lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);
            } else {
                // Lables and icons are hidden by default!
                // Hide all other connection icons
                lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(ui_Label7, LV_OBJ_FLAG_HIDDEN);
                // Show no WiFi Icon
                lv_obj_remove_flag(ui_Image3, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

/*
Led HUE based on CO2 levels as task
    xQueueReceive - destroy the message
    xQueuePeek - read the message, not destroying
*/
static void led_co2(void * pvParameters) {
    int to_wait_ms = wait_co2_to_led;
    int co2_counter; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(to_wait_ms);
    // Read from the queue
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));  // idle between cycles
        if (msg_queue_len > 1) {
            // Destructive read
            if (xQueueReceive(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
                // ESP_LOGI(TAG, "received data = %d", co2_counter);
            } else {
                // Skip drawing if there is no mesages left
                // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
            }
        } else {
            // Queue recieve, non destructive! Always with xQueueOverwrite
            if (xQueuePeek(msg_queue, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
                // ESP_LOGI(TAG, "received data = %d", co2_counter);
            } else {
                // Skip drawing if there is no mesages left
                // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
            }
        }
        // Update LED colour
        led_co2_severity(co2_counter);
    }
}

/*
Generating fake CO2 data
Reuse this function later, adding SDC41 readings

This function will and should always return a lot more values per time, than we usually consume, 
    thus we always consume most latest value or value before it as fallback option.

Queue fill will hang the process you must recieve from the queue!
    Since this task can also empty the queue, we can now consume with non-destrictive method!
    Example: https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/system/freertos/basic_freertos_smp_usage/main/queue_example.c#L33

Send to queue should not be faster, than consuming from it!
    1 second should be alsways enough for any type of measurements!

What if queue is full:
    If there are too many items in the queue - delete outdated items
        uxQueueMessagesWaiting - int
    If queue space become too low - delete older messages
        uxQueueSpacesAvailable - int
    TODO: Consider for ONE message
        xQueueOverwrite for queue of ONE item:
            https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#c.xQueueOverwrite

Now work with queue, add the value at the beginning of the queue (first), 
    so each next task can obtain the most recent value, 
    or the one older (second), if first value accidentally consumed!

*/
static void co2_reading(void * pvParameters) {
    int to_wait_ms = wait_co2_next_measure;
    while (1) {
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG, "sent data = %d", fake_co2_counter);
        if (msg_queue_len > 1) {
            // Always check the space and queue len, clean if half-full. Queue read is non-destructive always.
            int queue_messages = uxQueueMessagesWaiting(msg_queue);
            int queue_space = uxQueueSpacesAvailable(msg_queue);
            if (queue_messages > 1 || queue_space < 3) {
                ESP_LOGI(TAG, "Queue is filled with messages: %d, space left: %d - cleaning the queue!", 
                    queue_messages, 
                    queue_space);
                xQueueReset(msg_queue);
            }
            // When queue is len > 1
            if (xQueueGenericSend(msg_queue, (void *)&fake_co2_counter, 0, queueSEND_TO_FRONT) != pdTRUE) {
                ESP_LOGE(TAG, "Queue full and it should be emtied!\n");
            }
        } else {
            // No need to clean if xQueueOverwrite
            // When queue is len = 1, return is negligible
            xQueueOverwrite(msg_queue, (void *)&fake_co2_counter);
        }
        // Make up and down
        if (fake_co2_counter == 2500) {
            fake_co2_counter = 0;
        } else {
            fake_co2_counter++;
        }
        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(to_wait_ms));
    }
    // Always should end, when taking measurements if not in loop: https://stackoverflow.com/a/63635154
    // vTaskDelete(NULL);
}

void app_main() {
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(10));
    // Early init
    led_init();
    // SPI (local) flash partition mount and check:
    ESP_ERROR_CHECK(fs_setup());
    // SD Card before display
    ESP_ERROR_CHECK(card_init());
    // Display after SD, before LVGL:
    ESP_ERROR_CHECK(display_init());

    // Message Queue
    msg_queue = xQueueGenericCreate(msg_queue_len, sizeof(int), queueQUEUE_TYPE_SET);
    if (msg_queue == NULL) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    
    // Create a set of tasks to read sensors and update LCD, LED and other elements
    xTaskCreatePinnedToCore(co2_reading, "co2_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(led_co2, "led_co2", 4096, NULL, 8, NULL, tskNO_AFFINITY);
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