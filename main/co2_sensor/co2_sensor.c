
#include "co2_sensor.h"
#include "led_driver.h"

static const char *TAG = "co2-sensor";

static int co2_counter = 0;     // Faking CO2 levels by simple counter

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
void co2_reading(void * pvParameters) {
    // 
    while (1) {
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG, "sent data = %d", co2_counter);
        xQueueOverwrite(mq_co2, (void *)&co2_counter);
        // Make up and down
        if (co2_counter == 2500) {
            co2_counter = 0;
        } else {
            co2_counter++;
        }
        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(wait_co2_next_measure));
    }
    // Always should end, when taking measurements if not in loop: https://stackoverflow.com/a/63635154
    // vTaskDelete(NULL);
}

/*
Led HUE based on CO2 levels as task
    xQueueReceive - destroy the message
    xQueuePeek - read the message, not destroying
*/
void led_co2(void * pvParameters) {
    // Read from the queue
    int co2_counter; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(wait_co2_to_led);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(wait_co2_to_led));  // idle between cycles
        xQueuePeek(mq_co2, (void *)&co2_counter, xTicksToWait);
        // Update LED colour
        led_co2_severity(co2_counter);
    }
}


void create_mq_co2(void) {
    // Message Queue
    mq_co2 = xQueueGenericCreate(mq_co2_len, sizeof(int), queueQUEUE_TYPE_SET);
    if (mq_co2 == NULL) {
        ESP_LOGE(TAG, "queue creation failed");
    }
}

/*
if (mq_co2 > 1) {
    // Destructive read
    if (xQueueReceive(mq_co2, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
        // ESP_LOGI(TAG, "received data = %d", co2_counter);
    } else {
        // Skip drawing if there is no mesages left
        // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
    }
} else {
    // Queue recieve, non destructive! Always with xQueueOverwrite
    if (xQueuePeek(mq_co2, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
        // ESP_LOGI(TAG, "received data = %d", co2_counter);
    } else {
        // Skip drawing if there is no mesages left
        // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
    }
}
*/