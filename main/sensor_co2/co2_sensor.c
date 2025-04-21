#include "co2_sensor.h"
#include "led_driver.h"
#include "scd4x_i2c.h"

static const char *TAG_FAKE = "co2-sensor";
static const char *TAG = "scd40";

QueueHandle_t mq_co2;

i2c_master_bus_handle_t i2c_master_bus_handle;

i2c_master_dev_handle_t scd41_handle;

static uint16_t co2_counter = 0;     // Faking CO2 levels by simple counter

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
        struct SCD4XSensor scd4x_readings = {};
        ESP_LOGI(TAG_FAKE, "sent data = %d", co2_counter);

        scd4x_readings.co2_ppm = co2_counter;
        scd4x_readings.temperature = 0;
        scd4x_readings.humidity = 0;
        // Try to add item to queue, fail immediately if queue is full

        xQueueOverwrite(mq_co2, (void *)&scd4x_readings);
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

*/
void co2_scd4x_reading(void * pvParameters) {

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(wait_co2_next_measure));
    }

}

/*
Get I2C bus 
Add SCD40 sensor at it, and use device handle

*/
esp_err_t sensor_init(void) {
    int status;

    // Get main I2C bus
    master_bus_get(i2c_master_bus_handle);
    // Add device to the bus
    master_bus_device_add(i2c_master_bus_handle, scd41_handle, SCD4X_I2C_ADDR);
    ESP_LOGI(TAG, "Device added! Can start communication!");
    // Init SCD4x - add handler for module
    scd4x_init(scd41_handle);

    // Probably a good idea is to shut the sensor before use it again.
    // Stop any ongoing measurement.
    status = scd4x_stop_periodic_measurement();
    if (status) {
        printf("Unable to stop measurement. Error: %d\n", status);
        return status;
    }
    return ESP_OK;
}

/*
Led HUE based on CO2 levels as task
    xQueueReceive - destroy the message
    xQueuePeek - read the message, not destroying
*/
void led_co2(void * pvParameters) {
    // Read from the queue
    struct SCD4XSensor scd4x_readings; // data type should be same as queue item type
    const TickType_t xTicksToWait = pdMS_TO_TICKS(wait_co2_to_led);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(wait_co2_to_led));  // idle between cycles
        xQueuePeek(mq_co2, (void *)&scd4x_readings, xTicksToWait);
        // Update LED colour
        led_co2_severity(scd4x_readings.co2_ppm);
    }
}


/*
Always create queue first, at the very beginning.
*/
void create_mq_co2() {
    // Message Queue
    mq_co2 = xQueueGenericCreate(1, sizeof(struct SCD4XSensor), queueQUEUE_TYPE_SET);
    if (!mq_co2) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    // Add device to I2C bus and stop it to prepare for measurements
    ESP_ERROR_CHECK(sensor_init());

}

void task_co2() {

    xTaskCreatePinnedToCore(co2_reading, "co2_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(led_co2, "led_co2", 4096, NULL, 8, NULL, tskNO_AFFINITY);

    // xTaskCreatePinnedToCore(co2_scd4x_reading, "co2_scd4x_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
}