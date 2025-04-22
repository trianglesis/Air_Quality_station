#include "co2_sensor.h"
#include "led_driver.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

static const char *TAG_FAKE = "co2-fake";
static const char *TAG = "scd40";

QueueHandle_t mq_co2;  // Always init early, even empty!

i2c_master_dev_handle_t scd41_handle; // Update as soon as all other 


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
    static int co2_counter = 0;
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
    bool dataReady;
    uint16_t co2Raw;         // ppm
    int32_t t_mili_deg;  // millicelsius
    int32_t humid_mili_percent;     // millipercent
    while (1) {
        struct SCD4XSensor scd4x_readings = {};
        // Check if measurements are ready, for 5 sec cycle
        dataReady = scd4x_get_data_ready_status(scd41_handle);
        if (!dataReady) {
            ESP_LOGD(TAG, "CO2 data ready status is not ready! Wait for next check.");
            vTaskDelay(pdMS_TO_TICKS(1000));  // Waiting 1 sec before checking again!
            continue;
        } else {
            // Now read
            scd4x_read_measurement(scd41_handle, &co2Raw, &t_mili_deg, &humid_mili_percent);
            ESP_LOGD(TAG, "RAW Measurements ready co2: %d, t: %ld C Humidity: %ld (raw value)", 
                co2Raw, 
                t_mili_deg, 
                humid_mili_percent);
            // Post conversion from mili
            const int co2Ppm = co2Raw;
            const float t_celsius = t_mili_deg / 1000.0f;
            const float humid_percent = humid_mili_percent / 1000.0f;
            ESP_LOGD(TAG, "CO2: %d ppm, Temperature: %.1f C Humidity: %.1f%%\n", 
                co2Ppm, 
                t_celsius, 
                humid_percent);
            // Add to queue
            scd4x_readings.co2_ppm = co2Ppm;
            scd4x_readings.temperature = t_celsius;
            scd4x_readings.humidity = humid_percent;
            xQueueOverwrite(mq_co2, (void *)&scd4x_readings);
            // Waiting before get next measurements, usually not LT 5 sec.
            vTaskDelay(pdMS_TO_TICKS(wait_co2_next_measure));
        }

    }
}

/*
Get I2C bus 
Add SCD40 sensor at it, and use device handle

TODO: Add SD card read\write option to save states:
- last operation mode
- last power mode
- calibration values
- last measurement or even log

*/
esp_err_t sensor_init(void) {
    esp_err_t ret;
    i2c_device_config_t scd41_device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD4X_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    // Add device to the bus
    scd41_handle = master_bus_device_add(scd41_device);
    if (scd41_handle == NULL) {
        ESP_LOGI(TAG, "Device handle is NULL! Exit!");
        return ESP_FAIL;
    } else {
        ESP_LOGD(TAG, "Device added! Probe address!");
    }
    ESP_ERROR_CHECK(master_bus_probe_address(SCD4X_I2C_ADDR, 50)); // Wait 50 ms
    
    // Probably a good idea is to shut the sensor before use it again.
    ret = scd4x_stop_periodic_measurement(scd41_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot send command to device to stop measurements mode!");
        return ESP_FAIL;
    } else {
        ESP_LOGD(TAG, "Stopped measirements now! Can start again after cooldown.");
    }
    
    // Get serial N
    uint16_t serial_number[3] = {0};
    ret = scd4x_get_serial_number(scd41_handle, serial_number, sizeof(serial_number));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot get serial number!");
        return ESP_FAIL;
    } else {
        ESP_LOGD(TAG, "Sensor serial number is: 0x%x 0x%x 0x%x", 
            (int)serial_number[0], 
            (int)serial_number[1], 
            (int)serial_number[2]);
    }
    // TODO: Save states to SD Card.
    // TODO: Read previous states from SD Card
    // TODO: If no SD Card - write to SPI flash partition

    // Switch SCD40 to measurement mode
    // It will not respond to most of other commands in this mode, check the datasheet!
    ret = scd4x_start_periodic_measurement(scd41_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot start periodic measurement mode!");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Starting the periodic mesurement mode, you can check the next data in 5 seconds since now.");
        vTaskDelay(pdMS_TO_TICKS(5000));
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
}

void task_co2() {
    // Add device to I2C bus and stop it to prepare for measurements
    ESP_ERROR_CHECK(sensor_init());

    // Real
    xTaskCreatePinnedToCore(co2_scd4x_reading, "co2_scd4x_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    // Fake
    // xTaskCreatePinnedToCore(co2_reading, "co2_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    // Change LED color
    xTaskCreatePinnedToCore(led_co2, "led_co2", 4096, NULL, 8, NULL, tskNO_AFFINITY);

}