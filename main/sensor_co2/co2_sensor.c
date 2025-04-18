#include "co2_sensor.h"
#include "led_driver.h"

static const char *TAG_FAKE = "co2-sensor";
static const char *TAG = "scd4x";

QueueHandle_t mq_co2;


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
        struct SCD4XSensor scd4x_readings = {};
        scd4x_readings.co2_ppm = co2_counter;
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG_FAKE, "sent data = %d", co2_counter);
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
New driver and proper readings
https://esp-idf-lib.readthedocs.io/en/latest/groups/scd4x.html
https://github.com/UncleRus/esp-idf-lib/blob/a02cd6bb5190cab379125140780adcb8d88f9650/FAQ.md

TODO: Add calibration, pressure update, altitude, set ambient temp for this sensor from BME680
- scd4x_set_automatic_self_calibration
- scd4x_set_temperature_offset
- scd4x_set_ambient_pressure
*/
void co2_scd4x_reading(void * pvParameters) {
    i2c_dev_t dev = { 0 };
    
    dev.cfg.scl_pullup_en = true;
    dev.cfg.sda_pullup_en = true;
    dev.cfg.mode = I2C_MODE_MASTER;
    dev.cfg.sda_io_num = SDA_PIN_SCDX;         // select SDA GPIO specific to your project
    dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev.cfg.scl_io_num = SCL_PIN_SCDX;         // select SCL GPIO specific to your project
    dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    dev.cfg.master.clk_speed = I2C_FREQ_HZ;  // select frequency specific to your project
    dev.cfg.clk_flags = 0;

    ESP_ERROR_CHECK(scd4x_init_desc(&dev, 0, SDA_PIN_SCDX, SCL_PIN_SCDX));
    ESP_LOGI(TAG, "Initializing sensor...");
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Add delay before measurement
    ESP_LOGI(TAG, "Wake up sensor...");
    ESP_ERROR_CHECK(scd4x_wake_up(&dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&dev));
    ESP_ERROR_CHECK(scd4x_reinit(&dev));
    ESP_LOGI(TAG, "Sensor initialized");

    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    ESP_LOGI(TAG, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    // Calibration and set up
    // Set temperature offset in °C. t_offset – Temperature offset in degrees Celsius (°C) 
    // scd4x_set_temperature_offset(&dev, float t_offset);

    // Set ambient pressure.  uint16_t  Convert value to Pa by: value * 100 
    // scd4x_set_ambient_pressure(&dev, uint16_t pressure);

    // Set sensor altitude in meters above sea level. 
    // scd4x_set_sensor_altitude(&dev, uint16_t altitude);

    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));
    ESP_LOGI(TAG, "Periodic measurements started");

    uint16_t co2_ppm;
    float temperature, humidity;
    while (1)
    {
        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(wait_co2_next_measure));
        struct SCD4XSensor scd4x_readings = {};

        esp_err_t res = scd4x_read_measurement(&dev, &co2_ppm, &temperature, &humidity);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        if (co2_ppm == 0) {
            ESP_LOGW(TAG, "Invalid sample detected, skipping");
            continue;
        }
        scd4x_readings.temperature = temperature;
        scd4x_readings.humidity = humidity;
        scd4x_readings.co2_ppm = co2_ppm;

        xQueueOverwrite(mq_co2, (void *)&scd4x_readings);
        ESP_LOGI(TAG_FAKE, "CO2: %u ppm, Temperature: %.2f °C, Humidity: %.2f %%", co2_ppm, temperature, humidity);

    }

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


void create_mq_co2() {
    ESP_ERROR_CHECK(i2cdev_init());
    // Message Queue
    // static const uint8_t mq_co2_len = 1;
    mq_co2 = xQueueGenericCreate(1, sizeof(int), queueQUEUE_TYPE_SET);
    if (!mq_co2) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    // xTaskCreatePinnedToCore(co2_reading, "co2_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(co2_scd4x_reading, "co2_scd4x_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(led_co2, "led_co2", 4096, NULL, 8, NULL, tskNO_AFFINITY);
}
