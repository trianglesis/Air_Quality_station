#include "temp_sensor.h"
#include "bme680.h"
#include <string.h>

static const char *TAG_FAKE = "fake-temp";
static const char *TAG = "bme680";

QueueHandle_t mq_bme680;

i2c_master_dev_handle_t bme680_handle; // Update as soon as all other 


static float BME_temperature = 15.1; // Faking
static float BME_humidity = 15.1;    // Faking
static float BME_pressure = 500.1;    // Faking
static float BME_resistance = 1000.1;  // Faking

/*
Generating fake temp, humidity, pressure etc

*/
void bme680_reading_fake(void * pvParameters) {
    // 
    while (1) {
        struct BMESensor bme680_readings = {};
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG_FAKE, "Sending t: %4.0f, hum: %4.0f, press: %4.0f, res: %4.0f", BME_temperature, BME_humidity, BME_pressure, BME_resistance);
        bme680_readings.temperature = BME_temperature;
        bme680_readings.humidity = BME_humidity;
        bme680_readings.pressure = BME_pressure;
        bme680_readings.resistance = BME_resistance;
        
        xQueueOverwrite(mq_bme680, (void *)&bme680_readings);

        // Make up and down
        if (BME_temperature >= 45) {
            BME_temperature = 15;
        } else {
            BME_temperature++;
        }
        // Make up and down
        if (BME_humidity >= 100) {
            BME_humidity = 15;
        } else {
            BME_humidity++;
        }
        // Make up and down
        if (BME_pressure >= 850) {
            BME_pressure = 700;
        } else {
            BME_pressure++;
        }
        // Make up and down
        if (BME_resistance >= 1500) {
            BME_resistance = 900;
        } else {
            BME_resistance++;
        }

        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(wait_next_measure));
    }
    // Always should end, when taking measurements if not in loop: https://stackoverflow.com/a/63635154
    // vTaskDelete(NULL);
}

/*
Get I2C bus 
Add sensor at it, and use device handle

TODO: Add SD card read\write option to save states:
- last operation mode
- last power mode
- calibration values
- last measurement or even log

*/
esp_err_t bme680_sensor_init(void) {
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    // Do not send Pins and freq, they are already added by CO2 sensor, address is static in h
    ESP_ERROR_CHECK(bme680_init_desc(&sensor));
    ESP_ERROR_CHECK(master_bus_probe_address(BME680_I2C_ADDR_1, 50)); // Wait 50 ms

    // Now init the sensor itself
    bme680_init_sensor(&sensor);

    return ESP_OK;
}

void create_mq_bme680() {
    // Message Queue
    mq_bme680 = xQueueGenericCreate(1, sizeof(struct BMESensor), queueQUEUE_TYPE_SET);
    if (!mq_bme680) {
        ESP_LOGE(TAG, "queue creation failed");
    }
}

void task_bme680() {

    // Add device to I2C bus ...
    ESP_ERROR_CHECK(bme680_sensor_init());

    // xTaskCreatePinnedToCore(bme680_reading, "bme680_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(bme680_reading_fake, "bme680_reading_fake", 4096, NULL, 4, NULL, tskNO_AFFINITY);
}
