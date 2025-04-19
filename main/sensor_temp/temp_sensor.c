
#include "temp_sensor.h"
#include <string.h>

static const char *TAG = "BME680";
static const char *TAG_FAKE = "fake-temp";

QueueHandle_t mq_bme680;

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
        if (BME_pressure == 850) {
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
New driver and proper readings
https://esp-idf-lib.readthedocs.io/en/latest/groups/bme680.html

https://github.com/UncleRus/esp-idf-lib/blob/a02cd6bb5190cab379125140780adcb8d88f9650/FAQ.md
*/
void bme680_reading(void * pvParameters) {
    bme680_t sensor = { 0 };
    
    sensor.i2c_dev.cfg.scl_pullup_en = true;
    sensor.i2c_dev.cfg.sda_pullup_en = true;

    memset(&sensor, 0, sizeof(bme680_t));
    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR_0, PORT, SDA_PIN_BME680, SCL_PIN_BME680));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_NONE, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    bme680_values_float_t values;
    while (1)
    {
        struct BMESensor bme680_readings = {};
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK) {
            // passive waiting until measurement results are available
            vTaskDelay(duration);
            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK) {
                ESP_LOGI(TAG, "Got t: %4.0f, hum: %4.0f %%, hPa: %4.2f, res: %4.2f Ohm", values.temperature, values.humidity, values.pressure, values.gas_resistance);
                bme680_readings.temperature = values.temperature;
                bme680_readings.humidity = values.humidity;
                bme680_readings.pressure = values.pressure;
                bme680_readings.resistance = values.gas_resistance;
            }
        }
        ESP_LOGI(TAG_FAKE, "Sending sensor values");
        xQueueOverwrite(mq_bme680, (void *)&bme680_readings);
        // Wait next
        vTaskDelay(pdMS_TO_TICKS(wait_next_measure));
    }
}


void create_mq_bme680() {
    // Message Queue
    // static const uint8_t mq_co2_len = 1;
    mq_bme680 = xQueueGenericCreate(1, sizeof(struct BMESensor), queueQUEUE_TYPE_SET);
    if (!mq_bme680) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    
    xTaskCreatePinnedToCore(bme680_reading_fake, "bme680_reading_fake", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    // ESP_ERROR_CHECK(i2cdev_init());
    // xTaskCreatePinnedToCore(bme680_reading, "bme680_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);

}
