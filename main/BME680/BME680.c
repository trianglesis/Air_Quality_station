
#include "BME680.h"

static const char *TAG = "BME680";

QueueHandle_t mq_bme680;

static float BME_temperature = 15.1; // Faking
static float BME_humidity = 15.1;    // Faking
static float BME_pressure = 500.1;    // Faking
static float BME_resistance = 1000.1;  // Faking


/*
Generating fake temp, humidity, pressure etc

*/
void bme680_reading(void * pvParameters) {
    // 

    while (1) {
        struct BMESensor bme680_readings = {};
        // Try to add item to queue, fail immediately if queue is full
        ESP_LOGI(TAG, "Sending t: %4.0f, hum: %4.0f, press: %4.0f, res: %4.0f", BME_temperature, BME_humidity, BME_pressure, BME_resistance);
        bme680_readings.temperature = BME_temperature;
        bme680_readings.humidity = BME_humidity;
        bme680_readings.pressure = BME_pressure;
        bme680_readings.resistance = BME_resistance;
        
        xQueueOverwrite(mq_bme680, (void *)&bme680_readings);

        // Make up and down
        if (BME_temperature == 35) {
            BME_temperature = 15;
        } else {
            BME_temperature++;
        }
        // Make up and down
        if (BME_humidity == 100) {
            BME_humidity = 15;
        } else {
            BME_humidity++;
        }
        // Make up and down
        if (BME_pressure == 800) {
            BME_pressure = 700;
        } else {
            BME_pressure++;
        }
        // Make up and down
        if (BME_resistance == 1500) {
            BME_resistance = 900;
        } else {
            BME_resistance++;
        }

        // Now send CO2 level further, send an item for every 1000ms
        vTaskDelay(pdMS_TO_TICKS(wait_co2_next_measure));
    }
    // Always should end, when taking measurements if not in loop: https://stackoverflow.com/a/63635154
    // vTaskDelete(NULL);
}

void create_mq_bme680() {
    // Message Queue
    // static const uint8_t mq_co2_len = 1;
    mq_bme680 = xQueueGenericCreate(1, sizeof(struct BMESensor), queueQUEUE_TYPE_SET);
    if (!mq_bme680) {
        ESP_LOGE(TAG, "queue creation failed");
    }
    xTaskCreatePinnedToCore(bme680_reading, "bme680_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    // return mq_co2;
}
