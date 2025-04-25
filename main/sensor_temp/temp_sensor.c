#include <string.h>
#include "temp_sensor.h"
#include "bme680.h"
#include "i2c_driver.h"

static const char *TAG_FAKE = "fake-temp";
static const char *APP_TAG = "bme680";

QueueHandle_t mq_bme680;

i2c_master_dev_handle_t bme680_handle; // Update as soon as all other 
bme680_handle_t dev_hdl;


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

void bme680_reading(void * pvParameters) {
    // 
    TickType_t last_wake_time  = xTaskGetTickCount ();

    while (1) {
        esp_err_t result;
        struct BMESensor bme680_readings = {};

        for(uint8_t i = 0; i < dev_hdl->dev_config.heater_profile_size; i++) {
            bme680_data_t data;
            result = bme680_get_data_by_heater_profile(dev_hdl, i, &data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
            }
            ESP_LOGI(APP_TAG, "%u    %.2f    %.2f          %.2f         %.2f          %.2f               %u        %s        %u        %s            %u (%s)",
                i,
                data.air_temperature,
                data.dewpoint_temperature,
                data.relative_humidity,
                data.barometric_pressure/100,
                data.gas_resistance/1000,
                data.gas_range,
                data.gas_valid ? "yes" : "no",
                data.gas_index,
                data.heater_stable ? "yes" : "no",
                data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
            
            // Try to add item to queue, fail immediately if queue is full
            ESP_LOGI(TAG_FAKE, "Sending t: %4.0f, hum: %4.0f, press: %4.0f, res: %4.0f", data.air_temperature, data.relative_humidity, data.barometric_pressure/100, data.gas_resistance/1000);

            bme680_readings.temperature = data.air_temperature;
            bme680_readings.humidity = data.relative_humidity;
            bme680_readings.pressure = data.barometric_pressure/100;
            bme680_readings.resistance = data.gas_resistance/1000;
            bme680_readings.air_q_index = data.iaq_score;
            xQueueOverwrite(mq_bme680, (void *)&bme680_readings);

            vTaskDelay(pdMS_TO_TICKS(250));
        }
        vTaskDelayUntil( &last_wake_time, 60 );
    }
}

static inline void print_registers(bme680_handle_t handle) {
    /* configuration registers */
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas0_register_t        ctrl_gas0_reg;
    bme680_control_gas1_register_t        ctrl_gas1_reg;

    /* attempt to read control humidity register */
    bme680_get_control_humidity_register(handle, &ctrl_humi_reg);

    /* attempt to read control measurement register */
    bme680_get_control_measurement_register(handle, &ctrl_meas_reg);

    /* attempt to read configuration register */
    bme680_get_configuration_register(handle, &config_reg);

    /* attempt to read control gas 0 register */
    bme680_get_control_gas0_register(handle, &ctrl_gas0_reg);

    /* attempt to read control gas 1 register */
    bme680_get_control_gas1_register(handle, &ctrl_gas1_reg);

    ESP_LOGI(APP_TAG, "Variant Id          (0x%02x): %s", handle->variant_id,uint8_to_binary(handle->variant_id));
    ESP_LOGI(APP_TAG, "Configuration       (0x%02x): %s", config_reg.reg,    uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Control Measurement (0x%02x): %s", ctrl_meas_reg.reg, uint8_to_binary(ctrl_meas_reg.reg));
    ESP_LOGI(APP_TAG, "Control Humidity    (0x%02x): %s", ctrl_humi_reg.reg, uint8_to_binary(ctrl_humi_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 0       (0x%02x): %s", ctrl_gas0_reg.reg, uint8_to_binary(ctrl_gas0_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 1       (0x%02x): %s", ctrl_gas1_reg.reg, uint8_to_binary(ctrl_gas1_reg.reg));
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
    // initialize i2c device configuration
    bme680_config_t dev_cfg         = I2C_BME680_FORCED_CONFIG_DEFAULT;
    // init device
    bme680_init(bus_handle, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bme680 handle init failed");
        assert(dev_hdl);
    }
    print_registers(dev_hdl);
    return ESP_OK;
}

void create_mq_bme680() {
    // Message Queue
    mq_bme680 = xQueueGenericCreate(1, sizeof(struct BMESensor), queueQUEUE_TYPE_SET);
    if (!mq_bme680) {
        ESP_LOGE(APP_TAG, "queue creation failed");
    }
}

void task_bme680() {

    // Add device to I2C bus ...
    ESP_ERROR_CHECK(bme680_sensor_init());

    xTaskCreatePinnedToCore(bme680_reading, "bme680_reading", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(bme680_reading_fake, "bme680_reading_fake", 4096, NULL, 4, NULL, tskNO_AFFINITY);
}
