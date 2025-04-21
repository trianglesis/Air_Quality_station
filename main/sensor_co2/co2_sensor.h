// #pragma once

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "i2c_driver.h"

#include "esp_log.h"

#define wait_co2_next_measure 5000  // Sensor can only provide it once for 5 sec!
#define wait_co2_to_led 5000 * 3    // No need to update led too often

extern QueueHandle_t mq_co2;
extern i2c_master_dev_handle_t scd41_handle;

#define SCD4X_I2C_ADDR 0x62

/*
Pin and freq settings are common for all sensors here 
    and defined at main\i2c_driver\i2c_driver.h

    #define SCL_PIN_SCDX 0
#define SDA_PIN_SCDX 1
#define I2C_FREQ_HZ 100000 // 100kHz
*/


struct SCD4XSensor {
    float temperature;
    float humidity;
    uint16_t co2_ppm;
};

void co2_reading(void * pvParameters);  // Fake
void co2_scd4x_reading(void * pvParameters);
void led_co2(void * pvParameters);
void create_mq_co2(void);
void task_co2(void);

esp_err_t get_measures(void);

esp_err_t sensor_init(void);

