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

#define wait_next_measure 1000 // How often to measure CO2 level in ms

extern QueueHandle_t mq_bme680;

#define PORT 0
#define BME680_I2C_ADDR_0 0x76
#define BME680_I2C_ADDR_1 0x77

/*
#define COMMON_SDA_PIN 0
#define COMMON_SCL_PIN 1
#define SCL_PIN_BME680 1  // Same as SCD40
#define SDA_PIN_BME680 0  // Same as SCD40
#define I2C_FREQ_HZ 100000 // Same as SCD40
*/


struct BMESensor {
    float temperature;
    float humidity;
    float pressure;
    float resistance;
    uint16_t air_q_index;
};

void bme680_reading_fake(void * pvParameters);
void bme680_reading(void * pvParameters);
void create_mq_bme680(void);
void task_bme680(void);

esp_err_t bme680_sensor_init(void);