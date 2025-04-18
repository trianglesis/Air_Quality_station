// #pragma once

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "bme680.h"

#include "esp_log.h"


#define wait_next_measure 1000 // How often to measure CO2 level in ms

extern QueueHandle_t mq_bme680;

#define PORT 0
#define BME680_I2C_ADDR_0 0x76
#define BME680_I2C_ADDR_1 0x77
// TODO: Change to real!
#define SCL_PIN_BME680 20
#define SDA_PIN_BME680 23

struct BMESensor {
    float temperature;
    float humidity;
    float pressure;
    float resistance;
};


void bme680_reading_fake(void * pvParameters);
void bme680_reading(void * pvParameters);
void create_mq_bme680(void);