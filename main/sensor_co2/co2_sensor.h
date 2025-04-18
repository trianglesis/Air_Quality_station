// #pragma once

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "scd4x.h"

#include "esp_log.h"

#define wait_co2_next_measure 2000  // How often to measure CO2 level in ms
#define wait_co2_to_led 5000        // How often change LED colour for CO2 measurements in ms

extern QueueHandle_t mq_co2;

#define SCD4X_I2C_ADDR 0x62
#define SCL_PIN_SCDX 0
#define SDA_PIN_SCDX 1
#define I2C_FREQ_HZ 100000 // 100kHz


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

esp_err_t sensor_init(void);

