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

struct SCD4XSensor {
    float temperature;
    float humidity;
    uint16_t co2_ppm;
};

/*
Fake reading for testing only. 
Adding to the queue.
*/
void co2_reading(void * pvParameters);

/*
Real task measurements from CO2 sensor.
Adding to the queue
*/
void co2_scd4x_reading(void * pvParameters);

/*
Update LED colour depending on CO2 PPM level
In place, reading from the queue/
*/
void led_co2(void * pvParameters);

/*
Create the queue for CO2 measurements.
Queue len = 1, overwriting, consuming by peeking.
*/
void create_mq_co2(void);

/*
Task to read CO2 measurements coninuously. 
Sleep betweeen measurements.
*/
void task_co2(void);

/*
Add SCD40 device to I2C bus and update device handle glob var.
*/
esp_err_t sensor_init(void);

