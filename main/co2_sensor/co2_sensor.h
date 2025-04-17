// #pragma once

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "esp_log.h"


#define wait_co2_next_measure 1000 // How often to measure CO2 level in ms
#define wait_co2_to_led 1000 // How often change LED colour for CO2 measurements in ms

extern QueueHandle_t mq_co2;


// TODO: Change to real!
#define SCD4X_SCL_PIN 10
#define SCD4X_SDA_PIN 11

typedef int esp_err_t;

struct esp32_i2c_config {
    uint32_t freq;
    uint8_t addr;
    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;
    bool sda_pullup;
    bool scl_pullup;
};

void co2_reading(void * pvParameters);
void led_co2(void * pvParameters);
void create_mq_co2(void);

