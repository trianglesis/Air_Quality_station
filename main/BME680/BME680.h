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

extern QueueHandle_t mq_bme680;

struct BMESensor {
    float temperature;
    float humidity;
    float pressure;
    float resistance;
};


void bme680_reading(void * pvParameters);
void create_mq_bme680(void);