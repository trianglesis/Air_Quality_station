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

static QueueHandle_t mq_co2;
// Standard queue with len > 1, can consume any of multiple messages
// static const uint8_t msg_queue_len = 5;
// Or overwittable queue
static const uint8_t mq_co2_len = 1;
static volatile bool timed_out;

void co2_reading(void * pvParameters);
void led_co2(void * pvParameters);
void create_mq_co2(void);