/*
My custom implementation of I2C bus and devices.

It may luck a lot of features, since I only need it for a few sensors and readings.

Used a few examples as inspiration:

https://github.com/Sensirion/embedded-i2c-scd4x
https://github.com/UncleRus/esp-idf-lib

I2C bus in IDF:

https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/i2c.html
*/

#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c_master.h"

#define I2C_PORT 0

/*
Common pins for all sensors.
Currently SCD40 and BME680
*/
#define COMMON_SDA_PIN 0
#define COMMON_SCL_PIN 1

/*
Standard freq for my sensors.
*/
#define I2C_FREQ_HZ 100000 // 100kHz - usual


/*
Create a new master bus
*/
esp_err_t master_bus_init();

/*
Gen handle for already created bus.
*/
esp_err_t master_bus_get(i2c_master_bus_handle_t handle);


/*
Add a device to master bus.

1. Init master_bus_init
2. Get handle master_bus_get - it shoud be saved at init
3. Now add device

Note: obtaining bus may be added into this function, 
    but now I'd rather leave those steps pretty visible 
    for me to understand the full process.

Return device handle!
*/
i2c_master_dev_handle_t master_bus_device_add(i2c_device_config_t device_config);

esp_err_t master_bus_probe_address(uint16_t address, int xfer_timeout_ms);

esp_err_t i2c_receive(i2c_master_dev_handle_t i2c_dev, uint8_t* read_buffer, size_t read_size, int xfer_timeout_ms);

esp_err_t i2c_transmit_and_receive(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer, size_t read_size, int xfer_timeout_ms);

esp_err_t i2c_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size, int xfer_timeout_ms);