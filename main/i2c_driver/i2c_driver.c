/*
My custom implementation of I2C bus and devices.

It may luck a lot of features, since I only need it for a few sensors and readings.

Used a few examples as inspiration:

https://github.com/Sensirion/embedded-i2c-scd4x
https://github.com/UncleRus/esp-idf-lib

I2C bus in IDF:

https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/i2c.html

Thread safety was promised:
- https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/i2c.html#thread-safety
*/

#include "i2c_driver.h"

static const char *TAG = "i2c_master";

/*
No need to return the master bus, we can get it with: 

i2c_master_bus_handle_t handle;
ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &handle));

https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/i2c.html#get-i2c-master-handle-via-port

*/
esp_err_t master_bus_init() {
    esp_err_t ret;

    // New I2C bus setup, new driver used, from IDF 5.4+
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = COMMON_SCL_PIN,
        .sda_io_num = COMMON_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot init master bus!");
        while (1);
    } else {
        ESP_LOGI(TAG, "Master bus added!");
        return ESP_OK;
    }

}


/*

Get the master bus, if it was already created.

Assign var and it will be updated with master bus handle:

i2c_master_bus_handle_t handle

https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32c6/api-reference/peripherals/i2c.html#get-i2c-master-handle-via-port

*/
esp_err_t master_bus_get(i2c_master_bus_handle_t handle) {
    esp_err_t ret;

    ret = i2c_master_get_bus_handle(0, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot get master bus!");
        while (1);
    } else {
        ESP_LOGI(TAG, "Master bus obtained!");
        return ESP_OK;
    }
}


/*

i2c_master_dev_handle_t device handle;

*/
esp_err_t master_bus_device_add(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t device_handle, 
    uint16_t device_address) {
    esp_err_t ret;

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &device_config, &device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot add a device to master bus!");
        while (1);
    } else {
        ESP_LOGI(TAG, "Device added to master bus!");
        return ESP_OK;
    }
}

esp_err_t master_bus_probe_address(i2c_master_bus_handle_t bus_handle, uint16_t address, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_probe(bus_handle, address, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tested device address on the I2C BUS = OK!");
        while (1);
    } else {
        ESP_LOGE(TAG, "Device cannot be found by current address at I2C master bus!");
        return ESP_OK;
    }
}

esp_err_t i2c_receive(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_receive(i2c_dev, read_buffer, read_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot receive from device!");
        while (1);
    } else {
        ESP_LOGE(TAG, "Received from device!");
        return ESP_OK;
    }
}

esp_err_t i2c_transmit_and_receive(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_transmit_receive(i2c_dev, write_buffer, write_size, read_buffer, read_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot transmit and receive from device!");
        while (1);
    } else {
        ESP_LOGE(TAG, "Transmitted and received from device!");
        return ESP_OK;
    }
}

esp_err_t i2c_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot transmit to device!");
        while (1);
    } else {
        ESP_LOGE(TAG, "Transmitted to device!");
        return ESP_OK;
    }
}


