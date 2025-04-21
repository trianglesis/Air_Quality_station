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

i2c_master_bus_handle_t bus_handle;

static const char *TAG = "i2c_driver";

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

    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C master bus initialized successfully.");
        return ESP_OK;
    } else if (ret == ESP_ERR_INVALID_ARG ) {
        ESP_LOGE(TAG, "I2C bus initialization failed because of invalid argument!");
        return ESP_FAIL;
    } else if (ret == ESP_ERR_NO_MEM ) {
        ESP_LOGE(TAG, "Create I2C bus failed because of out of memory!");
        return ESP_FAIL;
    } else if (ret == ESP_ERR_NOT_FOUND ) {
        ESP_LOGE(TAG, "No more free bus!");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Master bus cannot be added by unknown error!");
        return ESP_FAIL;
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
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Master bus obtained!");
        return ESP_OK;
    }
}


/*
Returns added device handler.
*/
i2c_master_dev_handle_t master_bus_device_add(i2c_device_config_t device_config) {
    esp_err_t ret;
    i2c_master_dev_handle_t device_handle;
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C Bus handle is NULL!");
    }
    ret = i2c_master_bus_add_device(bus_handle, &device_config, &device_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Create I2C master device successfully!");
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "I2C bus initialization failed because of invalid argument!");
        // return ESP_FAIL;
    } else if (ret == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "Create I2C bus failed because of out of memory. !");
        // return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Device cannot be added, uknown error!");
        // return ESP_FAIL;
    }
    return device_handle;
}


/*

Parameters:
    bus_handle -- [in] I2C master device handle that created by i2c_master_bus_add_device.
    address -- [in] I2C device address that you want to probe.
    xfer_timeout_ms -- [in] Wait timeout, in ms. Note: -1 means wait forever (Not recommended in this function).

Returns:
    ESP_OK: I2C device probe successfully
    ESP_ERR_NOT_FOUND: I2C probe failed, doesn't find the device with specific address you gave.
    ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
*/
esp_err_t master_bus_probe_address(uint16_t address, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_probe(bus_handle, address, xfer_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Tested device address on the I2C BUS = OK!");
        return ESP_OK;
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Device cannot be found by current address at I2C master bus!");
        return ESP_FAIL;
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "TIMEOUT is not an expected failure, which indicated that the i2c probe not works properly, usually caused by pull-up resistors not be connected properly.\nSuggestion check data on SDA/SCL line to see whether there is ACK/NACK signal is on line when i2c probe function fails.\n\n");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Device cannot be found by current address at I2C master bus! Unspecified error!");
        return ESP_FAIL;
    }
}

esp_err_t i2c_receive(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms) {
    esp_err_t ret;
    ret = i2c_master_receive(i2c_dev, read_buffer, read_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot receive from device!");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Received from device!");
        return ESP_OK;
    }
}

esp_err_t i2c_transmit_and_receive(
    i2c_master_dev_handle_t i2c_dev, 
    const uint8_t *write_buffer, 
    size_t write_size, 
    uint8_t *read_buffer, 
    size_t read_size, 
    int xfer_timeout_ms
) {
    esp_err_t ret;
    ret = i2c_master_transmit_receive(i2c_dev, write_buffer, write_size, read_buffer, read_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot transmit and receive from device!");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Transmitted and received from device!");
        return ESP_OK;
    }
}

/*
Perform a write transaction on the I2C bus. The transaction will be undergoing until it finishes or it reaches the timeout provided. 

Parameters:
    i2c_dev -- [in] I2C master device handle that created by i2c_master_bus_add_device.
    write_buffer -- [in] Data bytes to send on the I2C bus.
    write_size -- [in] Size, in bytes, of the write buffer.
    xfer_timeout_ms -- [in] Wait timeout, in ms. Note: -1 means wait forever.
Returns:
    ESP_OK: I2C master transmit success
    ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
    ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
*/
esp_err_t i2c_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t* write_buffer, size_t write_size, int xfer_timeout_ms) {
    esp_err_t ret;

    ret = i2c_master_transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Transmitted to device!");
        return ESP_OK;
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "I2C master transmit parameter invalid!");
        return ESP_FAIL;
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash!");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Cannot transmit to device, unknown error!");
        return ESP_FAIL;
    }
}


