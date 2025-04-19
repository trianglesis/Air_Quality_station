# How to basics

Here I collect all my detailed walkthrouh. 

Trying to understand the C world from Python-dev perspective.


## Beautiful code organization

I want to recreate the example project file and submodules setup from [Waveshare](https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47)

I'd like to have display, lvgl, wifi, led, sensor and other logical setups in separate files and dirs for clean view and faster dev.


## Display

Setting up display driver similar to this [example](https://github.com/trianglesis/ESP32-C6-LCD-1.47-Test-LVGL/blob/c95cb298858690e018c0155daccdb1463647a111/main/LCD_Driver)

With actual code from [this](https://github.com/trianglesis/esp32-c6_LVGL/blob/338e3427ca3d009852cf9369b917257a4801f9ac/main/main.c)


```text
 LBS147TC-IF15 is a 172RGBX320 dot-matrix TFT LCD module. This module is composed of a TFT LCD 
 Panel, driver ICs, FPC and a Backlight unit. 
```

From datasheet

```text
LCD Size            1.47 inch
Display             Mode Normally black
Resolution          172(H)RGB x320(V) pixels
Pixel pitch         0.0337(H) x 0.1011(V) mm 
Active area         17.3892(H) x 32.352(V) mm
Pixel arrangement   RGB Vertical stripe
Interface           4 Line SPI
Display Colors      262K colors
Drive IC            ST7789V3
Backlight           2 White LED Parallel
```

Setup all static configs `main\display_driver\ST7789V3.h`:
- pins
- resolutions

Copied a typical setup from Waveshare example and LCD-only related code from modern setup at my example.

Setup LCD usage `main\display_driver\ST7789V3.c`:
- display_init to init LCD with backlight



## LVGL

Install:

Usual 1st cmd OR better local (to be able to configure lvgl for once)

- `idf.py add-dependency "lvgl/lvgl^9.2.2"`
- `git submodule add https://github.com/lvgl/lvgl.git components/lvgl`

Into the ignored folder (not to add the full other repo in my repo): 

- `git submodule add -f https://github.com/lvgl/lvgl.git components/lvgl`

IMPORTANT: Switch branch to a released last: `9.2.2`

- `cd .\components\lvgl\`
- `git checkout v9.2.2`
- `git checkout tags/1.0.0`


Setup static vars `main\lvgl_driver\lvgl_driver.h`:

```code
// contains internal graphic buffer(s) called draw buffer(s)
extern lv_disp_draw_buf_t disp_buf;
// contains callback functions
extern lv_disp_drv_t disp_drv;
```

I cannot find a proper example of such variables usage in new version LVGL, so I only use ones I use in my example.

I also reused resolution variables from display driver header:

```text
// Reuse vars from display driver for visibility.
#define LV_DISP_HOR_RES = DISP_HOR_RES;
#define LV_DISP_VER_RES = DISP_VER_RES;
```

Note: use this when display is rotated:
- change `Offset_X 34` if 0deg (Y 0)
- change `Offset_Y 34` if 270deg (X 0)

```text
// Rotate 90deg and compensate buffer change
// TODO: Move it to the main file for better visibility
#define Offset_X 34 // 0 IF NOT ROTATED 270deg
#define Offset_Y 0  // 34 IF ROTATED 270deg
```

PRO: Later check how to add events to LVGL objects, so it will show WiFi AP icon automatically when AP is active and user is connected

## CMAKE

- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#example-component-cmakelists

Updating `main\CMakeLists.txt` as soon as new modules added:

```text

```

Using [example](https://github.com/trianglesis/ESP32-C6-LCD-1.47-Test-LVGL/blob/c95cb298858690e018c0155daccdb1463647a111/main/CMakeLists.txt) as ref.


Adding new lib: `esp-idf-lib` in repo root reveals a lot of outdated in led:

```log
[1017/1611] Building C object esp-idf/led_strip/CMakeFiles/__idf_led_strip.dir/led_strip.c.obj
In file included from D:/Projects/ESP/projects/ESP32-C6-OLED/Air_Quality_station/esp-idf-lib/components/led_strip/led_strip.h:40,
                 from D:/Projects/ESP/projects/ESP32-C6-OLED/Air_Quality_station/esp-idf-lib/components/led_strip/led_strip.c:33:
D:/Projects/ESP/Espressif/v5.4.1/esp-idf/components/driver/deprecated/driver/rmt.h:18:2: warning: #warning "The legacy RMT driver is deprecated, please use driver/rmt_tx.h and/or driver/rmt_rx.h" [-Wcpp]
   18 | #warning "The legacy RMT driver is deprecated, please use driver/rmt_tx.h and/or driver/rmt_rx.h"
```


## Tasks and Queues

Use one of two approaches:

Queue len > 1 - is to write multiple messages, and the program can read any of them, even if one is outdated, it will be outdated in matter of ms, which is still good, and can be used as fall back method, because queue is always filled with something.

But this approach requires a reading from queue by desctruction of the message or cleaning the queue if it is half full, to prevent the queue to become 100% full. [Doc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#_CPPv417xQueueGenericSend13QueueHandle_tPCKv10TickType_tK10BaseType_t)

Queue len = 1 - just rewrite the one message (co2 measurement) and never read from this queue by destructive methods. And no need to clean the queue. [Doc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#c.xQueueOverwrite)


I've tested the 1st, and now will try the second approach.
Tested the queue overwrite and it is better to use.

Leaving both examples in code covered by ifs.

Doc:
- https://stackoverflow.com/questions/75908881/esp32-shared-variable-between-tasks
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#c.xQueueOverwrite


## Square Line studio

Use pics from:
- https://www.flaticon.com/search?word=gauge


## SD Card

There is no support for fast driver mode, probably:
- Fast [SD MMC](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/sdmmc_host.html)

Initial tests are working with Kingston 128Gb and GoodRam 64Gb SD Cards [test](https://github.com/trianglesis/ESP32-C6-LCD-1.47-Test-LVGL/blob/c95cb298858690e018c0155daccdb1463647a111/main/LVGL_UI/LVGL_Example.c)

Examples are fine, but one of my board's SD port is dead!

Seems like you should init SPI only once, use var `SDCard_Size` as check to skip SPI init when LCD load.

```log
I (457) SD-SPI: Filesystem mounted
Name: SD64G
Type: SDHC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 59638MB
CSD: ver=2, sector_size=512, capacity=122138624 read_bl_len=9
SSR: bus_width=1
I (457) SD-SPI: SD Card detected, size: 59638 MB
I (467) SD-SPI: Opening file /sdcard/hello.txt
I (487) SD-SPI: File written
I (487) SD-SPI: Computing hello.txt MD5 hash and test reading
I (487) SD-SPI: Computed MD5 hash of alice.txt: 25602f001b1b12367cfc90b905f0c6e7
I (497) SD-SPI: Reading hello.txt
I (497) SD-SPI: Read from hello.txt: Test file created!
I (497) Display_ST7789V3: Skip SPI Bus init at LCD as it was initialized at SD Card driver!
```

Next write a log to SD card, or serve a SQLite something?

Use this example for proper log writing on SD Card: [example](https://github.com/i400s/tmp-sdcard/blob/main/main/sdcard_main.c)

Or can I load SD Card data from the build?
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/fatfs.html#fatfs-partition-generator


NOTE: It seems we MUST have an SPI flash partition to serve initial files like `index.html` to be able to upload other files to SD Card

OR: We need to plug-unplug SD Card to upload html files, OR create a write method at the firmware itself, which is actually recreating the `littlefs` approach but more complex. However we can use SD card almost limitless!


## Web Server


Can use any example:

- [IDF Example file serving](https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/protocols/http_server/file_serving/README.md)
- [My example of web server but no upload](https://github.com/trianglesis/webserver-w-ap-portal-dns-redirect/blob/3c77dccb4fd825f2f68bc036e616b9afba420bd2/README.md)
- Add more

### Web Server with file server

Split example on two logical parts:

Web Server Core module:
- Is just an empty webserver with default host `/` of root and redirection to `/upload` of root is empty.
- All other methods and APIs are here

File server methods and URI at `file_server` module:
- [file_server.c](https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/protocols/http_server/file_serving/main/file_server.c)
- Only functions related to upload files

Allow upload files ONLY when connected to a trusted Wifi, but not at AP mode?

UPD: Get web and file server in ONE module.

TODO: Make dynamical load\unload fileserver handles as soon as local WiFi lost

### File serving

Initially we must to have an SPI `folder` made directly at board flash, to store the most basic files, like: `index.html`
But there is no need to keep this file as main page, only serve the `/upload/` url with this SPI flash.
Later using this endpoing we can upload `real` site and files at the root of SD Card, and place the webserver root `/` index file at sd card!

It's working, try to upload files to SD Card

```shell
curl -X POST --data-binary @sd_card/index.html  http://192.168.1.225:80/upload/index.html
curl -X POST --data-binary @index.html  http://192.168.1.225:80/upload/index.html
curl -X POST http://192.168.1.225:80/delete/index3.txt

# New added replace ARG
curl -X POST --data-binary @index.html  http://192.168.1.225:80/upload/index.txt?replace=1
File uploaded successfully

```
Cannot upload ONLY html files!

```log
E (158022) file_server: Failed to create file : /sdcard/tst.html
W (158022) httpd_txrx: httpd_resp_send_err: 500 Internal Server Error - Failed to create file
W (158022) httpd_uri: httpd_uri: uri handler execution failed

I (32682) file_server: Sending file : /index.txt (307 bytes)...
I (32682) file_server: File sending complete

I (112102) co2station: sent data = 111
I (113102) co2station: sent data = 112
I (114112) co2station: sent data = 113
I (115122) co2station: sent data = 114
I (115902) file_server: Receiving file : /index.s...
I (115902) file_server: Remaining size : 307
I (115912) file_server: File reception complete
I (116132) co2station: sent data = 115
I (117142) co2station: sent data = 116
I (118152) co2station: sent data = 117
I (118762) file_server: Receiving file : /index.h...
I (118762) file_server: Remaining size : 307
I (118772) file_server: File reception complete
I (119162) co2station: sent data = 118
I (120172) co2station: sent data = 119
I (120602) file_server: Receiving file : /index.ht...
I (120602) file_server: Remaining size : 307
I (120622) file_server: File reception complete
I (121182) co2station: sent data = 120
I (122202) co2station: sent data = 121
I (122452) file_server: Receiving file : /index.htm...
I (122452) file_server: Remaining size : 307
I (122462) file_server: File reception complete
I (123202) co2station: sent data = 122
I (124222) co2station: sent data = 123
E (124292) file_server: Failed to create file : /sdcard/index.html
W (124292) httpd_txrx: httpd_resp_send_err: 500 Internal Server Error - Failed to create file
W (124292) httpd_uri: httpd_uri: uri handler execution failed

```

Now check this: https://stackoverflow.com/a/72530185 `FATFS_LONG_FILENAMES`

It was the reason!



#### LittleFS


Add this at `main\CMakeLists.txt` to be able to flash local files using build firmware.

- `flash_data` is a dir name at the root of current project

```text
littlefs_create_partition_image(littlefs ../flash_data FLASH_IN_PROJECT)

```


### Queue and messages

You shoud decide how to work with the queue better.
Adding new messages and check each time if queue is not full or empty.
Rewriting the message with queue len = 1 without thinking about full\empty\old values in the queue.

Thewre are two ways to insert messages:
- `xQueueGenericSend` - just send appending a new mesage. (to front of the queue for example)
- `xQueueOverwrite` - overwrite the last message with new.

```code
if (mq_co2_len > 1) {
    // Always check the space and queue len, clean if half-full. Queue read is non-destructive always.
    int queue_messages = uxQueueMessagesWaiting(mq_co2);
    int queue_space = uxQueueSpacesAvailable(mq_co2);
    if (queue_messages > 1 || queue_space < 3) {
        ESP_LOGI(TAG, "Queue is filled with messages: %d, space left: %d - cleaning the queue!", 
            queue_messages, 
            queue_space);
        xQueueReset(mq_co2);
    }
    // When queue is len > 1
    if (xQueueGenericSend(mq_co2, (void *)&fake_co2_counter, 0, queueSEND_TO_FRONT) != pdTRUE) {
        ESP_LOGE(TAG, "Queue full and it should be emtied!\n");
    }
} else {
    // No need to clean if xQueueOverwrite
    // When queue is len = 1, return is negligible
    xQueueOverwrite(mq_co2, (void *)&fake_co2_counter);
}
```


There are two ways to get the message from the queue:
- `xQueuePeek` to get without removing
- `xQueueReceive` to get with removing


```code
if (mq_co2 > 1) {
    // Destructive read
    if (xQueueReceive(mq_co2, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
        // ESP_LOGI(TAG, "received data = %d", co2_counter);
    } else {
        // Skip drawing if there is no mesages left
        // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
    }
} else {
    // Queue recieve, non destructive! Always with xQueueOverwrite
    if (xQueuePeek(mq_co2, (void *)&co2_counter, xTicksToWait) == pdTRUE) {
        // ESP_LOGI(TAG, "received data = %d", co2_counter);
    } else {
        // Skip drawing if there is no mesages left
        // ESP_LOGI(TAG, "Did not received data in the past %d ms", to_wait_ms);
    }
}
```


# Sensors


## scd4x

https://esp-idf-lib.readthedocs.io/en/latest/groups/scd4x.html


Install:

Usual 1st cmd OR better local:

- `git submodule add https://github.com/Sensirion/embedded-i2c-scd4x.git`
- `git submodule add https://github.com/UncleRus/esp-idf-lib.git`

OR
- `cd components`
- `git clone https://github.com/UncleRus/esp-idf-lib.git`


Into the ignored folder (not to add the full other repo in my repo): 

- `git submodule add -f https://github.com/Sensirion/embedded-i2c-scd4x.git`
- `git submodule add -f https://github.com/UncleRus/esp-idf-lib.git`

If branch

- `cd .\components\embedded-i2c-scd4x`
- `git checkout branch`

Use files as you wish, do not waste time for adding a full lib as a component!

### Not working


```log
E (949) i2cdev: Could not write to device [0x62 at 0]: -1 (ESP_FAIL)
ESP_ERROR_CHECK failed: esp_err_t 0xffffffff (ESP_FAIL) at 0x420110d0
--- 0x420110d0: co2_scd4x_reading at D:/Projects/ESP/projects/ESP32-C6-OLED/Air_Quality_station/main/sensor_co2/co2_sensor.c:82 (discriminator 1)

file: "./main/sensor_co2/co2_sensor.c" line 82
func: co2_scd4x_reading
expression: scd4x_wake_up(&dev)

abort() was called at PC 0x4080b6d1 on core 0
--- 0x4080b6d1: _esp_error_check_failed at D:/Projects/ESP/Espressif/v5.4.1/esp-idf/components/esp_system/esp_err.c:49
```

Nothing works!
Try to debug I2C bus with examples from IDF and also check options at `menuconfig`

- [I2C tools](https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/peripherals/i2c/i2c_tools/README.md)

Check VARS for your case, `JTAG` variant allows me to use CMD right in the `VSCode`!

`#define CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG 1`

Using tools from above:
Pins connected `0` and `1`

Setup (optional)

`i2cconfig  --port=0 --freq=100000 --sda=1 --scl=0`
`i2cconfig  --port=0 --freq=100000 --sda=19 --scl=18`

i2cconfig  --port=0 --freq=100000 --sda=6 --scl=7
i2cconfig  --port=0 --freq=10000 --sda=6 --scl=7
i2cconfig  --port=0 --freq=1000 --sda=6 --scl=7
i2cconfig  --port=0 --freq=100 --sda=6 --scl=7


```text
i2c-tools> i2cdetect
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: 00 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- 62 -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

```text
i2c-tools> i2cdump -c 0x62
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
00: E (416302) i2c.master: I2C transaction unexpected nack detected
E (416302) i2c.master: s_i2c_synchronous_transaction(924): I2C transaction failed
E (416302) i2c.master: i2c_master_transmit_receive(1220): I2C transaction failed

i2c-tools> i2cget -c 0x62 -r 0x3682 -l 8
E (117253) i2c.master: I2C transaction unexpected nack detected
E (117253) i2c.master: s_i2c_synchronous_transaction(924): I2C transaction failed
E (117253) i2c.master: i2c_master_transmit_receive(1220): I2C transaction failed
W (117253) cmd_i2ctools: Read failed
```

Test manual bits:

`i2cset -c 0x62 -r 0x36F6 -l 1`

Use one:
```code
#define CMD_POWER_DOWN                             (0x36E0)
#define CMD_WAKE_UP                                (0x36F6)
#define CMD_GET_DATA_READY_STATUS                  (0xE4B8)
#define CMD_GET_SERIAL_NUMBER                      (0x3682)
#define CMD_PERFORM_SELF_TEST                      (0x3639)
#define CMD_REINIT                                 (0x3646)
#define CMD_MEASURE_SINGLE_SHOT                    (0x219D)
#define CMD_MEASURE_SINGLE_SHOT_RHT_ONLY           (0x2196)
```

i2cset -c 0x62 -r 0x36F6 -l 1

cmd_i2ctools: Write Failed

i2cget -c 0x62 -r 0x3682 -l 8
i2cget -c 0x62 -r 0xE4B8 -l 8
i2cget -c 0x62 -r 0x00 -l 1
i2cget -c 0x62 -r 0x02 -l 8

This is CO2 Sensor!

Setup the BME680 next:

`i2cconfig  --port=0 --freq=100000 --sda=3 --scl=2`

```text
i2c-tools> i2cconfig  --port=0 --freq=100000 --sda=3 --scl=2
i2c-tools> i2cdetect
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- 77 -- -- -- -- -- -- -- -- 
```
This is BME680!

Tests

```text
i2c-tools> i2cdump -c 77
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
00: E (416302) i2c.master: I2C transaction unexpected nack detected
E (416302) i2c.master: s_i2c_synchronous_transaction(924): I2C transaction failed
E (416302) i2c.master: i2c_master_transmit_receive(1220): I2C transaction failed
```

## bme680

- https://esp-idf-lib.readthedocs.io/en/latest/groups/bme680.html
- https://github.com/UncleRus/esp-idf-lib




# Assets

- https://www.flaticon.com/free-icons/

# END

## END

### END