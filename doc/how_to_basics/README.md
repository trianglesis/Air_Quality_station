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


## CMAKE

Updating `main\CMakeLists.txt` as soon as new modules added:

```text

```

Using [example](https://github.com/trianglesis/ESP32-C6-LCD-1.47-Test-LVGL/blob/c95cb298858690e018c0155daccdb1463647a111/main/CMakeLists.txt) as ref.


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

```log
I (571) SD-SPI: Filesystem mounted
Name: SD64G
Type: SDHC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 59638MB
CSD: ver=2, sector_size=512, capacity=122138624 read_bl_len=9
SSR: bus_width=1
I (571) SD-SPI: SD Card detected, size: 59638 MB
I (581) SD-SPI: Opening file sdcard/hello.txt
E (581) SD-SPI: Failed to open file for writing
I (581) SD-SPI: Computing hello.txt MD5 hash and test reading
E (591) SD-SPI: Failed to open hello.txt
```