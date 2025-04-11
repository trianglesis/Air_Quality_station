# How to basics

Here I collect all my detailed walkthrouh. 

Trying to understand the C world from Python-dev perspective.


## Beautiful code organozation

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