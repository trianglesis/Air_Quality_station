# Setup


All steps and problems will keep here.

How to BASICS for a very begginer is also [here](how_to_basics/README.md)


## Changing most common options.

Make partitions larger by default, utilize all 4Mb is SPI flash.

- Use `menuconfig` to change default flash size. (`ESPTOOLPY_FLASHSIZE`)
- Copy example `partitions.csv` into the project root and just change the last value to `3512K`
- Use `menuconfig` and set pertitions to custom csv. (`PARTITION_TABLE_TYPE`)
- Use `menuconfig` set HTTP header limit to `1024`. (`HTTPD_MAX_REQ_HDR_LEN`)
  - Later to accept mobile browsers UserAgent long string at captive portal
- Optionally change log level under `LOG_DEFAULT_LEVEL`
- Optionally change log level under `LOG_MAXIMUM_LEVEL`

- LATER: Add more setup options to save time, like SD Card flash setup, WiFI and etc



## Add modules






### LVGL

Adding LVGL lib as IDF submodule: 

- `idf.py add-dependency "lvgl/lvgl^9.2.2"`

Use [lvgl_conf_template.h](../managed_components/lvgl__lvgl/lv_conf_template.h) as template for `lvgl_conf.h`

Copy template file to upper directory:

```text
{PROJ_ROOT}/managed_components/lvgl__lvgl/..
{PROJ_ROOT}/managed_components/lvgl_conf.h
```

Modify `lvgl_conf.h` and change 0 to 1

```code
#if 0 /*Set it to "1" to enable content*/
```

Remember to check this section and enable all fonts you need!

```text
/*==================
 *   FONT USAGE
 *===================*/

/* Montserrat fonts with ASCII range and some symbols using bpp = 4
 * https://fonts.google.com/specimen/Montserrat */
#define LV_FONT_MONTSERRAT_8 1
```

Clean the build `Full clean` and run `menuconfig` option again.
- It should now go into LVGL dir and collect all options there.


- Use `menuconfig` set LVGL option `LV_CONF_SKIP` to not skip custom `lvgl_conf.h`! (actually can build without custom lv_conf.h)
- Check if color depth = 16
- Check `Font usage` - optional if you use more fonts
- Go to `Devices` - check box for `LV_USE_ST7789` (actually does not mater)
- Examples: disable, do not build.
- Themes: (optional) you can select default theme. (I have not change anything yet)

Can now run build, just to check, not flash.



### Led


`idf.py add-dependency "espressif/led_strip^3.0.1"`

And follow usual setup


### Square Line Studio

We can use [Square Line](https://docs.squareline.io) to draw graphics for us!


Create new project: ![Project create](pic/SQLine_setup_1.png)

- Choose `VS Code with SDL`
- Set screen dimensions according to the hardware.
- No rotations yet.
- Color `16 bit`.
- LVGL `9.2.2` - should be the same as loaded from `IDF Component Library`
- Dark by defaut is good to have
- Select dir to save the project, I use this repo

Create new project and open `File` -> `Project Settings` after.

![Setup step 2](pic/SQLine_setup_2.png)

Set `File Export` paths:

- Root as `{PROJECT_ROOT}/main/`
- UI as `{PROJECT_ROOT}/main/ui`
- Check `lvgl.h` should use direct path, as we using lvgl from IDF components, it will be initialized at root.
  - The file `ui.h` after exporting UI from SQ Line Studio will have this direct import: `#include "lvgl.h"`

Here you can change options and rotate UI. 
- You should also rotate the display in `lvgl_init` as explained in this [topic](https://forum.lvgl.io/t/gestures-are-slow-perceiving-only-detecting-one-of-5-10-tries/18515/101)

Exporting - no need to use `flat export`, run as default:

![Export](pic/SQLine_export_1.png)


Modify the `main\CMakeLists.txt`

I used the approach from this [example](https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example/blob/main/main/CMakeLists.txt)

```text
# https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example/blob/main/main/CMakeLists.txt

# Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS "main.c"  
                              ${SRC_UI}
                    INCLUDE_DIRS "." 
                                 "ui"
                       )
```

Modify the project `main.c`
- Add subdir import of exported graphics: `#include "ui/ui.h"`


### SD Card

Select one of approaches:

- Fast [SD MMC](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/sdmmc_host.html)
- Simple [SD SPI](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/sdspi_host.html)

Setup first code carcass based on example (from waveshare), read the doc, use another example to add wearing funcs:
- https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/sdmmc/main/sd_card_example_main.c
- https://github.com/espressif/esp-idf/blob/master/examples/storage/wear_levelling/main/wear_levelling_example_main.c

For FAT creation and auto download files at the board:

- Create custom partitions file: `partitions.csv`
- Set a proper SD Size max: `64GB=64000MB`

Use example for [SPIFFS](https://github.com/espressif/esp-idf/tree/master/examples/storage/spiffsgen)

Also update `partitions.csv` adding SD card partition, [DOC](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/partition-tables.html#partition-tables)

`Sizes and offsets can be specified as decimal numbers, hex numbers with the prefix 0x, or size multipliers K or M (1024 and 1024*1024 bytes).`

New table for 64GB card, will use half of the storage:

- Use 62000MB, to fit into sizes roudings

```text
# ESP-IDF Partition Table
# Name,     Type,   SubType,    Offset,     Size,   Flags
nvs,        data,   nvs,        0x9000,     24K,
phy_init,   data,   phy,        0xf000,     4K,
factory,    app,    factory,    0x10000,    3512K,
sdcard,     data,   fat,        ,           62000M,
```

Update `main` dir CMAKE file [with fs gen](https://github.com/espressif/esp-idf/blob/master/examples/storage/spiffsgen/main/CMakeLists.txt):

`spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)`


# Links, help, forums and etc

Use this list to get more info about each step here and all possible workarounds.

- [Waveshare](https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47)
- [LVGL](https://docs.lvgl.io/9.2/overview/index.html)
  - [Widgets](https://docs.lvgl.io/9.2/API/widgets/index.html)
- This [topic](https://forum.lvgl.io/t/gestures-are-slow-perceiving-only-detecting-one-of-5-10-tries/18515/101) helped me a lot to understand the proper LCD driver setup and display rotation and colors.
  - The [post](https://forum.lvgl.io/t/gestures-are-slow-perceiving-only-detecting-one-of-5-10-tries/18515/59) about display setup.
  - The [post](https://forum.lvgl.io/t/gestures-are-slow-perceiving-only-detecting-one-of-5-10-tries/18515/60) about rotation.
  - Fix rotated display dead zones [post](https://forum.lvgl.io/t/gestures-are-slow-perceiving-only-detecting-one-of-5-10-tries/18515/86)
  - Fix display invertion [post](https://forum.lvgl.io/t/lvgl-9-2-2-esp32-c6-lcd-1-47-idf-5-4-1-st7789v3/20871)
- This [repo](https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example) helped me to understand `Square Line Studio` expoting and compiling.