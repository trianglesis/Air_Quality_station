# https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example/blob/main/main/CMakeLists.txt

# Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} 
                        "ui/*.c"
                        "ui/components/*.c"
                        "ui/screens/*.c"
                        "ui/images/*.c"
)

idf_component_register(
    SRCS 
        "main.c"  
        "display_driver/ST7789V3.c"
        "lvgl_driver/lvgl_driver.c"
        "led_driver/led_driver.c"
        "card_driver/card_driver.c"
        "littlefs_driver/local_flash.c"
        "wifi/wifi_ap.c"
        "webserver/webserver.c"
        "i2c_driver/i2c_driver.c"
        "sensor_co2/sensiniron/sensirion_common.c"
        "sensor_co2/sensiniron/sensirion_i2c.c"
        "sensor_co2/sensiniron/scd4x_i2c.c"
        "sensor_co2/co2_sensor.c"
        "sensor_temp/bme680.c"
        "sensor_temp/temp_sensor.c"
        ${SRC_UI}
    INCLUDE_DIRS 
        "." 
        "./display_driver" 
        "./lvgl_driver" 
        "./led_driver"
        "./card_driver"
        "./littlefs_driver"
        "./wifi"
        "./webserver"
        "./i2c_driver"
        "./sensor_co2/sensiniron/"
        "./sensor_co2"
        "./sensor_temp"
        "ui"
)

idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_options(${lvgl_lib} PRIVATE -Wno-format)

# Note: you must have a partition named the first argument (here it's "littlefs")
# in your partition table csv file.
littlefs_create_partition_image(littlefs ../flash_data FLASH_IN_PROJECT)

# https://www.waveshare.com/wiki/Waveshare_SquareLine_Studio
# This setup recompiles everything each time!