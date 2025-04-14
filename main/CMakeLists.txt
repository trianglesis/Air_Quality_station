# https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example/blob/main/main/CMakeLists.txt

# Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(
    SRCS 
        "main.c"  
        "display_driver/ST7789V3.c"
        "lvgl_driver/lvgl_driver.c"
        "led_driver/led_driver.c"
        "card_driver/card_driver.c"
        ${SRC_UI}
    INCLUDE_DIRS 
        "." 
        "./display_driver" 
        "./lvgl_driver" 
        "./led_driver"
        "./card_driver"
        "ui"
        )


# spiffs_create_partition_image(sdcard ../sd_card FLASH_IN_PROJECT)

# https://www.waveshare.com/wiki/Waveshare_SquareLine_Studio
# This setup recompiles everything each time!

# Add sources from ui directory
# file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR}
#                          "*.c"
#                          "ui/*.c"
#                          "ui/components/*.c"
#                          "ui/screens/*.c"
# )


# idf_component_register(
#     SRCS 
#         "main.c"  
#         "display_driver/ST7789V3.c"
#         "lvgl_driver/lvgl_driver.c"
#         "led_driver/led_driver.c"
#         ${SRC_UI}
#     INCLUDE_DIRS 
#         "." 
#         "./display_driver" 
#         "./lvgl_driver" 
#         "./led_driver" 
#         "ui"
#         )
        
# idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
# target_compile_options(${lvgl_lib} PRIVATE -Wno-format)