# https://github.com/serdartoren/ESP32-S3_HMI_Squareline_Example/blob/main/main/CMakeLists.txt

# Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(
    SRCS 
        "main.c"  
        "display_driver/ST7789V3.c"
        "lvgl_driver/lvgl_driver.c"
        ${SRC_UI}
    INCLUDE_DIRS 
        "." 
        "./display_driver" 
        "./lvgl_driver" 
        "ui"
        )
