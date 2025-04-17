# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# idf_component_register(
#     SRCS 
#         "components/esp-idf-lib/components/scd4x/scd4x.c"
#     INCLUDE_DIRS 
#         "./components/esp-idf-lib/components/scd4x"
# )
# set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/components/esp-idf-lib/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Air_Quality_station)