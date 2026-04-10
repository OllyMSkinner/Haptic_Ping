cmake_minimum_required(VERSION 3.10)
project(gpio_blink)

set(CMAKE_CXX_STANDARD 17)

# Use libgpiod v2 C API (stable across all v2 releases)
find_library(GPIOD_LIB gpiod REQUIRED)
find_path(GPIOD_INCLUDE gpiod.h REQUIRED)

add_executable(gpio_blink main.cpp)

target_include_directories(gpio_blink PRIVATE ${GPIOD_INCLUDE})
target_link_libraries(gpio_blink ${GPIOD_LIB})