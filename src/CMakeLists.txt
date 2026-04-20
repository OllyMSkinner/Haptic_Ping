cmake_minimum_required(VERSION 3.10)
project(PI5Receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

add_library(IMUMATHS STATIC
    libs/IMU_math/positiondetector.cpp
    libs/IMU_math/SwingCalibrator.cpp
    libs/IMU_math/SwingDetector.cpp
    libs/IMU_math/SwingProcessor.cpp
)

add_library(LEDS STATIC
    libs/LEDs/ledcallback.cpp
    libs/LEDs/led_controller.cpp
)

add_library(MOTOR STATIC
    libs/Motor/swingfeedback.cpp
)

add_library(PIEZO STATIC
    libs/Piezo/event_detector.cpp
)

foreach(LIB IMUMATHS LEDS MOTOR PIEZO)
    target_include_directories(${LIB} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/IMU/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/IMU_math/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/LEDs/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/Motor/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/Piezo/Include
        ${GPIOD_INCLUDE_DIRS}
    )
    target_link_directories(${LIB} PUBLIC ${GPIOD_LIBRARY_DIRS})
    target_link_libraries(${LIB} PUBLIC ${GPIOD_LIBRARIES} gpiod pthread)
endforeach()

add_executable(rtes_main
    main.cpp
    libs/IMU/imureader.cpp
    libs/IMU/Icm20948driver.cpp
    libs/Piezo/ads1115rpi.cpp
)

target_include_directories(rtes_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/IMU/Include
    ${GPIOD_INCLUDE_DIRS}
)

target_link_directories(rtes_main PRIVATE ${GPIOD_LIBRARY_DIRS})

target_link_libraries(rtes_main PRIVATE
    IMUMATHS
    LEDS
    MOTOR
    PIEZO
    ${GPIOD_LIBRARIES}
    gpiod
    yaml-cpp
    pthread
)

option(BUILD_TESTS "Build GTest targets" OFF)

if(BUILD_TESTS)
    add_subdirectory(../test test)
endif()