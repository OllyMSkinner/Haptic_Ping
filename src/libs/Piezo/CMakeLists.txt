add_library(piezo_lib STATIC
    ads1115rpi.cpp
    event_detector.cpp
)

target_include_directories(piezo_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

target_link_libraries(piezo_lib
    PUBLIC  PkgConfig::GPIOD    # ads1115rpi.h and event_detector.h expose gpiod types
    PUBLIC  leds_lib            # event_detector.h includes led_controller.h
    PRIVATE imu_lib             # ads1115rpi.cpp uses i2c_mutex.hpp from IMU/Include
    PRIVATE pthread
)
