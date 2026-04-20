add_library(motor_lib STATIC
    swingfeedback.cpp
)

target_include_directories(motor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

target_link_libraries(motor_lib
    PUBLIC  PkgConfig::GPIOD    # rpi_pwm.h exposes gpiod types
    PRIVATE pthread
)
