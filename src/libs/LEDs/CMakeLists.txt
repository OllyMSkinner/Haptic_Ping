add_library(leds_lib STATIC
    ledcallback.cpp
    led_controller.cpp
)

target_include_directories(leds_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

target_link_libraries(leds_lib
    PUBLIC PkgConfig::GPIOD    # ledcallback.hpp and led_controller.h expose gpiod types
)
