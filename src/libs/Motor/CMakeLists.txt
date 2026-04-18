add_library(motor_lib STATIC
    swingfeedback.cpp
)

target_include_directories(motor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

target_link_libraries(motor_lib
    PRIVATE pthread    # implementation detail — threads used internally only
)
