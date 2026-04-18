add_library(imu_lib STATIC
    Icm20948driver.cpp
    imureader.cpp
)

target_include_directories(imu_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)

target_link_libraries(imu_lib
    PUBLIC  PkgConfig::GPIOD      # imureader.hpp exposes gpiod types
    PRIVATE yaml-cpp pthread      # implementation details only
)
