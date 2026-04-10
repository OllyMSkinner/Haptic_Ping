cmake_minimum_required(VERSION 3.16)
project(accel_pwm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── IMUPublisher static library ───────────────────────────────────────────────
add_library(imu_publisher STATIC
    IMU_processing/IMUPublisher/IMUPublisher.cpp
    IMU_processing/imudriver/icm20948_i2c.cpp
    IMU_processing/imu_settings/icm20948_utils.cpp
)

target_include_directories(imu_publisher PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/IMU_processing/IMUPublisher
    ${CMAKE_SOURCE_DIR}/IMU_processing/imudriver
    ${CMAKE_SOURCE_DIR}/IMU_processing/imu_settings
)

find_package(Threads REQUIRED)
target_link_libraries(imu_publisher PUBLIC
    Threads::Threads
    gpiod        # libgpiod for GPIO interrupt support
    mraa         # libmraa for I2C communication
    yaml-cpp     # libyaml-cpp for settings parsing
)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(accel_pwm
    accel_pwm.cpp
)

target_include_directories(accel_pwm PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/IMU_processing/IMUPublisher
    ${CMAKE_SOURCE_DIR}/IMU_processing/imudriver
    ${CMAKE_SOURCE_DIR}/IMU_processing/imu_settings
)

target_link_libraries(accel_pwm PRIVATE
    imu_publisher
)