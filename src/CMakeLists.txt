cmake_minimum_required(VERSION 3.10)
project(PI5Receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)

add_subdirectory(libs/LEDs)
add_subdirectory(libs/IMU)
add_subdirectory(libs/IMU_math)
add_subdirectory(libs/Motor)
add_subdirectory(libs/Piezo)

add_executable(rtes_main main.cpp)

target_link_libraries(rtes_main PRIVATE
    imu_lib
    imu_math_lib
    leds_lib
    motor_lib
    piezo_lib
    yaml-cpp
    pthread
)

option(BUILD_TESTS "Build GTest targets" OFF)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()

    macro(add_gtest TARGET)
        target_link_libraries(${TARGET} PRIVATE
            GTest::GTest
            GTest::Main
            pthread
        )
        target_compile_definitions(${TARGET} PRIVATE UNIT_TEST)
        add_test(NAME ${TARGET} COMMAND ${TARGET})
    endmacro()

    add_executable(SwingDetector_GTest      ../test/SwingDetector_GTest.cpp)
    target_link_libraries(SwingDetector_GTest PRIVATE imu_math_lib)
    add_gtest(SwingDetector_GTest)

    add_executable(SwingCalibrator_GTest    ../test/SwingCalibrator_GTest.cpp)
    target_link_libraries(SwingCalibrator_GTest PRIVATE imu_math_lib)
    add_gtest(SwingCalibrator_GTest)

    add_executable(PiezoEventDetector_GTest ../test/PiezoEventDetector_GTest.cpp)
    target_link_libraries(PiezoEventDetector_GTest PRIVATE piezo_lib leds_lib)
    add_gtest(PiezoEventDetector_GTest)

    add_executable(SwingFeedback_GTest      ../test/SwingFeedback_GTest.cpp)
    target_link_libraries(SwingFeedback_GTest PRIVATE motor_lib PkgConfig::GPIOD gpiod)
    add_gtest(SwingFeedback_GTest)

    add_executable(PositionDetector_GTest   ../test/PositionDetector_GTest.cpp)
    target_link_libraries(PositionDetector_GTest PRIVATE imu_math_lib)
    add_gtest(PositionDetector_GTest)

    add_executable(SwingProcessor_GTest     ../test/SwingProcessor_GTest.cpp)
    target_link_libraries(SwingProcessor_GTest PRIVATE imu_math_lib)
    add_gtest(SwingProcessor_GTest)
endif()