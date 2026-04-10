cmake_minimum_required(VERSION 3.10)
project(March23Receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
pkg_check_modules(GPIODCXX REQUIRED libgpiodcxx)
pkg_check_modules(MRAA REQUIRED mraa)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

add_executable(rtes_main
    main.cpp
    positiondetector.cpp
    ledcallback.cpp
    imureader.cpp
    Icm20948driver.cpp
)

target_include_directories(rtes_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GPIOD_INCLUDE_DIRS}
    ${GPIODCXX_INCLUDE_DIRS}
    ${MRAA_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

target_link_directories(rtes_main PRIVATE
    ${GPIOD_LIBRARY_DIRS}
    ${GPIODCXX_LIBRARY_DIRS}
    ${MRAA_LIBRARY_DIRS}
    ${YAMLCPP_LIBRARY_DIRS}
)

target_link_libraries(rtes_main PRIVATE
    ${GPIOD_LIBRARIES}
    ${GPIODCXX_LIBRARIES}
    ${MRAA_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    pthread
)