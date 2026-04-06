cmake_minimum_required(VERSION 3.10)
project(PI5Receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

add_executable(rtes_main
    main.cpp
    positiondetector.cpp
    ledcallback.cpp
    samplereceiver.cpp
)

target_include_directories(rtes_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GPIOD_INCLUDE_DIRS}
)

target_link_directories(rtes_main PRIVATE
    ${GPIOD_LIBRARY_DIRS}
)

target_link_libraries(rtes_main PRIVATE
    ${GPIOD_LIBRARIES}
    pthread)
