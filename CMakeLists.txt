cmake_minimum_required(VERSION 3.10)
project(rtes_main_project)

set(CMAKE_CXX_STANDARD 17)

# Position Detector
add_library(position_detector
    Starting_position/positiondetector.cpp
)

target_include_directories(position_detector PUBLIC
    Starting_position
)

# Executable
add_executable(rtes_main main.cpp)

target_link_libraries(rtes_main
    position_detector
    pthread
)