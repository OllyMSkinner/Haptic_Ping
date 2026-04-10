cmake_minimum_required(VERSION 3.13)
project(imu_coords LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Sources ----
set(SOURCES
    main.cpp
    icm20948_i2c.cpp
    icm20948_utils.cpp
)

# PositionDetector source file:
# Prefer correct spelling "positiondetector.cpp", but fall back to the typo "postiondetector.cpp"
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/positiondetector.cpp")
    list(APPEND SOURCES positiondetector.cpp)
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Positiondetector.cpp")
    list(APPEND SOURCES Positiondetector.cpp)
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/postiondetector.cpp")
    message(WARNING "Using 'postiondetector.cpp' (typo filename). Consider renaming to 'positiondetector.cpp'.")
    list(APPEND SOURCES postiondetector.cpp)
else()
    message(FATAL_ERROR "Could not find position detector .cpp file. Expected positiondetector.cpp or postiondetector.cpp")
endif()

add_executable(imu_coords ${SOURCES})

# Headers are in the repo root (and possibly Nat_Positiondetector if you include from there)
target_include_directories(imu_coords PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Nat_Positiondetector
)

# Needed for std::thread on Linux
find_package(Threads REQUIRED)
target_link_libraries(imu_coords PRIVATE Threads::Threads)

# Helpful warnings
target_compile_options(imu_coords PRIVATE -Wall -Wextra -Wpedantic)
