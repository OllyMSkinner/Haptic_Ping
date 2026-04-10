#CMake was not provided when library was cloned
#Quick file creating a STATIC library from it
add_library(IMU_library STATIC icm20948driver.cpp)

#Uses links the include file for access to headers and definitions
target_include_directories(IMU_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)