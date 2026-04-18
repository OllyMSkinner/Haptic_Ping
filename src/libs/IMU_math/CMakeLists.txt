add_library(imu_math_lib STATIC
    positiondetector.cpp
    SwingCalibrator.cpp
    SwingDetector.cpp
    SwingProcessor.cpp
)

target_include_directories(imu_math_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include
)
