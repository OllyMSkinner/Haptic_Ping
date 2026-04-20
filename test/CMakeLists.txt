find_package(GTest REQUIRED)

add_executable(run_tests
    PiezoEventDetector_GTest.cpp
    PositionDetector_GTest.cpp
    SwingCalibrator_GTest.cpp
    SwingDetector_GTest.cpp
    SwingFeedback_GTest.cpp
    SwingProcessor_GTest.cpp
)

enable_testing()

target_link_libraries(run_tests PRIVATE
    IMUMATHS
    LEDS
    MOTOR
    PIEZO
    ${GPIOD_LIBRARIES}
    gtest
    gtest_main
    pthread
)

add_test(NAME Test_all COMMAND ${CMAKE_CURRENT_BINARY_DIR}/run_tests)

set_tests_properties(Test_all PROPERTIES
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

target_compile_definitions(run_tests PRIVATE UNIT_TEST)