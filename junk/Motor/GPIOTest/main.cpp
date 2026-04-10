#include <gpiod.h>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    const unsigned int line_num = 18;

    // Build line request using v2 C API
    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(line_cfg, &line_num, 1, settings);

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "gpio_blink");

    gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    gpiod_line_request* request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    // Cleanup config objects (request keeps its own reference)
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    // Set HIGH
    std::cout << "GPIO 18 -> HIGH" << std::endl;
    gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_ACTIVE);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Set LOW
    std::cout << "GPIO 18 -> LOW" << std::endl;
    gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Set HIGH again
    std::cout << "GPIO 18 -> HIGH" << std::endl;
    gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_ACTIVE);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Set LOW at the end
    std::cout << "GPIO 18 -> LOW" << std::endl;
    gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE);

    // Cleanup
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);

    std::cout << "Done." << std::endl;
    return 0;
}