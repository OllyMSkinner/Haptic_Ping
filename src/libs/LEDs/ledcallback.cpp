    /// @brief This file implements the simple LED controller class.
    /// @brief It sets up the GPIO line for LED output, releases the GPIO resources
    /// @brief when finished, and allows the LED state to be switched on or off.


#include "ledcallback.hpp"

#include <stdexcept>
#include <string>

/// @brief Initialises the LED controller by setting up the GPIO chip, output line, and line request for the selected pin.
SimpleLEDController::SimpleLEDController(int pinNo, int chipNo)
    : pin(pinNo), chipNumber(chipNo)
{
    const std::string chipPath =
        "/dev/gpiochip" + std::to_string(chipNumber);

    const std::string consumerName =
        "ledcontroller_" + std::to_string(chipNumber) + "_" + std::to_string(pin);

    chip = std::make_shared<gpiod::chip>(chipPath);

    gpiod::line_config line_cfg;
    line_cfg.add_line_settings(
        pin,
        gpiod::line_settings()
            .set_direction(gpiod::line::direction::OUTPUT)
            .set_output_value(gpiod::line::value::INACTIVE));

    auto builder = chip->prepare_request();
    builder.set_consumer(consumerName);
    builder.set_line_config(line_cfg);

    request = std::make_shared<gpiod::line_request>(builder.do_request());
}

/// @brief Releases the GPIO resources and makes sure the LED is turned off before closing.
SimpleLEDController::~SimpleLEDController()
{
    try {
        if (request)
            request->set_value(pin, gpiod::line::value::INACTIVE);
    } catch (...) {}

    if (request) {
        request->release();
    }

    if (chip) {
        chip->close();
    }
}

/// @brief Sets the LED output state to on or off.
void SimpleLEDController::set(bool on)
{
    if (!request) {
        throw std::runtime_error("LED GPIO line is not initialized");
    }

    request->set_value(
        pin,
        on ? gpiod::line::value::ACTIVE
           : gpiod::line::value::INACTIVE);
}
