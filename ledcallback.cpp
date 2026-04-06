#include "ledcallback.hpp"

#include <format>
#include <stdexcept>
#include <string>

LEDController::LEDController(int pinNo, int chipNo)
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

LEDController::~LEDController()
{
    if (request) {
        request->release();
    }

    if (chip) {
        chip->close();
    }
}

void LEDController::set(bool on)
{
    if (!request) {
        throw std::runtime_error("LED GPIO line is not initialized");
    }

    request->set_value(
        pin,
        on ? gpiod::line::value::ACTIVE
           : gpiod::line::value::INACTIVE);
}