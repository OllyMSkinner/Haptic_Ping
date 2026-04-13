#include "led_controller.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

LEDController::LEDController(int chipNumber, unsigned int redGpio, unsigned int greenGpio)
    : redGpio_(redGpio),
      greenGpio_(greenGpio)
{
    const std::string chipPath = "/dev/gpiochip" + std::to_string(chipNumber);

    chip_ = std::make_shared<gpiod::chip>(chipPath);

    gpiod::line_config line_cfg;

    line_cfg.add_line_settings(
        redGpio_,
        gpiod::line_settings()
            .set_direction(gpiod::line::direction::OUTPUT)
            .set_output_value(gpiod::line::value::INACTIVE));

    line_cfg.add_line_settings(
        greenGpio_,
        gpiod::line_settings()
            .set_direction(gpiod::line::direction::OUTPUT)
            .set_output_value(gpiod::line::value::INACTIVE));

    auto builder = chip_->prepare_request();
    builder.set_consumer("led_controller");
    builder.set_line_config(line_cfg);

    request_ = std::make_shared<gpiod::line_request>(builder.do_request());
}

LEDController::~LEDController()
{
    try
    {
        allOff();
        if (request_)
            request_->release();
        if (chip_)
            chip_->close();
    }
    catch (...)
    {
    }
}

void LEDController::redOn()
{
    request_->set_value(redGpio_, gpiod::line::value::ACTIVE);
}

void LEDController::redOff()
{
    request_->set_value(redGpio_, gpiod::line::value::INACTIVE);
}

void LEDController::greenOn()
{
    request_->set_value(greenGpio_, gpiod::line::value::ACTIVE);
}

void LEDController::greenOff()
{
    request_->set_value(greenGpio_, gpiod::line::value::INACTIVE);
}

void LEDController::allOff()
{
    redOff();
    greenOff();
}

void LEDController::flashRed(int flashMs)
{
    redOn();
    std::this_thread::sleep_for(std::chrono::milliseconds(flashMs));
    redOff();
}

void LEDController::flashGreen(int flashMs)
{
    greenOn();
    std::this_thread::sleep_for(std::chrono::milliseconds(flashMs));
    greenOff();
}
