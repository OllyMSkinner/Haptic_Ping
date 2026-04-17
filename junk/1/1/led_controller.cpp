#include "led_controller.h"

#include <gpiod.hpp>
#include <stdexcept>
#include <cstdio>

namespace {
gpiod::chip& getChip(int chipNumber)
{
    static std::unique_ptr<gpiod::chip> chip0;
    if (!chip0) {
        chip0 = std::make_unique<gpiod::chip>("gpiochip" + std::to_string(chipNumber));
    }
    return *chip0;
}
}

LEDController::LEDController(LEDControllerSettings settings)
    : SimpleLEDController(settings.greenGpio, settings.chipNumber)
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

LEDController::~LEDController()
{
    allOff();
}

void LEDController::set(bool on)
{
    std::printf("[led] set(%s)\n", on ? "true" : "false");

    greenActive_ = on;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

void LEDController::greenOn()
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

void LEDController::greenOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

void LEDController::allOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

void LEDController::flashGreen(int flashMs)
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(flashMs);
    updateOutputs();
}

void LEDController::service()
{
    if (greenActive_ &&
        greenOffTime_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() >= greenOffTime_) {
        greenActive_ = false;
        greenOffTime_ = std::chrono::steady_clock::time_point{};
        updateOutputs();
    }
}

void LEDController::updateOutputs()
{
    SimpleLEDController::set(greenActive_);
}