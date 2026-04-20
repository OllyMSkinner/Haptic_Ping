/** This file implements the LED controller behaviour.
* It keeps LED state changes, timed flashing, and hardware output updates
* brief inside one implementation unit. */

#include "led_controller.h"

#include <gpiod.hpp>
#include <stdexcept>
#include <cstdio>

namespace {
///Declares a helper function for accessing the GPIO chip instance.
gpiod::chip& getChip(int chipNumber)
{
    static std::unique_ptr<gpiod::chip> chip0;
    if (!chip0) {
        chip0 = std::make_unique<gpiod::chip>("gpiochip" + std::to_string(chipNumber));
    }
    return *chip0;
}
}

///Initialises the LED controller, clears the LED state, and updates the output.
LEDController::LEDController(LEDControllerSettings settings)
    : SimpleLEDController(settings.greenGpio, settings.chipNumber)
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

///Turns all LED output off when the controller is destroyed.
LEDController::~LEDController()
{
    allOff();
}

///Sets the green LED state directly and updates the output.
void LEDController::set(bool on)
{
    std::printf("[led] set(%s)\n", on ? "true" : "false");

    greenActive_ = on;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

///Turns the green LED on and clears any flash timing.
void LEDController::greenOn()
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

///Turns the green LED off and clears any flash timing.
void LEDController::greenOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

///Turns all LED output off and clears any flash timing.
void LEDController::allOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}
///Turns the green LED on for a set time and stores when it should switch off.
void LEDController::flashGreen(int flashMs)
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(flashMs);
    updateOutputs();
}

///Checks whether the flash time has ended and switches the LED off if needed.
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

///Updates the physical LED output to match the current stored state.
void LEDController::updateOutputs()
{
    SimpleLEDController::set(greenActive_);
}
