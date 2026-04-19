// This file implements the LED controller class for the piezo events.
// It keeps LED-specific behaviour in one place by managing the green LED
// state, supporting timed flashing, and updating the output through the
// base controller so the rest of the application does not deal directly
// with GPIO output details or timing state.
//
// SOLID principles:
//   S - This file is focused on LED output behaviour only: storing LED state,
//       applying timed flashing, and updating the hardware-facing base controller.
//   O - Client code can request LED actions through the public interface without
//       modifying the internal output logic.
//   L - The concrete LEDController builds on the simpler base controller so client
//       code can depend on a narrower LED abstraction rather than GPIO details.
//   I - Client code can use the small LED-facing interface instead of dealing with
//       GPIO details or unrelated hardware operations.
//   D - Higher-level code can depend on the simpler SimpleLEDController abstraction
//       rather than this concrete GPIO-backed implementation.

#include "led_controller.h"

#include <gpiod.hpp>
#include <stdexcept>
#include <cstdio>

namespace {
// Declares a helper function for accessing the GPIO chip instance.
gpiod::chip& getChip(int chipNumber)
{
    static std::unique_ptr<gpiod::chip> chip0;
    if (!chip0) {
        chip0 = std::make_unique<gpiod::chip>("gpiochip" + std::to_string(chipNumber));
    }
    return *chip0;
}
}

// Initialises the LED controller, clears the LED state, and updates the output.
LEDController::LEDController(LEDControllerSettings settings)
    : SimpleLEDController(settings.greenGpio, settings.chipNumber)
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

// Turns all LED output off when the controller is destroyed.
LEDController::~LEDController()
{
    allOff();
}

// Sets the green LED state directly and updates the output.
void LEDController::set(bool on)
{
    std::printf("[led] set(%s)\n", on ? "true" : "false");

    greenActive_ = on;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

// Turns the green LED on and clears any flash timing.
void LEDController::greenOn()
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

// Turns the green LED off and clears any flash timing.
void LEDController::greenOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

// Turns all LED output off and clears any flash timing.
void LEDController::allOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

// Turns the green LED on for a set time and stores when it should switch off.
void LEDController::flashGreen(int flashMs)
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(flashMs);
    updateOutputs();
}

// Checks whether the flash time has ended and switches the LED off if needed.
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

// Updates the physical LED output to match the current stored state.
void LEDController::updateOutputs()
{
    SimpleLEDController::set(greenActive_);
}
