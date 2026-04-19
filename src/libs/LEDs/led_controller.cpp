/// @brief This file implements the LED controller class for the piezo events.
/// @brief It keeps LED-specific behaviour in one place by managing the green LED
/// @brief state, supporting timed flashing, and updating the output through the
/// @brief base controller so the rest of the application does not deal directly
/// @brief with GPIO output details or timing state.
/// @brief
/// @brief SOLID principles:
/// @brief   S - This file is focused on LED output behaviour only: storing LED state,
/// @brief       applying timed flashing, and updating the hardware-facing base controller.
/// @brief   O - Client code can request LED actions through the public interface without
/// @brief       modifying the internal output logic.
/// @brief   L - The concrete LEDController builds on the simpler base controller so client
/// @brief       code can depend on a narrower LED abstraction rather than GPIO details.
/// @brief   I - Client code can use the small LED-facing interface instead of dealing with
/// @brief       GPIO details or unrelated hardware operations.
/// @brief   D - Higher-level code can depend on the simpler SimpleLEDController abstraction
/// @brief       rather than this concrete GPIO-backed implementation.

#include "led_controller.h"

#include <gpiod.hpp>
#include <stdexcept>
#include <cstdio>

namespace {
/// @brief Declares a helper function for accessing the GPIO chip instance.
gpiod::chip& getChip(int chipNumber)
{
    static std::unique_ptr<gpiod::chip> chip0;
    if (!chip0) {
        chip0 = std::make_unique<gpiod::chip>("gpiochip" + std::to_string(chipNumber));
    }
    return *chip0;
}
}

/// @brief Initialises the LED controller, clears the LED state, and updates the output.
LEDController::LEDController(LEDControllerSettings settings)
    : SimpleLEDController(settings.greenGpio, settings.chipNumber)
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

/// @brief Turns all LED output off when the controller is destroyed.
LEDController::~LEDController()
{
    allOff();
}

/// @brief Sets the green LED state directly and updates the output.
void LEDController::set(bool on)
{
    std::printf("[led] set(%s)\n", on ? "true" : "false");

    greenActive_ = on;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

/// @brief Turns the green LED on and clears any flash timing.
void LEDController::greenOn()
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

/// @brief Turns the green LED off and clears any flash timing.
void LEDController::greenOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

/// @brief Turns all LED output off and clears any flash timing.
void LEDController::allOff()
{
    greenActive_ = false;
    greenOffTime_ = std::chrono::steady_clock::time_point{};
    updateOutputs();
}

/// @brief Turns the green LED on for a set time and stores when it should switch off.
void LEDController::flashGreen(int flashMs)
{
    greenActive_ = true;
    greenOffTime_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(flashMs);
    updateOutputs();
}

/// @brief Checks whether the flash time has ended and switches the LED off if needed.
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
