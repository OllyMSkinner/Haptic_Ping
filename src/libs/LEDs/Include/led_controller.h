/// @brief This header declares the LED controller settings and class.
/// @brief It manages green LED output, supports timed flashing, and exposes
/// @brief a small LED-facing interface through the inherited controller.
/// @brief
/// @brief SOLID principles:
/// @brief   S - This class is responsible for LED-specific behaviour only.
/// @brief   O - Client code can request LED actions through the public interface
/// @brief       without modifying the internal output logic.
/// @brief   L - The concrete LEDController builds on the simpler base controller so
/// @brief       it can be substituted wherever that LED abstraction is expected.
/// @brief   I - Client code uses a small LED-facing interface rather than dealing
/// @brief       with GPIO details or unrelated hardware operations.
/// @brief   D - Higher-level code can depend on the simpler SimpleLEDController
/// @brief       abstraction rather than this concrete GPIO-backed implementation.

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "ledcallback.hpp"
#include <chrono>

/// @brief Declares the LED controller settings, including the chip number and green LED GPIO pin.
struct LEDControllerSettings {
    int          chipNumber = 0;
    unsigned int greenGpio  = 25;
};

/// @brief Declares the LED controller class and its main functions for setting, switching, flashing, and updating the LED output.
class LEDController : public SimpleLEDController
{
public:
    LEDController(LEDControllerSettings settings = LEDControllerSettings());
    ~LEDController() override;

    void set(bool on) override;

    void greenOn();
    void greenOff();
    void allOff();

    void flashGreen(int flashMs);
    void service();

private:
    /// @brief Declares the internal update function and private members used to track the green LED state and flash timing.
    void updateOutputs();

    bool greenActive_ = false;
    std::chrono::steady_clock::time_point greenOffTime_{};
};

#endif
