    // This header declares the LED controller settings and class.
    // It manages the green LED output, supports timed flashing,
    // and updates the LED state through the inherited controller interface

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "ledcallback.hpp"
#include <chrono>

// Declares the LED controller settings, including the chip number and green LED GPIO pin.
struct LEDControllerSettings {
    int          chipNumber = 0;
    unsigned int greenGpio  = 25;
};

// Declares the LED controller class and its main functions for setting, switching, flashing, and updating the LED output.
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
    // Declares the internal update function and private members used to track the green LED state and flash timing.
    void updateOutputs();

    bool greenActive_ = false;
    std::chrono::steady_clock::time_point greenOffTime_{};
};

#endif
