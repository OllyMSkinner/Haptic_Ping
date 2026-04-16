#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "ledcallback.hpp"
#include <chrono>

struct LEDControllerSettings {
    int          chipNumber = 0;
    unsigned int greenGpio  = 25;
};

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
    void updateOutputs();

    bool greenActive_ = false;
    std::chrono::steady_clock::time_point greenOffTime_{};
};

#endif
