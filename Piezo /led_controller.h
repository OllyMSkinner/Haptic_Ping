#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <gpiod.hpp>
#include <memory>
#include <chrono>

struct LEDControllerSettings
{
    int chipNumber = 0;
    unsigned int greenGpio = 25;
};

class LEDController
{
public:
    LEDController(LEDControllerSettings settings = LEDControllerSettings());
    ~LEDController();

    void greenOn();
    void greenOff();
    void allOff();

    void flashGreen(int flashMs);

    void service();

private:
    void updateOutputs();

    unsigned int greenGpio_;

    std::shared_ptr<gpiod::chip> chip_;
    std::shared_ptr<gpiod::line_request> request_;

    bool greenActive_ = false;

    std::chrono::steady_clock::time_point greenOffTime_{};
};

#endif
