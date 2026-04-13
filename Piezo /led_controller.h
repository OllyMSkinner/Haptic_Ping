#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <gpiod.hpp>
#include <memory>
#include <chrono>

struct LEDControllerSettings
{
    int chipNumber = 0;
    unsigned int redGpio = 17;
    unsigned int greenGpio = 27;
};

class LEDController
{
public:
    LEDController(LEDControllerSettings settings = LEDControllerSettings());
    ~LEDController();

    void redOn();
    void redOff();
    void greenOn();
    void greenOff();
    void allOff();

    void flashRed(int flashMs);
    void flashGreen(int flashMs);

    void service();

private:
    void updateOutputs();

    unsigned int redGpio_;
    unsigned int greenGpio_;

    std::shared_ptr<gpiod::chip> chip_;
    std::shared_ptr<gpiod::line_request> request_;

    bool redActive_ = false;
    bool greenActive_ = false;

    std::chrono::steady_clock::time_point redOffTime_{};
    std::chrono::steady_clock::time_point greenOffTime_{};
};

#endif
