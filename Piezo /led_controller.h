#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <gpiod.hpp>
#include <memory>

class LEDController
{
public:
    LEDController(int chipNumber = 0,
                  unsigned int redGpio = 17,
                  unsigned int greenGpio = 27);
    ~LEDController();

    void redOn();
    void redOff();
    void greenOn();
    void greenOff();
    void allOff();

    void flashRed(int flashMs);
    void flashGreen(int flashMs);

private:
    unsigned int redGpio_;
    unsigned int greenGpio_;

    std::shared_ptr<gpiod::chip> chip_;
    std::shared_ptr<gpiod::line_request> request_;
};

#endif
