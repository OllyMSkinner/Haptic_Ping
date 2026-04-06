#ifndef LEDCALLBACK_HPP
#define LEDCALLBACK_HPP

#include <gpiod.hpp>
#include <memory>

class LEDController
{
public:
    LEDController(int pinNo, int chipNo = 0);
    ~LEDController();

    void set(bool on);

private:
    int pin;
    int chipNumber;

    std::shared_ptr<gpiod::chip> chip;
    std::shared_ptr<gpiod::line_request> request;
};
#endif