#ifndef LEDCALLBACK_HPP
#define LEDCALLBACK_HPP

#include <gpiod.hpp>
#include <memory>

class SimpleLEDController
{
public:
    SimpleLEDController(int pinNo, int chipNo = 0);
    virtual ~SimpleLEDController();

    virtual void set(bool on);

    // Flash + service interface — overridden by LEDController.
    // No-ops here so the base can be used standalone and test stubs
    // only override what they need.
    virtual void flashGreen(int /*flashMs*/) {}
    virtual void service()                   {}

protected:
    // No-hardware constructor for unit-test stubs.
    SimpleLEDController() = default;

    int pin        = -1;
    int chipNumber = -1;

    std::shared_ptr<gpiod::chip>         chip;
    std::shared_ptr<gpiod::line_request> request;
};

#endif