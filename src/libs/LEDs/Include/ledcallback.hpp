/*
    This header declares a simple LED controller class.
    It provides basic LED control functions, optional flash and service
    functions, and stores the GPIO resources needed to control the LED.
*/

#ifndef LEDCALLBACK_HPP
#define LEDCALLBACK_HPP

#include <gpiod.hpp>
#include <memory>

// Declares the LED controller class and its main functions for setting the LED state, flashing, and servicing updates.
class SimpleLEDController
{
public:
    SimpleLEDController(int pinNo, int chipNo = 0);
    virtual ~SimpleLEDController();

    virtual void set(bool on);
    virtual void flashGreen(int /*flashMs*/) {}
    virtual void service()                   {}

protected:
    // Declares the default constructor and the private members used to store GPIO pin, chip, and request data. 
    SimpleLEDController() = default;

    int pin        = -1;
    int chipNumber = -1;

    std::shared_ptr<gpiod::chip>         chip;
    std::shared_ptr<gpiod::line_request> request;
};

#endif
