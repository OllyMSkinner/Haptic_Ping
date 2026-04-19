/*
    This header declares a simple LED controller base class. This is to control the LED associated with the IMU swing position.

    Design intention:
    - Single Responsibility:
      This class is responsible only for wrapping the basic GPIO-backed LED
      control mechanism and exposing a small interface for derived LED classes.
    - Open/Closed:
      Derived classes can extend behaviour such as flashing or timed service
      updates without changing the base class interface.
    - Liskov Substitution:
      Client code can use this base class interface and substitute any derived
      LED controller that implements the same contract.
    - Interface Segregation / Dependency Inversion:
      Other parts of the system can depend on this small LED abstraction rather
      than on a larger concrete GPIO implementation.

    It provides:
    - basic LED control through set(bool),
    - optional flash/service hooks for derived classes,
    - storage for the GPIO resources needed to control the LED.
*/

#ifndef LEDCALLBACK_HPP
#define LEDCALLBACK_HPP

#include <gpiod.hpp>
#include <memory>

// SimpleLEDController acts as a narrow abstraction for LED behaviour.
// Higher-level logic can depend on this interface rather than depending
// directly on a concrete GPIO-specific LED implementation.
class SimpleLEDController
{
public:
    // Construct the controller with the GPIO pin number and optional chip number.
    // The dependency on the physical GPIO configuration is contained here so that
    // client code does not need to manage those hardware details itself.
    SimpleLEDController(int pinNo, int chipNo = 0);

    // Virtual destructor ensures correct cleanup through the base-class interface,
    // supporting safe substitution of derived LED controller types.
    virtual ~SimpleLEDController();

    // Minimal public LED interface: callers express intent (on/off) while the
    // hardware-specific details remain encapsulated inside the implementation.
    virtual void set(bool on);

    // Optional extension point for derived classes that support timed flashing.
    // Default implementation does nothing so derived classes are not forced to
    // implement behaviour they do not need.
    virtual void flashGreen(int /*flashMs*/) {}

    // Optional extension point for derived classes that need periodic servicing,
    // such as time-based LED state updates.
    virtual void service()                   {}

protected:
    // Protected default constructor allows derived classes controlled access
    // without exposing incomplete construction to general client code.
    SimpleLEDController() = default;

    // Internal GPIO configuration state is protected so it is available to
    // derived implementations, while still hidden from external users.
    int pin        = -1;
    int chipNumber = -1;

    // Shared ownership is used for the GPIO chip/request resources so that the
    // resource lifetime is managed safely without manual memory handling.
    std::shared_ptr<gpiod::chip>         chip;
    std::shared_ptr<gpiod::line_request> request;
};

#endif
