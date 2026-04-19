    /// @brief This header declares a simple LED controller base class. This is to control the LED associated with the IMU swing position.

    /// @brief Design intention:
    /// @brief - Single Responsibility:
    /// @brief   This class is responsible only for wrapping the basic GPIO-backed LED
    /// @brief   control mechanism and exposing a small interface for derived LED classes.
    /// @brief - Open/Closed:
    /// @brief   Derived classes can extend behaviour such as flashing or timed service
    /// @brief   updates without changing the base class interface.
    /// @brief - Liskov Substitution:
    /// @brief   Client code can use this base class interface and substitute any derived
    /// @brief   LED controller that implements the same contract.
    /// @brief - Interface Segregation / Dependency Inversion:
    /// @brief   Other parts of the system can depend on this small LED abstraction rather
    /// @brief   than on a larger concrete GPIO implementation.

    /// @brief It provides:
    /// @brief - basic LED control through set(bool),
    /// @brief - optional flash/service hooks for derived classes,
    /// @brief - storage for the GPIO resources needed to control the LED.


#ifndef LEDCALLBACK_HPP
#define LEDCALLBACK_HPP

#include <gpiod.hpp>
#include <memory>

/// @brief SimpleLEDController acts as a narrow abstraction for LED behaviour.
/// @brief Higher-level logic can depend on this interface rather than depending
/// @brief directly on a concrete GPIO-specific LED implementation.
class SimpleLEDController
{
public:
    /// @brief Construct the controller with the GPIO pin number and optional chip number.
    /// @brief The dependency on the physical GPIO configuration is contained here so that
    /// @brief client code does not need to manage those hardware details itself.
    SimpleLEDController(int pinNo, int chipNo = 0);

    /// @brief Virtual destructor ensures correct cleanup through the base-class interface,
    /// @brief supporting safe substitution of derived LED controller types.
    virtual ~SimpleLEDController();

    /// @brief Minimal public LED interface: callers express intent (on/off) while the
    /// @brief hardware-specific details remain encapsulated inside the implementation.
    virtual void set(bool on);

    /// @brief Optional extension point for derived classes that support timed flashing.
    /// @brief Default implementation does nothing so derived classes are not forced to
    /// @brief implement behaviour they do not need.
    virtual void flashGreen(int /*flashMs*/) {}

    /// @brief Optional extension point for derived classes that need periodic servicing,
    /// @brief such as time-based LED state updates.
    virtual void service()                   {}

protected:
    /// @brief Protected default constructor allows derived classes controlled access
    /// @brief without exposing incomplete construction to general client code.
    SimpleLEDController() = default;

    /// @brief Internal GPIO configuration state is protected so it is available to
    /// @brief derived implementations, while still hidden from external users.
    int pin        = -1;
    int chipNumber = -1;

    /// @brief Shared ownership is used for the GPIO chip/request resources so that the
    /// @brief resource lifetime is managed safely without manual memory handling.
    std::shared_ptr<gpiod::chip>         chip;
    std::shared_ptr<gpiod::line_request> request;
};

#endif
