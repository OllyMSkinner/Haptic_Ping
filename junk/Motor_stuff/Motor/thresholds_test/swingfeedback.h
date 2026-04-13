#pragma once

#include "SwingDetector.hpp"
#include "rpi_pwm.h"

#include <string_view>

// Receives swing level strings from SwingDetector and drives the PWM motor
// accordingly. Wire up via SwingDetector::setCallback.

class SwingFeedback {
public:
    explicit SwingFeedback(RPI_PWM& pwm);

    // Pass this to SwingDetector::setCallback
    void onLevel(const char* level);

private:
    RPI_PWM& pwm_;

    static float levelToDutyCycle(std::string_view level);
}; 