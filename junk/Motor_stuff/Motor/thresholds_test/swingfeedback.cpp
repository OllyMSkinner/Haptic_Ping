#include "swingfeedback.h"

#include <iostream>

SwingFeedback::SwingFeedback(RPI_PWM& pwm)
    : pwm_(pwm)
{}

void SwingFeedback::onLevel(const char* level)
{
    float duty = levelToDutyCycle(level);
    pwm_.setDutyCycle(duty);
    std::cout << "Swing level: " << level << " → duty cycle: " << duty << "%\n";
}

float SwingFeedback::levelToDutyCycle(std::string_view level)
{
    if (level == "Highest Duty Cycle") return 100.0f;
    if (level == "Medium Duty Cycle")  return 66.0f;
    if (level == "Low Duty Cycle")     return 33.0f;
    return 0.0f; // "No Duty Cycle"
}