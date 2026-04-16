#include "SwingDetector.hpp"

SwingDetector::SwingDetector()
    : last_level_(Level::NONE)
{}

void SwingDetector::setCallback(EventCallback cb)
{
    callback_ = std::move(cb);
}

void SwingDetector::reset()
{
    last_level_ = Level::NONE;
}

void SwingDetector::detect(float accel_mag)
{
    Level current = classify(accel_mag);
    if (current > last_level_) {
        last_level_ = current;
        if (callback_) callback_(label(current));
    }
}

SwingDetector::Level SwingDetector::classify(float a)
{
    if (a >= 20.f)  return Level::HIGH;
    if (a >= 10.f)  return Level::MEDIUM;
    if (a >=  5.f)  return Level::LOW;
    return Level::NONE;
}

const char* SwingDetector::label(Level level)
{
    switch (level) {
    case Level::HIGH:   return "Highest Duty Cycle";
    case Level::MEDIUM: return "Medium Duty Cycle";
    case Level::LOW:    return "Low Duty Cycle";
    default:            return "No Duty Cycle";
    }
}
