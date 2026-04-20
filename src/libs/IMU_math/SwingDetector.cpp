#include "SwingDetector.hpp"

/// Start with no previously emitted level.
SwingDetector::SwingDetector()
    : last_level_(Level::NONE)
{}

/// Register the callback fired on level changes.
void SwingDetector::setCallback(EventCallback cb)
{
    callback_ = std::move(cb);
}

/// Clear cached state so the next detect() call always emits.
void SwingDetector::reset()
{
    last_level_ = Level::NONE;
}

void SwingDetector::detect(float accel_mag)
{
    /// Classify the current sample into a duty-cycle band.
    Level current = classify(accel_mag);

    /// Only emit when the band changes.
    if (current != last_level_) {
        last_level_ = current;
        if (callback_) callback_(label(current));
    }
}

SwingDetector::Level SwingDetector::classify(float a)
{
    /// Map acceleration magnitude to a discrete level.
    if (a >= 25.f)  return Level::HIGH;
    if (a >= 15.f)  return Level::MEDIUM;
    if (a >= 10.f)  return Level::LOW;
    return Level::NONE;
}

const char* SwingDetector::label(Level level)
{
    /// Convert enum level to the external string label.
    switch (level) {
    case Level::HIGH:   return "Highest Duty Cycle";
    case Level::MEDIUM: return "Medium Duty Cycle";
    case Level::LOW:    return "Low Duty Cycle";
    default:            return "No Duty Cycle";
    }
}
