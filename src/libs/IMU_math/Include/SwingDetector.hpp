#pragma once

// SOLID principles:
// S - SwingDetector categorised the swing speed intro 3 levels, and creates a callback when 
// the level changes. This actually does 2 things but it is very small so we decided to wrap 
// them up with each other.
// I - The calculation of the level classification in made private as other caller doesn't need to know.
// D - The std::function is an abstraction which allows for any inputs to the function.

#include <functional>

// Classifies linear acceleration magnitude into duty cycle levels.
// Fires the registered callback only when the level changes.
//
// Thresholds (m/s²):
//   a >= 25          → "Highest Duty Cycle"
//   15 <= a < 25     → "Medium Duty Cycle"
//    5 <= a < 15     → "Low Duty Cycle"
//         a <  5     → "No Duty Cycle"

class SwingDetector {
public:
    using EventCallback = std::function<void(const char* level)>;

    SwingDetector();

    void setCallback(EventCallback cb);

    // Feed one accel magnitude sample (m/s²). Fires callback on level change.
    void detect(float accel_mag);

    // Reset last known level so the next detect() always fires a callback.
    void reset();

private:
    enum class Level { NONE, LOW, MEDIUM, HIGH };

    // Convert accel magnitude to a duty-cycle band.
    static Level    classify(float accel_mag);

    // Convert enum value to callback label.
    static const char* label(Level level);

    // Last emitted band.
    Level         last_level_;

    EventCallback callback_;
};
