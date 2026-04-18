#pragma once

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

    static Level    classify(float accel_mag);
    static const char* label(Level level);

    Level         last_level_;
    EventCallback callback_;
};
