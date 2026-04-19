#pragma once

#include <functional>

// Maps linear acceleration magnitude to duty-cycle bands.
// Only emits when the band changes.
// a >= 25 → "Highest Duty Cycle" 
// 15 <= a < 25 → "Medium Duty Cycle" 
// 5 <= a < 15 → "Low Duty Cycle" 
// a < 5 → "No Duty Cycle"

class SwingDetector {
public:
    using EventCallback = std::function<void(const char* level)>;

    SwingDetector();

    void setCallback(EventCallback cb);

    // Process one accel-magnitude sample and emit on band change.
    void detect(float accel_mag);

    // Clear cached state so the next detect() emits unconditionally.
    void reset();

private:
    enum class Level { NONE, LOW, MEDIUM, HIGH };

    // Convert accel magnitude to a duty-cycle band.
    static Level classify(float accel_mag);

    // Convert enum value to callback label.
    static const char* label(Level level);

    // Last emitted band.
    Level last_level_;

    EventCallback callback_;
};
