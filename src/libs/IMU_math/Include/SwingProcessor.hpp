#pragma once

// SOLID Principles:
// D - MagnitudeCallback and PositionStateCallback are std::function output abstractions.

#include "SwingCalibrator.hpp"
#include "positiondetector.hpp"

#include <chrono>
#include <functional>

static constexpr unsigned I2C_BUS        = 1;
static constexpr int      CALIB_SAMPLES  = 50;   // ~0.5s at 100Hz — startup bias
static constexpr int      RECAL_SAMPLES  = 20;   // ~0.2s recal on position confirm
static constexpr float    ACCEL_DEADBAND = 1.0f;  // raised to reject motor EMI noise floor

// Gate logic:
//   1. Hold start position  → isArmed_ = true, bias recalibrated.
//      Must still be in position when piezo fires.
//   2. Press piezo          → if armed, isActive_ = true, motor feedback live.
//      Once active, further piezo events are ignored (motor buzz causes false triggers).
//   3. SwingFeedback timer  → reset() clears both flags.

class SwingProcessor : public PositionDetector {
public:
    using MagnitudeCallback     = std::function<void(float accel_mag)>;
    using PositionStateCallback = std::function<void(bool upright)>;

    explicit SwingProcessor(float accel_deadband = ACCEL_DEADBAND,
                            int   calib_samples  = CALIB_SAMPLES,
                            int   recal_samples  = RECAL_SAMPLES);

    void setMagnitudeCallback(MagnitudeCallback cb);
    void setPositionCallback(PositionStateCallback cb);

    void onForceReady(bool correctForce);
    bool isActive() const { return isActive_; }
    void onPositionReady(bool upright);
    void reset();

    void onSample(float ax, float ay, float az,
                  float mx, float my) override;

    void operator()(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my);

private:
    float                 accel_deadband_;
    SwingCalibrator       calibrator_;
    MagnitudeCallback     mag_callback_;
    PositionStateCallback pos_callback_;

    bool isArmed_      = false;  // position confirmed, ready for piezo
    bool isActive_     = false;  // gate open, motor feedback running
    bool hasLeftStart_ = false;  // paddle has left the starting position since activation

    static constexpr int COOLDOWN_MS = 500;
    std::chrono::steady_clock::time_point reset_time_{};


};
