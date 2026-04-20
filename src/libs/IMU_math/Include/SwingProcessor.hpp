/** This header declares the swing processor class.
    It combines calibration, position detection, force input,
    and sample processing to control when swing feedback becomes active.*/

#pragma once

/** SOLID Principles:
* D - MagnitudeCallback and PositionStateCallback are std::function output abstractions. */

#include "SwingCalibrator.hpp"
#include "positiondetector.hpp"

#include <chrono>
#include <functional>

/// Defines the I2C bus and the main constants used for calibration, recalibration, and acceleration deadband control.
static constexpr unsigned I2C_BUS        = 1;
static constexpr int      CALIB_SAMPLES  = 50;   /// ~0.5s at 100Hz — startup bias
static constexpr int      RECAL_SAMPLES  = 20;   /// ~0.2s recal on position confirm
static constexpr float    ACCEL_DEADBAND = 1.0f;  /// raised to reject motor EMI noise floor

/** Gate logic:
   1. When the correct start position is held, isArmed_ becomes true and the bias is recalibrated.
      The system must still be in this valid position when the piezo is triggered.
   2. When the piezo is pressed while armed, isActive_ becomes true and the motor feedback starts.
      Once active, any further piezo events are ignored to avoid false triggers caused by motor vibration.
   3. When the SwingFeedback timer ends, reset() is called to clear both state flags.
*/
    
/// Declares the swing processor class, including its callback types and main functions for handling force, position, reset, and sensor sample processing.
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
    /// Declares the private members used to store thresholds, calibration, callbacks, activity state, and reset timing.
    float                 accel_deadband_;
    SwingCalibrator       calibrator_;
    MagnitudeCallback     mag_callback_;
    PositionStateCallback pos_callback_;

    bool isArmed_      = false;  /// position confirmed, ready for piezo
    bool isActive_     = false;  /// gate open, motor feedback running
    bool hasLeftStart_ = false;  /// paddle has left the starting position since activation


    static constexpr int COOLDOWN_MS = 500;
    std::chrono::steady_clock::time_point reset_time_{};


};
