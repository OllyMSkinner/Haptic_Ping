#pragma once

#include <functional>

/** Gravity-bias calibrator for the swing detector. Feed IMU samples continuously; 
 * while calibrating, feed() returns true. Once ready, biasX/Y/Z() hold the current 
 * bias estimate. triggerRecal() starts a short recalibration window.*/

class SwingCalibrator {
public:
    /// Called when a calibration pass completes.
    /// initial == true for startup calibration, false for recalibration.
    using DoneCallback = std::function<void(float bx, float by, float bz, bool initial)>;

    explicit SwingCalibrator(int initial_samples = 200, int recal_samples = 50);

    void setCallback(DoneCallback cb);

    /// Push one accel sample.
    /// Returns true if the sample was consumed for calibration.
    bool feed(float ax, float ay, float az);

    /// Start a recalibration pass if not already calibrating.
    void triggerRecal();

    bool isReady() const;

    float biasX() const { return bias_x_; }
    float biasY() const { return bias_y_; }
    float biasZ() const { return bias_z_; }

private:
    enum class State { INITIAL, READY, RECAL };

    int   initial_samples_;
    int   recal_samples_;
    State state_;
    int   count_;

    /// Running sums for the current calibration window.
    float sum_x_, sum_y_, sum_z_;

    /// Latest completed bias estimate.
    float bias_x_, bias_y_, bias_z_;

    DoneCallback callback_;

    /// Accumulate one sample and finalise when target samples are reached.
    void accumulate(float ax, float ay, float az, int target, bool initial);
};
