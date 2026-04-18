#pragma once

#include <functional>

// Manages gravity-bias calibration for the swing detector.
//
// Usage:
//   1. Construct and set a callback.
//   2. Call feed() on every IMU sample.
//      - While calibrating, feed() returns true (caller should skip measurement).
//      - Once done, feed() returns false and the bias is ready via biasX/Y/Z().
//   3. Call triggerRecal() whenever the position detector confirms the start
//      position. The next RECAL_SAMPLES samples will be used to refresh the bias.

class SwingCalibrator {
public:
    // Fired when a calibration phase completes.
    // bx/by/bz  – new gravity bias (m/s²)
    // initial   – true for the first calibration, false for recalibrations
    using DoneCallback = std::function<void(float bx, float by, float bz, bool initial)>;

    explicit SwingCalibrator(int initial_samples = 200, int recal_samples = 50);

    void setCallback(DoneCallback cb);

    // Feed one raw accel sample (m/s²).
    // Returns true  → sample was consumed for calibration; skip measurement output.
    // Returns false → calibration is idle; use biasX/Y/Z() to correct the sample.
    bool feed(float ax, float ay, float az);

    // Request a recalibration burst (e.g. triggered by the position detector).
    // Ignored if a calibration phase is already in progress.
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
    float sum_x_, sum_y_, sum_z_;
    float bias_x_, bias_y_, bias_z_;

    DoneCallback callback_;

    void accumulate(float ax, float ay, float az, int target, bool initial);
};
