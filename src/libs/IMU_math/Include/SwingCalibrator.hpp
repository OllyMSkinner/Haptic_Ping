#pragma once

/** SOLID Principles:
* S - This file has exactly one function - to remove the gravitational bias from the 
* acceleration value of each axis.
* I - The accumulate() is kept private as it is only used for moving average calibration, 
* other caller doesn't need it. 
* D - The std::function, which is an abstraction, allows the SwingCalibrator to do 
* its job without needing to know the caller or the results. */


#include <functional>

/** Manages gravity-bias calibration for the swing detector.
*
* Usage:
*   1. Construct and set a callback.
*   2. Call feed() on every IMU sample.
*      - While calibrating, feed() returns true (caller should skip measurement).
*      - Once done, feed() returns false and the bias is ready via biasX/Y/Z().
*   3. Call triggerRecal() whenever the position detector confirms the start
*      position. The next RECAL_SAMPLES samples will be used to refresh the bias. */

class SwingCalibrator {
public:

    /** Fired when a calibration phase completes.
    * bx/by/bz  – new gravity bias (m/s²)
    * initial   – true for the first calibration, false for recalibrations */
    using DoneCallback = std::function<void(float bx, float by, float bz, bool initial)>;

    explicit SwingCalibrator(int initial_samples = 200, int recal_samples = 50);

    void setCallback(DoneCallback cb);

    /** Feed one raw accel sample (m/s²).
    * Returns true  → sample was consumed for calibration; skip measurement output.
    * Returns false → calibration is idle; use biasX/Y/Z() to correct the sample. */
    bool feed(float ax, float ay, float az);

    /// Request a recalibration burst (e.g. triggered by the position detector).
    /// Ignored if a calibration phase is already in progress.
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
