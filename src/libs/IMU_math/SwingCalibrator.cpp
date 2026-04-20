#include "SwingCalibrator.hpp"

/// Start in initial-calibration mode with empty accumulators and zero bias.
SwingCalibrator::SwingCalibrator(int initial_samples, int recal_samples)
    : initial_samples_(initial_samples),
      recal_samples_(recal_samples),
      state_(State::INITIAL),
      count_(0),
      sum_x_(0.f), sum_y_(0.f), sum_z_(0.f),
      bias_x_(0.f), bias_y_(0.f), bias_z_(0.f)
{}

/// Register the callback fired when a calibration pass completes.
void SwingCalibrator::setCallback(DoneCallback cb)
{
    /// Register completion callback.
    callback_ = std::move(cb);
}

bool SwingCalibrator::feed(float ax, float ay, float az)
{
    switch (state_) {
    case State::INITIAL:
        /// Consume samples for initial calibration.
        accumulate(ax, ay, az, initial_samples_, true);
        return true;

    case State::RECAL:
        /// Consume samples for recalibration.
        accumulate(ax, ay, az, recal_samples_, false);
        return true;

    case State::READY:
        /// Calibration idle; caller can use the current bias.
        return false;
    }
    return false;
}

void SwingCalibrator::triggerRecal()
{
    /// Ignore if a calibration pass is already in progress.
    if (state_ != State::READY) return;  
    /// already calibrating
    
    /// Reset accumulators and enter recalibration state.
    state_ = State::RECAL;
    count_ = 0;
    sum_x_ = sum_y_ = sum_z_ = 0.f;
}

bool SwingCalibrator::isReady() const
{
    return state_ == State::READY;
}

void SwingCalibrator::accumulate(float ax, float ay, float az, int target, bool initial)
{
    /// Update running sums for the current calibration window.
    sum_x_ += ax;
    sum_y_ += ay;
    sum_z_ += az;
    ++count_;

    /// Finalise once the target sample count is reached.
    if (count_ == target) {
        bias_x_ = sum_x_ / target;
        bias_y_ = sum_y_ / target;
        bias_z_ = sum_z_ / target;

        /// Reset state for normal operation.
        state_  = State::READY;
        count_  = 0;
        sum_x_  = sum_y_ = sum_z_ = 0.f;
        
        /// Notify caller that a new bias is available.
        if (callback_) callback_(bias_x_, bias_y_, bias_z_, initial);
    }
}
