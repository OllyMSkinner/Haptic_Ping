#include "SwingCalibrator.hpp"

SwingCalibrator::SwingCalibrator(int initial_samples, int recal_samples)
    : initial_samples_(initial_samples),
      recal_samples_(recal_samples),
      state_(State::INITIAL),
      count_(0),
      sum_x_(0.f), sum_y_(0.f), sum_z_(0.f),
      bias_x_(0.f), bias_y_(0.f), bias_z_(0.f)
{}

void SwingCalibrator::setCallback(DoneCallback cb)
{
    callback_ = std::move(cb);
}

bool SwingCalibrator::feed(float ax, float ay, float az)
{
    switch (state_) {
    case State::INITIAL:
        accumulate(ax, ay, az, initial_samples_, true);
        return true;

    case State::RECAL:
        accumulate(ax, ay, az, recal_samples_, false);
        return true;

    case State::READY:
        return false;
    }
    return false;
}

void SwingCalibrator::triggerRecal()
{
    if (state_ != State::READY) return;  // already calibrating
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
    sum_x_ += ax;
    sum_y_ += ay;
    sum_z_ += az;
    ++count_;

    if (count_ == target) {
        bias_x_ = sum_x_ / target;
        bias_y_ = sum_y_ / target;
        bias_z_ = sum_z_ / target;
        state_  = State::READY;
        count_  = 0;
        sum_x_  = sum_y_ = sum_z_ = 0.f;

        if (callback_) callback_(bias_x_, bias_y_, bias_z_, initial);
    }
}
