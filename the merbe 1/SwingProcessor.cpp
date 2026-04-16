#include "SwingProcessor.hpp"

#include <cmath>
#include <cstdio>

SwingProcessor::SwingProcessor(float accel_deadband, int calib_samples, int recal_samples)
    : accel_deadband_(accel_deadband),
      calibrator_(calib_samples, recal_samples)
{
    calibrator_.setCallback([](float, float, float, bool) {});

    PositionDetector::setCallback([this](bool upright) {
        if (pos_callback_) pos_callback_(upright);
        onPositionReady(upright);
    });
}

void SwingProcessor::setMagnitudeCallback(MagnitudeCallback cb)
{
    mag_callback_ = std::move(cb);
}

void SwingProcessor::setPositionCallback(PositionStateCallback cb)
{
    pos_callback_ = std::move(cb);
}

void SwingProcessor::onPositionReady(bool upright)
{
    auto elapsed = std::chrono::steady_clock::now() - reset_time_;
    if (elapsed < std::chrono::milliseconds(COOLDOWN_MS)) return;

    if (upright && !isArmed_) {
        std::printf("In position — hold and press piezo\n");
        isArmed_ = true;
        calibrator_.triggerRecal();
    } else if (!upright && isArmed_ && !isActive_) {
        // Left position before piezo — disarm
        std::printf("Left position before piezo — disarmed\n");
        isArmed_ = false;
    }
    // If already active (mid-swing), leaving position is expected — don't disarm
}

void SwingProcessor::onForceReady(bool correctForce)
{
    if (!correctForce) {
        // Dip-only event — ignore completely while active.
        // Motor vibration feeds back to the piezo and causes false dips;
        // we don't want those to kill the motor mid-swing.
        return;
    }

    if (!isArmed_) {
        std::printf("Piezo hit but not in position — hold start position first\n");
        return;
    }

    if (isActive_) return;  // already swinging, ignore re-triggers

    std::printf("Gate open — swing!\n");
    isActive_ = true;
}

void SwingProcessor::reset()
{
    isArmed_    = false;
    isActive_   = false;
    reset_time_ = std::chrono::steady_clock::now();
    std::printf("Gate reset — return to start position\n");
}

void SwingProcessor::onSample(float ax, float ay, float az,
                              float mx, float my)
{
    PositionDetector::onSample(ax, ay, az, mx, my);

    if (!isActive_) return;
    if (calibrator_.feed(ax, ay, az)) return;

    float lin_ax = ax - calibrator_.biasX();
    float lin_ay = ay - calibrator_.biasY();
    float lin_az = az - calibrator_.biasZ();

    float lin_mag = std::sqrt(lin_ax*lin_ax + lin_ay*lin_ay + lin_az*lin_az);
    if (lin_mag < accel_deadband_) lin_mag = 0.f;

    if (mag_callback_) mag_callback_(lin_mag);
}

void SwingProcessor::operator()(float ax, float ay, float az,
                                float /*gx*/, float /*gy*/, float /*gz*/,
                                float mx, float my)
{
    onSample(ax, ay, az, mx, my);
}