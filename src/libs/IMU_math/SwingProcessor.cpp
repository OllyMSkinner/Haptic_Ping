#include "SwingProcessor.hpp"

#include <cmath>

// Initialise the deadband and embedded gravity-bias calibrator.
SwingProcessor::SwingProcessor(float accel_deadband, int calib_samples, int recal_samples)
    : accel_deadband_(accel_deadband),
      calibrator_(calib_samples, recal_samples)
{
    // Default no-op calibration callback.
    calibrator_.setCallback([](float, float, float, bool) {});

    // Forward position events to any external callback, then update swing state.
    PositionDetector::setCallback([this](bool upright) {
        if (pos_callback_) pos_callback_(upright);
        onPositionReady(upright);
    });
}

// Register callback for filtered linear-acceleration magnitude.
void SwingProcessor::setMagnitudeCallback(MagnitudeCallback cb)
{
    mag_callback_ = std::move(cb);
}

// Register callback for upright/not-upright state changes.
void SwingProcessor::setPositionCallback(PositionStateCallback cb)
{
    pos_callback_ = std::move(cb);
}

void SwingProcessor::onPositionReady(bool upright)
{
    // Ignore position events during the post-reset cooldown.
    auto elapsed = std::chrono::steady_clock::now() - reset_time_;
    if (elapsed < std::chrono::milliseconds(COOLDOWN_MS)) return;

    if (upright && !isArmed_) {
        // Reaching the start position arms the swing and refreshes bias.
        isArmed_ = true;
        calibrator_.triggerRecal();
    } else if (!upright && isArmed_ && !isActive_) {
        // Left the start position before force confirmation; cancel arming.
        isArmed_ = false;
    }
    // If already active, leaving the start position is expected.
}

void SwingProcessor::onForceReady(bool correctForce)
{
    if (!correctForce) {
        // Drop output to zero if force is lost mid-swing.
        if (isActive_ && mag_callback_) mag_callback_(0.f);
        return;
    }

    // Ignore force events unless the system is armed from the start position.
    if (!isArmed_) {
        return;
    }

    // Ignore duplicate triggers while already active.
    if (isActive_) return;

    // Force confirmation starts the swing.
    isActive_ = true;
}

void SwingProcessor::reset()
{
    // Clear swing state and restart cooldown.
    isArmed_    = false;
    isActive_   = false;
    reset_time_ = std::chrono::steady_clock::now();
}

void SwingProcessor::onSample(float ax, float ay, float az,
                              float mx, float my)
{
    // Feed the sample into the position detector first.
    PositionDetector::onSample(ax, ay, az, mx, my);

    // Only process magnitude while a swing is active.
    if (!isActive_) return;

    // Hold output until the current calibration window completes.
    if (calibrator_.feed(ax, ay, az)) return;

    // Remove gravity bias to estimate linear acceleration.
    float lin_ax = ax - calibrator_.biasX();
    float lin_ay = ay - calibrator_.biasY();
    float lin_az = az - calibrator_.biasZ();

    // Collapse 3D linear acceleration into a scalar magnitude.
    float lin_mag = std::sqrt(lin_ax*lin_ax + lin_ay*lin_ay + lin_az*lin_az);

    // Clamp small values to zero to suppress noise.
    if (lin_mag < accel_deadband_) lin_mag = 0.f;

    if (mag_callback_) mag_callback_(lin_mag);
}

void SwingProcessor::operator()(float ax, float ay, float az,
                                float /*gx*/, float /*gy*/, float /*gz*/,
                                float mx, float my)
{
    // Adapter for callback-style IMU interfaces.
    onSample(ax, ay, az, mx, my);
}
