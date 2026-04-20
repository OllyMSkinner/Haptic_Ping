#include "positiondetector.hpp"

/// Default constructor delegates to config-based constructor.
PositionDetector::PositionDetector()
    : PositionDetector(Config{}) {}

PositionDetector::PositionDetector(const Config& c)
    : cfg(c), stableCount(0), confirmed(false)
{
    /// Precompute normalised reference gravity vector.
    float mag = std::sqrt(c.refX*c.refX + c.refY*c.refY + c.refZ*c.refZ);
    nRefX = c.refX / mag;
    nRefY = c.refY / mag;
    nRefZ = c.refZ / mag;
}

float PositionDetector::wrapAngleDeg(float a)
{
    /// Wrap angle into [-180, 180] range.
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void PositionDetector::resetState()
{
    /// Reset stability tracking and confirmation state.
    stableCount = 0;
    confirmed   = false;
}

void PositionDetector::onSample(float ax, float ay, float az, float mx, float my)
{
    /// Compute acceleration magnitude.
    float g = std::sqrt(ax * ax + ay * ay + az * az);
    bool accelOK = false;
    bool headingOK = false;

    /// Compute heading from magnetometer.
    float headingDeg = std::atan2(my, mx) * 180.0f / 3.14159265f;

    /// Check heading against reference (with wrap-around).
    float headingError = wrapAngleDeg(headingDeg - cfg.refHeadingDeg);
    headingOK = (std::fabs(headingError) <= cfg.headingToleranceDeg);

    /// Check if acceleration magnitude is within expected bounds.
    if (g >= cfg.minG && g <= cfg.maxG) {
        /// Normalise acceleration vector.
        float nx = ax / g;
        float ny = ay / g;
        float nz = az / g;

        /// Compare with reference orientation using dot product.
        float dot = nx * nRefX + ny * nRefY + nz * nRefZ;
        accelOK = (dot >= cfg.minDot);
    }

    /// Position valid only if both checks pass.
    bool ok = accelOK && headingOK;

    if (ok) {
        /// Count consecutive valid samples.
        if (stableCount < cfg.stabilitySamples) {
            ++stableCount;
        }
    } else {
        /// Reset on failure.
        stableCount = 0;
    }
    
    /// Confirm only after enough consecutive valid samples.
    bool nowConfirmed = (stableCount >= cfg.stabilitySamples);

    /// Emit callback only on state change.
    if (nowConfirmed != confirmed) {
        confirmed = nowConfirmed;
        if (callback) {
            callback(confirmed);
        }
    }
}
