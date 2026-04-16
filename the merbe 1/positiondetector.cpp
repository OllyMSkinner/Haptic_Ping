#include "positiondetector.hpp"


PositionDetector::PositionDetector()
    : PositionDetector(Config{}) {}

PositionDetector::PositionDetector(const Config& c)
    : cfg(c), stableCount(0), confirmed(false)
{
    float mag = std::sqrt(c.refX*c.refX + c.refY*c.refY + c.refZ*c.refZ);
    nRefX = c.refX / mag;
    nRefY = c.refY / mag;
    nRefZ = c.refZ / mag;
}

float PositionDetector::wrapAngleDeg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void PositionDetector::resetState()
{
    stableCount = 0;
    confirmed   = false;
}

void PositionDetector::onSample(float ax, float ay, float az, float mx, float my)
{
    float g = std::sqrt(ax * ax + ay * ay + az * az);
    bool accelOK = false;
    bool headingOK = false;

    float headingDeg = std::atan2(my, mx) * 180.0f / 3.14159265f;
    float headingError = wrapAngleDeg(headingDeg - cfg.refHeadingDeg);
    headingOK = (std::fabs(headingError) <= cfg.headingToleranceDeg);

    if (g >= cfg.minG && g <= cfg.maxG) {
        float nx = ax / g;
        float ny = ay / g;
        float nz = az / g;

        float dot = nx * nRefX + ny * nRefY + nz * nRefZ;
        accelOK = (dot >= cfg.minDot);
    }

    bool ok = accelOK && headingOK;

    if (ok) {
        if (stableCount < cfg.stabilitySamples) {
            ++stableCount;
        }
    } else {
        stableCount = 0;
    }

    bool nowConfirmed = (stableCount >= cfg.stabilitySamples);

    if (nowConfirmed != confirmed) {
        confirmed = nowConfirmed;
        if (callback) {
            callback(confirmed);
        }
    }
}