#include "positiondetector.hpp"


PositionDetector::PositionDetector()
    : PositionDetector(Config{}) {}

PositionDetector::PositionDetector(const Config& c)
    : cfg(c), stableCount(0), confirmed(false) {}

float PositionDetector::wrapAngleDeg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
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

        float dot = nx * cfg.refX + ny * cfg.refY + nz * cfg.refZ;
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