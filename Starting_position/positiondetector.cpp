#include "positiondetector.h"

#include <iostream>
#include <cmath>

namespace PositionDetectorName {

static constexpr int stabilitySamplesRequired = 4;

// Reference orientation based on measured "ready" position (AX≈-8, AY≈-5.5, AZ≈2)
// Normalized values ≈ (-0.81, -0.56, 0.20)
static constexpr float TARGET_NX = -0.81f;
static constexpr float TARGET_NY = -0.56f;
static constexpr float TARGET_NZ =  0.20f;

// Allow a generous angular tolerance (dot >= 0.65 → ±~49°)
static constexpr float MIN_DOT = 0.65f;

// Keep gravity within a realistic range but not too strict
static constexpr float MIN_G = 8.0f;
static constexpr float MAX_G = 11.0f;

// Gyro near stillness, relaxed slightly
static constexpr float GYRO_LIMIT = 0.25f;

PositionDetector::PositionDetector()
    : Counter(0),
      UprightConfirmed(false),
      callback(nullptr)
{
}

void PositionDetector::HasSample(const Sample& sample)
{
    CheckPosition(
        sample.ax, sample.ay, sample.az,
        sample.gx, sample.gy, sample.gz,
        sample.mx, sample.my, sample.mz
    );
}

void PositionDetector::CheckPosition(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz)
{
    const float g = std::sqrt(ax * ax + ay * ay + az * az);

    bool accelOK = false;
    float dot = -2.0f;

    if (g >= MIN_G && g <= MAX_G) {
        const float nx = ax / g;
        const float ny = ay / g;
        const float nz = az / g;

        dot = (nx * TARGET_NX) + (ny * TARGET_NY) + (nz * TARGET_NZ);
        accelOK = (dot >= MIN_DOT);
    }

    const bool gyroStable =
        (std::fabs(gx) <= GYRO_LIMIT) &&
        (std::fabs(gy) <= GYRO_LIMIT) &&
        (std::fabs(gz) <= GYRO_LIMIT);

    const bool isUprightNow = accelOK && gyroStable;

    std::cout << "AX=" << ax
              << " AY=" << ay
              << " AZ=" << az
              << " | GX=" << gx
              << " GY=" << gy
              << " GZ=" << gz
              << " | MX=" << mx
              << " MY=" << my
              << " MZ=" << mz
              << " | g=" << g
              << " | dot=" << dot
              << " | ACCEL_OK=" << accelOK
              << " | GYRO_OK=" << gyroStable
              << " | UprightNow=" << isUprightNow
              << " | Counter=" << Counter
              << " | Confirmed=" << UprightConfirmed
              << std::endl;

    if (isUprightNow) {
        if (Counter < stabilitySamplesRequired) {
            ++Counter;
        }
    } else {
        Counter = 0;
    }

    const bool newConfirmed = (Counter >= stabilitySamplesRequired);

    if (newConfirmed != UprightConfirmed) {
        UprightConfirmed = newConfirmed;

        std::cout << "[EVENT] Upright state changed -> "
                  << (UprightConfirmed ? "TRUE" : "FALSE")
                  << std::endl;

        if (callback) {
            callback->OnUprightStateChanged(UprightConfirmed);
        }
    }
}

} // namespace PositionDetectorName
