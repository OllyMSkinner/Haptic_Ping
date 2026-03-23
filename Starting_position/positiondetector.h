#ifndef POSITIONDETECTOR_H
#define POSITIONDETECTOR_H

#include "../IMU_processing/imudriver/icm20948_i2c.hpp"

namespace PositionDetectorName {

// Callback interface for reporting upright-state changes
class UprightCallback {
public:
    virtual ~UprightCallback() = default;
    virtual void OnUprightStateChanged(bool isUpright) = 0;
};

class PositionDetector {
public:
    using Sample = icm20948::IMUSample;
    using Callback = UprightCallback;

    PositionDetector();

    // Called by IMU publisher when a new sample arrives
    void HasSample(const Sample& sample);

    void RegisterCallback(Callback* cb) { callback = cb; }

    bool IsUpright() const { return UprightConfirmed; }

private:
    void CheckPosition(
        float ax, float ay, float az,
        float gx, float gy, float gz,
        float mx, float my, float mz);

    int Counter;
    bool UprightConfirmed;
    Callback* callback;
};

} // namespace PositionDetectorName

#endif