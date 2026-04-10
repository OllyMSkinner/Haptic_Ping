#ifndef POSITIONDETECTOR_H
#define POSITIONDETECTOR_H

#include <atomic>

namespace PositionDetectorName {

// Existing callback interface
class UprightCallback {
public:
    virtual ~UprightCallback() = default;
    virtual void OnUprightDetected() = 0;
};

class PositionDetector {
public:
    using Callback = UprightCallback;

    PositionDetector()
        : Counter(0),
          UprightConfirmed(false),
          callback(nullptr)
    {}

    // Core logic (9-axis IMU)
    void CheckPosition(
        float ax, float ay, float az,
        float gx, float gy, float gz,
        float mx, float my, float mz);

    // Mode 1: Direct IMU on this Pi
    bool RunFromIMU(unsigned bus,
                    unsigned addr,
                    std::atomic_bool& runFlag,
                    int samplePeriodMs = 50);

    // Mode 2: Read IMU values from Bluetooth RFCOMM
    bool RunFromRFCOMM(const char* devPath,
                       std::atomic_bool& runFlag,
                       int samplePeriodMs = 50);

    void RegisterCallback(Callback* cb) { callback = cb; }

    bool IsUpright() const { return UprightConfirmed; }

private:
    int Counter;
    bool UprightConfirmed;
    Callback* callback;
};

} // namespace PositionDetectorName

#endif