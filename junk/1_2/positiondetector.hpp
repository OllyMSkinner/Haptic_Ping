#pragma once

#include <cmath>
#include <functional>

class PositionDetector {
public:
    using StateCallback = std::function<void(bool upright)>;

    struct Config {
        float refX =  9.88f;
        float refY = -0.87f;
        float refZ =  0.10f;

        float minG = 7.0f;
        float maxG = 12.5f;
        float minDot = 0.90f;

        float refHeadingDeg = 25.0f;
        float headingToleranceDeg = 180.0f;

        int stabilitySamples = 10;
    };

    PositionDetector();
    explicit PositionDetector(const Config& c);
    virtual ~PositionDetector() = default;

    virtual void onSample(float ax, float ay, float az, float mx, float my);
    void setCallback(StateCallback cb) { callback = std::move(cb); }
    // Clears stability count and confirmed state so the callback can re-fire
    // after a cycle reset while the device stays at the same position.
    void resetState();

private:
    static float wrapAngleDeg(float a);

    Config cfg;
    float nRefX, nRefY, nRefZ;   // normalised ref vector, computed from cfg
    int stableCount;
    bool confirmed;
    StateCallback callback;
};
