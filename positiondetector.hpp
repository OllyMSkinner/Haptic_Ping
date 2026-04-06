#pragma once

#include <cmath>
#include <functional>

class PositionDetector {
public:
    using StateCallback = std::function<void(bool upright)>;

    struct Config {
        float refX = -0.9906f;
        float refY =  0.0488f;
        float refZ =  0.1278f;

        float minG = 7.0f;
        float maxG = 12.5f;
        float minDot = 0.90f;

        float refHeadingDeg = 25.0f;
        float headingToleranceDeg = 180.0f;

        int stabilitySamples = 50;
    };

    PositionDetector();
    explicit PositionDetector(const Config& c);

    void onSample(float ax, float ay, float az, float mx, float my);
    void setCallback(StateCallback cb) { callback = std::move(cb); }

private:
    static float wrapAngleDeg(float a);

    Config cfg;
    int stableCount;
    bool confirmed;
    StateCallback callback;
};