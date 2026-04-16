#pragma once

#include <functional>

#include "led_controller.h"

struct PiezoEventDetectorSettings
{
    float pressThreshold = 0.95f;
    float releaseThreshold = 0.70f;
    bool enableDebugPrints = false;
};

class PiezoEventDetector
{
public:
    using ForceCallback = std::function<void(bool)>;

    PiezoEventDetector(SimpleLEDController& ledRef,
                       PiezoEventDetectorSettings settings);

    void setForceCallback(ForceCallback cb);
    void processSample(float v);
    void reset();

private:
    SimpleLEDController& led_;
    PiezoEventDetectorSettings settings_;
    ForceCallback forceCallback_;

    bool  pressed_     = false;
    bool  filter_init_ = false;
    float filtered_v_  = 0.0f;
};