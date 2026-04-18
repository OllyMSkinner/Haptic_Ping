#include "event_detector.h"

#include <cstdio>
#include <utility>

PiezoEventDetector::PiezoEventDetector(SimpleLEDController& ledRef,
                                       PiezoEventDetectorSettings settings)
    : led_(ledRef), settings_(settings)
{
}

void PiezoEventDetector::setForceCallback(ForceCallback cb)
{
    forceCallback_ = std::move(cb);
}

void PiezoEventDetector::reset()
{
    pressed_     = false;
    filter_init_ = false;
    filtered_v_  = 0.0f;
    led_.set(false);
}

void PiezoEventDetector::processSample(float v)
{
    if (!filter_init_) {
        filtered_v_  = v;
        filter_init_ = true;
    } else {
        filtered_v_ = 0.2f * filtered_v_ + 0.8f * v;
    }
    v = filtered_v_;

    if (settings_.enableDebugPrints) {
        std::printf("[piezo] v=%.3f pressed=%s\n",
                    v,
                    pressed_ ? "yes" : "no");
    }

    if (!pressed_ && v >= settings_.pressThreshold) {
        pressed_ = true;
        led_.set(true);

        if (settings_.enableDebugPrints) {
            std::printf("[piezo] pressed -> LED ON\n");
        }

        if (forceCallback_) {
            forceCallback_(true);
        }
        return;
    }

    if (pressed_ && v <= settings_.releaseThreshold) {
        pressed_ = false;
        led_.set(false);

        if (settings_.enableDebugPrints) {
            std::printf("[piezo] released -> LED OFF\n");
        }

        if (forceCallback_) {
            forceCallback_(false);
        }
    }
}