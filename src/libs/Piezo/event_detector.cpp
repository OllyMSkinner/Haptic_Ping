/**  This class processes piezo sensor samples to detect press and release events.
  *  It smooths the input signal, compares it against press/release thresholds,
  *  updates the LED state, and triggers a callback when the state changes. */

#include "event_detector.h"

#include <cstdio>
#include <utility>

/// Stores the LED reference and detector settings.
PiezoEventDetector::PiezoEventDetector(SimpleLEDController& ledRef,
                                       PiezoEventDetectorSettings settings)
    : led_(ledRef), settings_(settings)
{
}

/// Sets the callback used when the press state changes.
void PiezoEventDetector::setForceCallback(ForceCallback cb)
{
    forceCallback_ = std::move(cb);
}

/// Resets the detector state and turns the LED off.
void PiezoEventDetector::reset()
{
    pressed_     = false;
    filter_init_ = false;
    filtered_v_  = 0.0f;
    led_.set(false);
}

/// Filters the sample, checks thresholds, updates state, and triggers outputs.
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
