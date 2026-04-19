// This header defines the piezo event detector and its settings.
// The detector is responsible for interpreting sensor samples from the ADC,
// tracking press/release state, and exposing higher-level event behaviour
// through a callback while also providing direct LED feedback.


// SOLID principle:
//   S -   The main responsibility of this class is to interpret piezo samples using
//         filtering and threshold logic.
//   O -   New behaviours can be attached through the callback without changing the
//         detector logic itself.
//   L/D - The detector depends on a simpler LED abstraction and a callback rather than
//         on broader application logic, helping to reduce coupling.

#pragma once

#include <functional>

#include "led_controller.h"

// Stores the threshold and debug settings for the detector.
struct PiezoEventDetectorSettings
{
    float pressThreshold = 0.95f;
    float releaseThreshold = 0.70f;
    bool enableDebugPrints = false;
};

// Declares the detector class and its public interface.
class PiezoEventDetector
{
public:
    // Defines the callback type used for press state changes.
    using ForceCallback = std::function<void(bool)>;

    // Creates the detector with an LED reference and settings.
    PiezoEventDetector(SimpleLEDController& ledRef,
                       PiezoEventDetectorSettings settings);

    void setForceCallback(ForceCallback cb);     // Sets the callback function for state changes.
    void processSample(float v);     // Processes a new sensor sample.
    void reset();    // Resets the detector state.

private:
    SimpleLEDController& led_;    // Reference to the LED controller.
    PiezoEventDetectorSettings settings_;    // Stores the detector settings.
    ForceCallback forceCallback_;    // Stores the callback function.

    // Tracks whether the detector is currently pressed, whether the filter has been initialised and stores the current filtered sample value.
    bool  pressed_     = false;
    bool  filter_init_ = false;
    float filtered_v_  = 0.0f;
};
