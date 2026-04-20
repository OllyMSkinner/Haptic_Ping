/**
  *  This header declares the position detector class.
  *  It stores the configuration for detecting an upright position,
  *  processes sensor samples, tracks stability, and uses a callback
  *  when the required position has been confirmed. 
  *
  *
  *  SOLID principles:
  *    S - This class is responsible for analysing incoming motion/orientation
  *        samples and deciding whether the required upright position has been
  *        stably detected.
  *    O - Client code can extend what happens after detection by attaching a
  *       callback, without modifying the detector logic itself.
  *    L - The virtual destructor and virtual onSample() allow derived detector
  *        types to replace the base class safely, provided they preserve the same
  *        detection contract.
  *    I - The public API is intentionally small, exposing only the operations
  *        needed by client code: process samples, register a callback, and reset
  *        state. */

#pragma once

#include <cmath>
#include <functional>

/// Declares the position detector class, including its callback type, configuration settings, and main functions for processing samples and resetting state.
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
    /** Clears stability count and confirmed state so the callback can re-fire
    * after a cycle reset while the device stays at the same position. */
    void resetState();

private:
    /// Declares the helper function and private members used to store configuration, reference values, stability count, state, and callback data.
    static float wrapAngleDeg(float a);

    Config cfg;
    float nRefX, nRefY, nRefZ;   /// normalised ref vector, computed from cfg
    int stableCount;
    bool confirmed;
    StateCallback callback;
};
