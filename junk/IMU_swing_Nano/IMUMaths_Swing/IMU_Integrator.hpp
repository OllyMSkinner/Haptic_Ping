#pragma once

#include <functional>

// Numerically integrates a 3-axis signal using the trapezoidal rule.
// Call process() once per sample with the current (x, y, z) values and the
// elapsed time since the last sample (dt, in seconds).  When the integral
// is updated the registered callback is fired with the accumulated result.
class IMUIntegrator {
public:
    using Callback = std::function<void(float x, float y, float z)>;

    IMUIntegrator();

    // Register the function that will be called with the integrated output.
    void setCallback(Callback cb);

    // Feed one sample.  dt is the time elapsed since the previous call (s).
    // On the very first call dt is ignored and the state is initialised to 0.
    void process(float x, float y, float z, float dt);

    // Reset the accumulated state back to zero.
    void reset();

private:
    bool   first_;
    float  state_[3];
    float  prev_[3];
    Callback callback_;
};
