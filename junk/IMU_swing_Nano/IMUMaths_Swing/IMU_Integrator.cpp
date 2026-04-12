#include "IMU_Integrator.hpp"

IMUIntegrator::IMUIntegrator()
    : first_(true), state_{0.f, 0.f, 0.f}, prev_{0.f, 0.f, 0.f}
{}

void IMUIntegrator::setCallback(Callback cb)
{
    callback_ = std::move(cb);
}

void IMUIntegrator::process(float x, float y, float z, float dt)
{
    if (first_) {
        // Initialise without integrating – just store the first sample.
        prev_[0] = x;  prev_[1] = y;  prev_[2] = z;
        first_ = false;
        return;
    }

    // Trapezoidal rule: integral += (prev + current) / 2 * dt
    state_[0] += (prev_[0] + x) * 0.5f * dt;
    state_[1] += (prev_[1] + y) * 0.5f * dt;
    state_[2] += (prev_[2] + z) * 0.5f * dt;

    prev_[0] = x;  prev_[1] = y;  prev_[2] = z;

    if (callback_) {
        callback_(state_[0], state_[1], state_[2]);
    }
}

void IMUIntegrator::reset()
{
    first_    = true;
    state_[0] = state_[1] = state_[2] = 0.f;
    prev_[0]  = prev_[1]  = prev_[2]  = 0.f;
}
