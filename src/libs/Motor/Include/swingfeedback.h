/*
    This header declares the swing feedback class.
    It manages PWM feedback output, handles level changes,
    checks for time-outs, and supports a reset callback.
*/

#pragma once

#include "rpi_pwm.h"

#include <atomic>
#include <chrono>
#include <functional>

// Declares the swing feedback class, including its callback type and main functions for handling feedback, time-outs, and reset behaviour.
class SwingFeedback {
public:
    using ResetCallback = std::function<void()>;

    explicit SwingFeedback(RPI_PWM& pwm);

    void setResetCallback(ResetCallback cb);
    void checkTimeout();
    void onLevel(const char* level);
    void forceOff();

private:
    // Declares the internal worker function and the private members used to track PWM output, callback state, pending level, and timeout timing.
    void worker();

    RPI_PWM&           pwm_;
    ResetCallback      reset_cb_;
    std::atomic<bool>  busy_{false};
    std::atomic<int>   pending_level_{0};
    std::atomic<int64_t> busy_since_ms_{0};  // ms since epoch; 0 = idle

    static constexpr int64_t TIMEOUT_MS = 3000;
};
