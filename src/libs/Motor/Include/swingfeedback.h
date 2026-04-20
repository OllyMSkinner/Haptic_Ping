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

<<<<<<< HEAD
// Declares the swing feedback class, including its callback type and main functions for handling feedback, time-outs, and reset behaviour.
=======
/// Drives PWM feedback from detected swing levels.

>>>>>>> BeforeSubmission
class SwingFeedback {
public:
    /// Fired when the feedback cycle times out and needs a reset.
    using ResetCallback = std::function<void()>;

    explicit SwingFeedback(RPI_PWM& pwm);

    void setResetCallback(ResetCallback cb);
    void checkTimeout();
    void onLevel(const char* level);
    void forceOff();

private:
<<<<<<< HEAD
    // Declares the internal worker function and the private members used to track PWM output, callback state, pending level, and timeout timing.
=======
    /// Internal worker for applying queued feedback updates.
>>>>>>> BeforeSubmission
    void worker();

    /// PWM output owned externally.
    RPI_PWM&           pwm_;

    /// Optional callback for timeout/reset handling.
    ResetCallback      reset_cb_;

    /// True while a feedback cycle is active.
    std::atomic<bool>  busy_{false};

    /// Pending duty-cycle level to apply.
    std::atomic<int>   pending_level_{0};

    /// Start time of the active feedback cycle in ms since epoch.
    /// Zero means idle.
    std::atomic<int64_t> busy_since_ms_{0};

    static constexpr int64_t TIMEOUT_MS = 3000;
};
