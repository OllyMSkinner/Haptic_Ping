#pragma once

#include "rpi_pwm.h"

#include <atomic>
#include <chrono>
#include <functional>

class SwingFeedback {
public:
    using ResetCallback = std::function<void()>;

    explicit SwingFeedback(RPI_PWM& pwm);

    void setResetCallback(ResetCallback cb);
    void checkTimeout();
    void onLevel(const char* level);
    void forceOff();

private:
    void worker();

    RPI_PWM&           pwm_;
    ResetCallback      reset_cb_;
    std::atomic<bool>  busy_{false};
    std::atomic<int>   pending_level_{0};
    std::atomic<int64_t> busy_since_ms_{0};  // ms since epoch; 0 = idle

    static constexpr int64_t TIMEOUT_MS = 3000;
};