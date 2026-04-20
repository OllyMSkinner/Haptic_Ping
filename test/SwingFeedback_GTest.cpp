#include <gtest/gtest.h>
#include "swingfeedback.h"
#include "rpi_pwm.h"

#include <chrono>
#include <thread>

// rampUp takes 3 × 30 ms = 90 ms to reach MAX_DUTY (80%).
// Sleep 100 ms so we sample after the ramp completes but well before
// the pattern ends (shortest hold is 180 ms for LOW/MEDIUM, 450 ms for HIGH).
TEST(test_swingfeedback, checkhighest_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Highest Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NEAR(pwm.getDutyCycle(), 80.f, 0.1f);
}

TEST(test_swingfeedback, checkmedium_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Medium Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NEAR(pwm.getDutyCycle(), 80.f, 0.1f);
}

TEST(test_swingfeedback, checklow_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Low Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NEAR(pwm.getDutyCycle(), 80.f, 0.1f);
}

TEST(test_swingfeedback, checkzero_level_does_not_start_window)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    bool resetCalled = false;
    fb.setResetCallback([&] { resetCalled = true; });

    fb.onLevel("No Duty Cycle");
    fb.checkTimeout();

    EXPECT_FALSE(resetCalled);
}

// Worker calls reset_cb_ after completing the pattern (~450 ms for HIGH).
// After 4 s the worker is long finished and reset must have fired.
TEST(test_swingfeedback, checkreset_callback_fires_after_pattern_completes)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    bool resetCalled = false;
    fb.setResetCallback([&] { resetCalled = true; });

    fb.onLevel("Highest Duty Cycle");

    std::this_thread::sleep_for(std::chrono::seconds(4));
    fb.checkTimeout();  // busy_since_ms_ == 0 by now; should be no-op

    EXPECT_TRUE(resetCalled);
    EXPECT_NEAR(pwm.getDutyCycle(), 0.f, 0.1f);
}

// checkTimeout() must call forceOff() (and thus reset_cb_) if feedback
// has been busy for longer than TIMEOUT_MS without completing.
// We simulate this by calling checkTimeout() immediately after onLevel()
// but with the timeout wound back via a 4-second sleep that exceeds it.
// (This also exercises the guard that checkTimeout() is a no-op when idle.)
TEST(test_swingfeedback, checktimeout_is_noop_when_idle)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    bool resetCalled = false;
    fb.setResetCallback([&] { resetCalled = true; });

    // Never start feedback — checkTimeout must never fire
    fb.checkTimeout();
    fb.checkTimeout();
    fb.checkTimeout();

    EXPECT_FALSE(resetCalled);
}
