#include <gtest/gtest.h>
#include "swingfeedback.h"
#include "rpi_pwm.h"

#include <chrono>
#include <thread>

TEST(test_swingfeedback, checkhighest_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Highest Duty Cycle");
    fb.onLevel("No Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_NEAR(pwm.getDutyCycle(), 80.f, 0.1f);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
}

TEST(test_swingfeedback, checkmedium_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Medium Duty Cycle");
    fb.onLevel("No Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_NEAR(pwm.getDutyCycle(), 80.f, 0.1f);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
}

TEST(test_swingfeedback, checklow_level_drives_motor)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    fb.onLevel("Low Duty Cycle");
    fb.onLevel("No Duty Cycle");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

TEST(test_swingfeedback, checkreset_callback_fires_after_pattern_completes)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    bool resetCalled = false;
    fb.setResetCallback([&] { resetCalled = true; });

    fb.onLevel("Highest Duty Cycle");
    fb.onLevel("No Duty Cycle");

    std::this_thread::sleep_for(std::chrono::seconds(4));
    fb.checkTimeout();

    EXPECT_TRUE(resetCalled);
    EXPECT_NEAR(pwm.getDutyCycle(), 0.f, 0.1f);
}

TEST(test_swingfeedback, checktimeout_is_noop_when_idle)
{
    RPI_PWM pwm;
    SwingFeedback fb(pwm);
    bool resetCalled = false;
    fb.setResetCallback([&] { resetCalled = true; });

    fb.checkTimeout();
    fb.checkTimeout();
    fb.checkTimeout();

    EXPECT_FALSE(resetCalled);
}
