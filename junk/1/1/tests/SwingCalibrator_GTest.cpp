#include <gtest/gtest.h>
#include "SwingCalibrator.hpp"

TEST(test_swingcalibrator, checkfeed_returns_true_during_calibration)
{
    SwingCalibrator cal(5, 3);
    for (int i = 0; i < 5; ++i)
        EXPECT_TRUE(cal.feed(0.f, 0.f, 9.81f));
    EXPECT_FALSE(cal.feed(0.f, 0.f, 9.81f));
}

TEST(test_swingcalibrator, checkbias_is_average_of_samples)
{
    SwingCalibrator cal(4, 2);
    cal.feed(1.f, 2.f, 3.f);
    cal.feed(3.f, 4.f, 5.f);
    cal.feed(5.f, 6.f, 7.f);
    cal.feed(7.f, 8.f, 9.f);

    EXPECT_NEAR(cal.biasX(), 4.f, 0.01f);
    EXPECT_NEAR(cal.biasY(), 5.f, 0.01f);
    EXPECT_NEAR(cal.biasZ(), 6.f, 0.01f);
}

TEST(test_swingcalibrator, checkisready_after_initial_phase)
{
    SwingCalibrator cal(3, 2);
    EXPECT_FALSE(cal.isReady());
    cal.feed(0.f, 0.f, 9.81f);
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_FALSE(cal.isReady());
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_TRUE(cal.isReady());
}

TEST(test_swingcalibrator, checkcallback_fires_on_completion)
{
    bool fired      = false;
    bool wasInitial = false;
    SwingCalibrator cal(3, 2);
    cal.setCallback([&](float, float, float, bool initial) {
        fired      = true;
        wasInitial = initial;
    });

    cal.feed(0.f, 0.f, 9.81f);
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_FALSE(fired);
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_TRUE(fired);
    EXPECT_TRUE(wasInitial);
}

TEST(test_swingcalibrator, checktriggerrecal_refreshes_bias)
{
    SwingCalibrator cal(3, 2);
    for (int i = 0; i < 3; ++i) cal.feed(0.f, 0.f, 9.81f);

    cal.triggerRecal();
    cal.feed(2.f, 4.f, 6.f);
    cal.feed(4.f, 8.f, 2.f);

    EXPECT_TRUE(cal.isReady());
    EXPECT_NEAR(cal.biasX(), 3.f, 0.01f);
    EXPECT_NEAR(cal.biasY(), 6.f, 0.01f);
    EXPECT_NEAR(cal.biasZ(), 4.f, 0.01f);
}

TEST(test_swingcalibrator, checktriggerrecal_ignored_while_calibrating)
{
    SwingCalibrator cal(3, 2);
    cal.feed(0.f, 0.f, 9.81f);
    cal.triggerRecal();
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_FALSE(cal.isReady());
    cal.feed(0.f, 0.f, 9.81f);
    EXPECT_TRUE(cal.isReady());
}
