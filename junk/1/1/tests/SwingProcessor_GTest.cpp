#include <gtest/gtest.h>
#include "SwingProcessor.hpp"

static void sendGoodSample(SwingProcessor& proc, int n = 1)
{
    for (int i = 0; i < n; ++i)
        proc.onSample(-9.906f, 0.488f, 1.278f, 1.0f, 0.0f);
}

TEST(test_swingprocessor, checkforce_alone_does_not_fire_magnitude)
{
    SwingProcessor proc(0.05f, 1, 1);
    bool called = false;
    proc.setMagnitudeCallback([&](float) { called = true; });

    proc.onForceReady(true);
    sendGoodSample(proc, 2);

    EXPECT_FALSE(called);
}

TEST(test_swingprocessor, checkposition_alone_does_not_fire_magnitude)
{
    SwingProcessor proc(0.05f, 1, 1);
    bool called = false;
    proc.setMagnitudeCallback([&](float) { called = true; });

    sendGoodSample(proc, 50);

    EXPECT_FALSE(called);
}

TEST(test_swingprocessor, checkgate_opens_when_both_position_and_force_ready)
{
    SwingProcessor proc(0.05f, 1, 1);
    bool called = false;
    proc.setMagnitudeCallback([&](float) { called = true; });

    sendGoodSample(proc, 50);
    proc.onForceReady(true);
    sendGoodSample(proc, 1);
    sendGoodSample(proc, 1);

    EXPECT_TRUE(called);
}

TEST(test_swingprocessor, checkforce_false_zeros_magnitude_when_active)
{
    SwingProcessor proc(0.05f, 1, 1);
    float lastMag = -1.f;
    proc.setMagnitudeCallback([&](float m) { lastMag = m; });

    sendGoodSample(proc, 50);
    proc.onForceReady(true);
    sendGoodSample(proc, 2);

    proc.onForceReady(false);
    EXPECT_NEAR(lastMag, 0.f, 0.001f);
}
