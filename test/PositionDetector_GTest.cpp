#include <gtest/gtest.h>
#include "positiondetector.hpp"

static void sendGoodSample(PositionDetector& det, int n = 1)
{
    for (int i = 0; i < n; ++i)
        det.onSample(9.906f, -0.488f, 1.278f, 1.0f, 0.0f);
}

static void sendBadSample(PositionDetector& det, int n = 1)
{
    for (int i = 0; i < n; ++i)
        det.onSample(0.f, 0.f, 9.81f, 1.0f, 0.0f);
}

static PositionDetector::Config fastConfig()
{
    PositionDetector::Config c;
    c.stabilitySamples = 3;
    return c;
}

TEST(test_positiondetector, checkno_callback_before_stability_samples)
{
    PositionDetector det(fastConfig());
    bool fired = false;
    det.setCallback([&](bool) { fired = true; });

    sendGoodSample(det, 2);
    EXPECT_FALSE(fired);
}

TEST(test_positiondetector, checkcallback_true_after_stability_samples)
{
    PositionDetector det(fastConfig());
    bool upright = false;
    det.setCallback([&](bool v) { upright = v; });

    sendGoodSample(det, 3);
    EXPECT_TRUE(upright);
}

TEST(test_positiondetector, checkbad_sample_resets_stability_counter)
{
    PositionDetector det(fastConfig());
    bool fired = false;
    det.setCallback([&](bool) { fired = true; });

    sendGoodSample(det, 2);
    sendBadSample(det, 1);
    sendGoodSample(det, 2);
    EXPECT_FALSE(fired);
}

TEST(test_positiondetector, checkcallback_false_when_position_lost)
{
    PositionDetector det(fastConfig());
    bool lastValue = false;
    int callCount  = 0;
    det.setCallback([&](bool v) { lastValue = v; ++callCount; });

    sendGoodSample(det, 3);
    EXPECT_EQ(callCount, 1);
    EXPECT_TRUE(lastValue);

    sendBadSample(det, 1);
    EXPECT_EQ(callCount, 2);
    EXPECT_FALSE(lastValue);
}
