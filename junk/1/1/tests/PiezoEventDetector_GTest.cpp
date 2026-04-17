#include <gtest/gtest.h>
#include "event_detector.h"
#include "ledcallback.hpp"

#include <vector>

// EMA filter inside processSample: first call sets filtered=v,
// subsequent calls: filtered = 0.2*prev + 0.8*v
// pressThreshold=0.95, releaseThreshold=0.70

class StubLED : public SimpleLEDController {
public:
    StubLED() = default;
    void set(bool on)    override { lastState = on; }
    void flashGreen(int) override {}
    void service()       override {}
    bool lastState = false;
};

TEST(test_piezoeventdetector, checkdip_then_peak_fires_true)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    bool gotTrue = false;
    det.setForceCallback([&](bool v) { if (v) gotTrue = true; });

    // 0.0 → filtered=0.0 (below 0.95, no press)
    // 3.0 → filtered=0.2*0.0+0.8*3.0=2.4 (above 0.95, press fires)
    det.processSample(0.0f);
    det.processSample(3.0f);
    EXPECT_TRUE(gotTrue);
}

TEST(test_piezoeventdetector, checksecond_peak_while_pressed_does_not_refire)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    int count = 0;
    det.setForceCallback([&](bool v) { if (v) ++count; });

    // Press once, then send another high sample — !pressed_ guard prevents re-fire
    det.processSample(0.0f);
    det.processSample(3.0f);  // press (count=1)
    det.processSample(3.0f);  // still pressed, no re-fire
    EXPECT_EQ(count, 1);
}

TEST(test_piezoeventdetector, checkpress_then_release_fires_both_events)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    std::vector<bool> events;
    det.setForceCallback([&](bool v) { events.push_back(v); });

    // 0.0 → not pressed
    // 3.0 → filtered=2.4, press
    // 0.0 → filtered=0.2*2.4+0.8*0.0=0.48 (below 0.70, release)
    det.processSample(0.0f);
    det.processSample(3.0f);
    det.processSample(0.0f);

    EXPECT_EQ(events.size(), 2u);
    EXPECT_TRUE(events[0]);
    EXPECT_FALSE(events[1]);
}

TEST(test_piezoeventdetector, checkmidrange_alone_does_not_fire)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    bool fired = false;
    det.setForceCallback([&](bool) { fired = true; });

    // 0.5 → filtered=0.5 (below 0.95 press threshold)
    det.processSample(0.5f);
    EXPECT_FALSE(fired);
}

TEST(test_piezoeventdetector, checksecond_cycle_works_after_release)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    int trueCount = 0;
    det.setForceCallback([&](bool v) { if (v) ++trueCount; });

    // Cycle 1
    det.processSample(0.3f); det.processSample(3.0f);   // press (count=1)
    det.processSample(1.5f);                             // filtered=1.692, no release yet
    det.processSample(0.3f);                             // filtered=0.578, release
    // Cycle 2
    det.processSample(3.0f);                             // press (count=2)

    EXPECT_EQ(trueCount, 2);
}

TEST(test_piezoeventdetector, checkreset_clears_state_and_allows_refire)
{
    StubLED led;
    PiezoEventDetector det(led, PiezoEventDetectorSettings{});
    int trueCount = 0;
    det.setForceCallback([&](bool v) { if (v) ++trueCount; });

    det.processSample(0.0f);
    det.processSample(3.0f);  // press (count=1)
    det.reset();              // clears pressed_, filter state, LED
    det.processSample(0.0f);
    det.processSample(3.0f);  // press again after reset (count=2)

    EXPECT_EQ(trueCount, 2);
}
