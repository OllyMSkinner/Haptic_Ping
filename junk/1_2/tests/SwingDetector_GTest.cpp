#include <gtest/gtest.h>
#include "SwingDetector.hpp"

#include <string>
#include <vector>

TEST(test_swingdetector, checkinit_fires_on_first_sample)
{
    SwingDetector det;
    std::string received;
    det.setCallback([&](const char* level) { received = level; });

    det.detect(25.f);
    EXPECT_EQ(received, "Highest Duty Cycle");
}

TEST(test_swingdetector, checkno_callback_when_level_unchanged)
{
    SwingDetector det;
    int count = 0;
    det.setCallback([&](const char*) { ++count; });

    det.detect(25.f);
    det.detect(22.f);
    det.detect(21.f);
    EXPECT_EQ(count, 1);
}

TEST(test_swingdetector, checklevel_transitions_each_fire_callback)
{
    SwingDetector det;
    std::vector<std::string> levels;
    det.setCallback([&](const char* l) { levels.push_back(l); });

    det.detect(25.f);
    det.detect(12.f);
    det.detect(6.f);
    det.detect(1.f);

    EXPECT_EQ(levels.size(), 4u);
    EXPECT_EQ(levels[0], "Highest Duty Cycle");
    EXPECT_EQ(levels[1], "Medium Duty Cycle");
    EXPECT_EQ(levels[2], "Low Duty Cycle");
    EXPECT_EQ(levels[3], "No Duty Cycle");
}

TEST(test_swingdetector, checkreset_allows_same_level_to_refire)
{
    SwingDetector det;
    int count = 0;
    det.setCallback([&](const char*) { ++count; });

    det.detect(25.f);
    det.detect(25.f);
    det.reset();
    det.detect(25.f);

    EXPECT_EQ(count, 2);
}

TEST(test_swingdetector, checkthreshold_boundary_values)
{
    SwingDetector det;
    std::string last;
    det.setCallback([&](const char* l) { last = l; });

    det.detect(5.0f);  EXPECT_EQ(last, "Low Duty Cycle");
    det.detect(4.9f);  EXPECT_EQ(last, "No Duty Cycle");

    det.reset(); det.detect(9.9f);  EXPECT_EQ(last, "Low Duty Cycle");
    det.reset(); det.detect(10.0f); EXPECT_EQ(last, "Medium Duty Cycle");
    det.reset(); det.detect(19.9f); EXPECT_EQ(last, "Medium Duty Cycle");
    det.reset(); det.detect(20.0f); EXPECT_EQ(last, "Highest Duty Cycle");
}
