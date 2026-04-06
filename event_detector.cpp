#include "event_detector.h"
#include <iostream>

PiezoEventDetector::PiezoEventDetector(float dipThreshold, float peakThreshold)
    : dipThreshold_(dipThreshold),
      peakThreshold_(peakThreshold),
      seenDip_(false)
{
    std::cout << "Detector initialised with dip=" << dipThreshold_
              << " peak=" << peakThreshold_ << std::endl;
}

void PiezoEventDetector::processSample(float v)
{
    // Dip detected (first event)
    if (!seenDip_ && v <= dipThreshold_)
    {
        std::cout << "GREEN" << std::endl;
        seenDip_ = true;
    }

    // Peak detected (second event)
    else if (seenDip_ && v >= peakThreshold_)
    {
        std::cout << "RED" << std::endl;
        seenDip_ = false;  // reset for next cycle
    }
}