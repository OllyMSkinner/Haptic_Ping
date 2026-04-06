// #include "event_detector.h"
// #include "led_controller.h"
// #include <iostream>

// PiezoEventDetector::PiezoEventDetector(LEDController& ledRef,
//                                        float dipThreshold,
//                                        float peakThreshold,
//                                        int flashMs)
//     : led_(ledRef),
//       dipThreshold_(dipThreshold),
//       peakThreshold_(peakThreshold),
//       flashMs_(flashMs),
//       dipTriggered_(false),
//       peakTriggered_(false)
// {
// }

// void PiezoEventDetector::processSample(float v)
// {
//     std::cout << "processSample running, v = " << v << std::endl;

//     if (!dipTriggered_ && v <= dipThreshold_)
//     {
//         std::cout << "GREEN FLASH" << std::endl;
//         led_.flashGreen(flashMs_);
//         dipTriggered_ = true;
//         peakTriggered_ = false;
//     }
//     else if (dipTriggered_ && !peakTriggered_ && v >= peakThreshold_)
//     {
//         std::cout << "RED FLASH" << std::endl;
//         led_.flashRed(flashMs_);
//         peakTriggered_ = true;
//     }
//     else if (v > dipThreshold_ && v < peakThreshold_)
//     {
//         dipTriggered_ = false;
//         peakTriggered_ = false;
//     }
// }