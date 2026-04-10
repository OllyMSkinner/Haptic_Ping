#ifndef EVENT_DETECTOR_H
#define EVENT_DETECTOR_H

class LEDController;

class PiezoEventDetector
{
public:
    PiezoEventDetector(LEDController& ledRef,
                       float dipThreshold,
                       float peakThreshold,
                       int flashMs = 100);

    void processSample(float v);

private:
    LEDController& led_;
    float dipThreshold_;
    float peakThreshold_;
    int flashMs_;

    bool dipTriggered_;
    bool peakTriggered_;
};

#endif
