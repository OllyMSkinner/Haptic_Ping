#ifndef EVENT_DETECTOR_H
#define EVENT_DETECTOR_H

class PiezoEventDetector
{
public:
    PiezoEventDetector(float dipThreshold, float peakThreshold);

    void processSample(float v);

private:
    float dipThreshold_;
    float peakThreshold_;

    bool seenDip_;
};

#endif
