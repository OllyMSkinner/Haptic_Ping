#include "positiondetector.hpp"
#include "imureader.hpp"
#include "SwingProcessor.hpp"
#include "SwingDetector.hpp"
#include "swingfeedback.h"
#include "rpi_pwm.h"

#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <chrono>

#include "ads1115rpi.h"
#include "event_detector.h"
#include "led_controller.h"
#include "ledcallback.hpp"

static std::atomic<bool> running(true);

void handle_signal(int)
{
    running = false;
}

int main()
{
    signal(SIGINT, handle_signal);
    signal(SIGHUP, handle_signal);

    SimpleLEDController imuLed(22, 0);
    LEDController piezoLed(LEDControllerSettings{.chipNumber = 0, .greenGpio = 25});

    RPI_PWM pwm;
    if (!pwm.start()) return EXIT_FAILURE;
    pwm.setDutyCycle(0.0f);

    SwingDetector  swing_detector;
    SwingProcessor processor;
    SwingFeedback  feedback(pwm);

    swing_detector.setCallback([&](const char* level) {
        feedback.onLevel(level);
    });

    processor.setMagnitudeCallback([&](float mag) {
        swing_detector.detect(mag);
    });

    processor.setPositionCallback([&](bool upright) {
        imuLed.set(upright);
    });

    IMUReader reader(I2C_BUS);

    if (!reader.init()) {
        feedback.forceOff();
        pwm.setDutyCycle(0.0f);
        return EXIT_FAILURE;
    }

    PiezoEventDetector piezoDetector(piezoLed, PiezoEventDetectorSettings{});

    ADS1115rpi ads;
    try {
        ads.start(makeDefaultADS1115Settings(), false);
    } catch (...) {
        feedback.forceOff();
        pwm.setDutyCycle(0.0f);
        return EXIT_FAILURE;
    }

    feedback.setResetCallback([&]() {
        swing_detector.reset();
        processor.reset();
        piezoDetector.reset();
    });

    piezoDetector.setForceCallback([&](bool pressed) {
        if (!pressed) return;
        if (processor.isActive()) return;
        processor.onForceReady(true);
    });

    while (running)
    {
        icm20948::IMUSample sample{};
        if (reader.getIMU().read_sample(sample))
        {
            feedback.checkTimeout();

            processor(sample.ax, sample.ay, sample.az,
                      sample.gx, sample.gy, sample.gz,
                      sample.mx, sample.my);
        }

        int raw = ads.readOnce();
        if (raw >= 0)
        {
            float v = static_cast<float>(raw) / 32767.0f * ads.fullScaleVoltage();

            if (!processor.isActive()) {
                piezoDetector.processSample(v);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    ads.stop();
    reader.stop();
    feedback.forceOff();
    pwm.setDutyCycle(0.0f);

    return EXIT_SUCCESS;
}