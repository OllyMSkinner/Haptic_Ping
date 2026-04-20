// #include "positiondetector.hpp"
// #include "imureader.hpp"
// #include "SwingProcessor.hpp"
// #include "SwingDetector.hpp"
// #include "swingfeedback.h"
// #include "rpi_pwm.h"

// #include <cstdlib>
// #include <csignal>
// #include <unistd.h>
// #include <atomic>
// #include <thread>
// #include <chrono>

// #include "ads1115rpi.h"
// #include "event_detector.h"
// #include "led_controller.h"
// #include "ledcallback.hpp"

// static std::atomic<bool> running(true);

// void handle_signal(int)
// {
//     running = false;
// }

// int main()
// {
//     signal(SIGINT, handle_signal);
//     signal(SIGHUP, handle_signal);

//     SimpleLEDController imuLed(22, 0);
//     LEDController piezoLed(LEDControllerSettings{.chipNumber = 0, .greenGpio = 25});

//     RPI_PWM pwm;
//     if (!pwm.start()) return EXIT_FAILURE;
//     pwm.setDutyCycle(0.0f);

//     SwingDetector  swing_detector;
//     SwingProcessor processor;
//     SwingFeedback  feedback(pwm);

//     swing_detector.setCallback([&](const char* level) {
//         feedback.onLevel(level);
//     processor.setPositionCallback([&](bool upright) {
//         imuLed.set(upright);
//     });

//     /// Latency test for IMU
//     int16_t sample_rate_div = 0;        /// change this to test for latency
    
//     icm20948::settings imuSettings(
//         icm20948::accel_settings(sample_rate_div),                          /// 12-bit divider
//         icm20948::gyro_settings(static_cast<uint8_t>(sample_rate_div))      /// 8-bit divider
//     );

//     IMUReader reader(I2C_BUS, ICM20948_I2C_ADDR, "/dev/gpiochip0", 27, imuSettings);

//     if (!reader.init()) {
//         feedback.forceOff();
//         pwm.setDutyCycle(0.0f);
//         return EXIT_FAILURE;
//     }

//     PiezoEventDetector piezoDetector(piezoLed, PiezoEventDetectorSettings{});

//     ADS1115rpi ads;
//     try {
//         ads.start(makeDefaultADS1115Settings(), false);
//     } catch (...) {
//         feedback.forceOff();
//         pwm.setDutyCycle(0.0f);
//         return EXIT_FAILURE;
//     }

//     feedback.setResetCallback([&]() {
//         swing_detector.reset();
//         processor.reset();
//         piezoDetector.reset();
//     });

//     piezoDetector.setForceCallback([&](bool pressed) {
//         if (!pressed) return;
//         if (processor.isActive()) return;
//         processor.onForceReady(true);
//         (void)wasActive;
//     });

//     ADS1115rpi ads1115rpi;
//     ads1115rpi.registerCallback([&](float v) {
//         if (processor.isActive()) return;
//         piezoDetector.processSample(v);
//     });

//     // try {
//     //     ads1115rpi.start();
//     // } catch (const std::exception&) {}

//     /// Latency test for Piezo
//     ADS1115settings piezoSettings;
//     piezoSettings.samplingRate = ADS1115settings::FS128HZ;
//     // available rates: 8, 16, 32, 64, 128 (default), 250, 475, 860 

//     try {
//         ads1115rpi.start(piezoSettings);
//     } catch (const std::exception&) {}

//     sigset_t mask;
//     sigemptyset(&mask);
//     sigaddset(&mask, SIGINT);
//     sigaddset(&mask, SIGHUP);

//     if (sigprocmask(SIG_BLOCK, &mask, nullptr) == -1) {
//         reader.stop();
//         ads1115rpi.stop();
//         feedback.forceOff();
//         pwm.setDutyCycle(0.0f);
//         return EXIT_FAILURE;
//     }

//     int sfd = signalfd(-1, &mask, 0);
//     if (sfd == -1) {
//         reader.stop();
//         ads1115rpi.stop();
//         feedback.forceOff();
//         pwm.setDutyCycle(0.0f);
//         return EXIT_FAILURE;
//     }

//     bool running = true;
//     while (running) {
//         signalfd_siginfo fdsi;
//         ssize_t s = ::read(sfd, &fdsi, sizeof(fdsi));
//         if (s != static_cast<ssize_t>(sizeof(fdsi))) break;

//         if (fdsi.ssi_signo == SIGINT || fdsi.ssi_signo == SIGHUP) {
//             running = false;
//         }
//     }

//     ::close(sfd);
//     reader.stop();
//     feedback.forceOff();
//     pwm.setDutyCycle(0.0f);

//     return EXIT_SUCCESS;
// }

#include "positiondetector.hpp"
#include "imureader.hpp"
#include "SwingProcessor.hpp"
#include "SwingDetector.hpp"
#include "swingfeedback.h"
#include "rpi_pwm.h"

#include <iostream>
#include <cstdlib>
#include <csignal>
#include <sys/signalfd.h>
#include <unistd.h>

#include "ads1115rpi.h"
#include "event_detector.h"
#include "led_controller.h"
#include "ledcallback.hpp"

int main()
{
    SimpleLEDController imuLed(22, 0);
    LEDController piezoLed(LEDControllerSettings{.chipNumber = 0, .greenGpio = 25});

    RPI_PWM pwm;
    if (!pwm.start()) {
        return EXIT_FAILURE;
    }
    pwm.setDutyCycle(0.0f);

    SwingDetector  swing_detector;
    SwingProcessor processor;
    SwingFeedback  feedback(pwm);

    swing_detector.setCallback([&feedback](const char* level) {
        feedback.onLevel(level);
    });

    processor.setMagnitudeCallback([&swing_detector](float mag) {
        swing_detector.detect(mag);
    });

    processor.setPositionCallback([&imuLed](bool upright) {
        imuLed.set(upright);
    });

    /// Latency test for IMU
    int16_t sample_rate_div = 0;        /// change this to test for latency
    
    icm20948::settings imuSettings(
        icm20948::accel_settings(sample_rate_div),                          /// 12-bit divider
        icm20948::gyro_settings(static_cast<uint8_t>(sample_rate_div))      /// 8-bit divider
    );

    IMUReader reader(I2C_BUS, ICM20948_I2C_ADDR, "/dev/gpiochip0", 27, imuSettings);

    if (!reader.init()) {
        pwm.setDutyCycle(0.0f);
        return EXIT_FAILURE;
    }

    reader.setCallback([&](float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float mx, float my) {
        feedback.checkTimeout();
        processor(ax, ay, az, gx, gy, gz, mx, my);
    });

    reader.start();

    PiezoEventDetector piezoDetector(piezoLed, PiezoEventDetectorSettings{});

    feedback.setResetCallback([&processor, &swing_detector, &piezoDetector]() {
        swing_detector.reset();
        processor.reset();
        piezoDetector.reset();
    });

    piezoDetector.setForceCallback([&](bool pressed) {
        if (!pressed) return;
        if (processor.isActive()) return;
        const bool wasActive = processor.isActive();
        processor.onForceReady(true);
        (void)wasActive;
    });

    ADS1115rpi ads1115rpi;
    ads1115rpi.registerCallback([&](float v) {
        if (processor.isActive()) return;
        piezoDetector.processSample(v);
    });

    // try {
    //     ads1115rpi.start();
    // } catch (const std::exception&) {}

    /// Latency test for Piezo
    ADS1115settings piezoSettings;
    piezoSettings.samplingRate = ADS1115settings::FS128HZ;
    // available rates: 8, 16, 32, 64, 128 (default), 250, 475, 860 

    try {
        ads1115rpi.start(piezoSettings);
    } catch (const std::exception&) {}

    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGHUP);

    if (sigprocmask(SIG_BLOCK, &mask, nullptr) == -1) {
        reader.stop();
        ads1115rpi.stop();
        feedback.forceOff();
        pwm.setDutyCycle(0.0f);
        return EXIT_FAILURE;
    }

    int sfd = signalfd(-1, &mask, 0);
    if (sfd == -1) {
        reader.stop();
        ads1115rpi.stop();
        feedback.forceOff();
        pwm.setDutyCycle(0.0f);
        return EXIT_FAILURE;
    }

    bool running = true;
    while (running) {
        signalfd_siginfo fdsi;
        ssize_t s = ::read(sfd, &fdsi, sizeof(fdsi));
        if (s != static_cast<ssize_t>(sizeof(fdsi))) break;

        if (fdsi.ssi_signo == SIGINT || fdsi.ssi_signo == SIGHUP) {
            running = false;
        }
    }

    ::close(sfd);
    reader.stop();
    ads1115rpi.stop();
    feedback.forceOff();
    pwm.setDutyCycle(0.0f);

    return EXIT_SUCCESS;
}