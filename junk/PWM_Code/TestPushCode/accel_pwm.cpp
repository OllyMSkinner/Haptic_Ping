/**
 * accel_pwm.cpp
 *
 * Event-driven acceleration → PWM motor buzz controller.
 * Receives IMU samples via the existing IMUPublisher callback — no polling.
 *
 * Uses the magnitude of the acceleration vector (ax, ay, az) so it responds
 * to motion in any direction.
 *
 * Build (adjust include/lib paths to match your project):
 *
 * Requires rpi_pwm.h and IMUPublisher.hpp in your include path.
 */

#include "rpi_pwm.h"
#include "IMU_processing/IMUPublisher/IMUPublisher.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <string>
#include <unistd.h>

// ── Hardware configuration ────────────────────────────────────────────────────

// PWM channel: 2 = GPIO18, 3 = GPIO19
static constexpr int   PWM_CHANNEL  = 2;
// Vibration motor drive frequency in Hz
static constexpr float PWM_FREQ_HZ  = 200.0f;

// Acceleration magnitude range that maps to 0 % → 100 % duty cycle (in g).
// At rest the sensor reads ~1 g (gravity). Set MIN_G slightly above that
// so the motor only buzzes when there is actual motion.
static constexpr float ACCEL_MIN_G  = 9.81f;   // below this → 0 % (motor off)
static constexpr float ACCEL_MAX_G  = 19.62f;   // above this → 100 % (full buzz)

// ── Utility: acceleration magnitude → duty cycle % ───────────────────────────

/**
 * Computes the magnitude of the acceleration vector and maps it linearly
 * onto [0, 100] % between ACCEL_MIN_G and ACCEL_MAX_G.
 *
 *  |a| < MIN_G  →  0 %   (below threshold, motor silent)
 *  |a| > MAX_G  →  100 % (clamped, motor at full buzz)
 */
static float accelToDutyCycle(float ax, float ay, float az)
{
    const float magnitude = std::sqrt(ax*ax + ay*ay + az*az);
    const float clamped   = std::max(ACCEL_MIN_G,
                                     std::min(magnitude, ACCEL_MAX_G));
    return (clamped - ACCEL_MIN_G) / (ACCEL_MAX_G - ACCEL_MIN_G) * 100.0f;
}

// ── Signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
static void signalHandler(int) { g_running = false; }

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // I2C / GPIO defaults — same convention as your existing sender
    unsigned    bus      = 1;
    unsigned    addr     = ICM20948_I2C_ADDR;
    const char* gpiochip = "/dev/gpiochip0";
    unsigned    rdy_line = 27;

    if (argc >= 2) bus      = static_cast<unsigned>(std::stoul(argv[1]));
    if (argc >= 3) addr     = static_cast<unsigned>(std::stoul(argv[2], nullptr, 16));
    if (argc >= 4) gpiochip = argv[3];
    if (argc >= 5) rdy_line = static_cast<unsigned>(std::stoul(argv[4]));

    std::cout << "IMU → PWM motor buzz controller\n"
              << "  I2C bus=" << bus
              << " addr=0x" << std::hex << addr << std::dec
              << " GPIO=" << gpiochip << ":" << rdy_line << "\n"
              << "  PWM channel=" << PWM_CHANNEL
              << " freq=" << PWM_FREQ_HZ << " Hz\n"
              << "  Buzz range: " << ACCEL_MIN_G << " g – "
              << ACCEL_MAX_G << " g\n";

    // ── PWM setup ─────────────────────────────────────────────────────────
    RPI_PWM pwm;
    pwm.start(PWM_CHANNEL, PWM_FREQ_HZ);
    pwm.setDutyCycle(0.0f);
    std::cout << "PWM started on channel " << PWM_CHANNEL << "\n";

    // ── IMU setup ─────────────────────────────────────────────────────────
    icm20948::IMUPublisher imuPublisher(bus, addr, gpiochip, rdy_line);

    if (!imuPublisher.init()) {
        std::cerr << "ICM20948 init() failed\n";
        pwm.setDutyCycle(0.0f);
        return 1;
    }
    std::cout << "IMU init OK\n";

    // ── Event callback: fired by IMUPublisher on every new sample ─────────
    // This is the only place the PWM is updated — purely event-driven.
    imuPublisher.registerCallback(
        [&pwm](const icm20948::IMUSample& sample) {
            const float duty = accelToDutyCycle(sample.ax, sample.ay, sample.az);
            pwm.setDutyCycle(duty);

            // Optional: remove in production
            std::cout << "accel=("
                      << sample.ax << ", "
                      << sample.ay << ", "
                      << sample.az << ") g  →  duty="
                      << duty << " %\n";
        }
    );

    if (!imuPublisher.start()) {
        std::cerr << "IMUPublisher start() failed\n";
        pwm.setDutyCycle(0.0f);
        return 1;
    }

    std::cout << "Streaming — press Ctrl+C to stop\n";

    // Main thread blocks on pause() — no polling, no busy-wait
    while (g_running) {
        pause();
    }

    std::cout << "\nStopping...\n";
    imuPublisher.stop();
    pwm.setDutyCycle(0.0f);   // safe stop: motor off
    std::cout << "Done\n";

    return 0;
}
