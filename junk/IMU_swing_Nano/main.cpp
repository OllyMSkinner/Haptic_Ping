//  main.cpp – IMU swing reader with interrupt-driven (non-polling) sampling
//
//  Data flow (no busy-waiting anywhere):
//
//    ICM-20948 hardware
//        │  pulses GPIO 27 (data-ready) on every new sample
//        ▼
//    epoll_wait()  ← background thread sleeps here; woken by the GPIO edge
//        │  reads accel from the driver
//        ▼
//    linear_accel = raw_accel − static_gravity_bias   ← subtracted once at startup
//        ▼
//    accel_mag = |linear_accel|  →  printf
//
//  The main thread sleeps in read(signalfd) and only wakes on SIGINT/SIGHUP.

#include "IMU_library/Icm20948driver.hpp"
#include "positiondetector.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/signalfd.h>
#include <unistd.h>
#include <gpiod.h>

// ── tunables ──────────────────────────────────────────────────────────────────
static constexpr unsigned    I2C_BUS        = 1;
static constexpr const char* GPIO_CHIP      = "/dev/gpiochip0";
static constexpr unsigned    RDY_LINE       = 27;   // ICM-20948 INT1 → Pi GPIO 27
static constexpr int         CALIB_SAMPLES  = 200;  // samples at startup for initial bias
static constexpr int         RECAL_SAMPLES  = 50;   // samples when start pos confirmed
static constexpr float       ACCEL_DEADBAND = 0.05f;// m/s² – print 0 below this
// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    // ── initialise the IMU driver (magnetometer disabled) ─────────────────
    icm20948::settings imu_settings;
    imu_settings.magn.mode = icm20948::MAGN_SHUTDOWN;
    icm20948::ICM20948_I2C imu(I2C_BUS, ICM20948_I2C_ADDR, imu_settings);
    if (!imu.init()) {
        std::fprintf(stderr, "ERROR: IMU init failed\n");
        return EXIT_FAILURE;
    }

    // ── set up GPIO data-ready line (rising-edge, interrupt-driven) ────────
    gpiod_chip* chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        std::fprintf(stderr, "ERROR: cannot open %s\n", GPIO_CHIP);
        return EXIT_FAILURE;
    }

    gpiod_line_settings* line_cfg_s = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(line_cfg_s, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(line_cfg_s, GPIOD_LINE_EDGE_RISING);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    unsigned int offset = RDY_LINE;
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, line_cfg_s);

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "imu-rdy");

    gpiod_line_request* request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(line_cfg_s);

    if (!request) {
        std::fprintf(stderr, "ERROR: cannot request GPIO line %u\n", RDY_LINE);
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }

    int gpio_fd = gpiod_line_request_get_fd(request);

    // ── eventfd used to wake the worker thread cleanly on shutdown ─────────
    int stop_fd = eventfd(0, EFD_NONBLOCK);

    // ── background worker thread – sleeps in epoll, never polls ───────────
    std::atomic_bool running{true};

    std::thread worker([&]() {
        int ep = epoll_create1(0);

        epoll_event gpio_ev{};
        gpio_ev.events  = EPOLLIN | EPOLLET;
        gpio_ev.data.fd = gpio_fd;
        epoll_ctl(ep, EPOLL_CTL_ADD, gpio_fd, &gpio_ev);

        epoll_event stop_ev{};
        stop_ev.events  = EPOLLIN;
        stop_ev.data.fd = stop_fd;
        epoll_ctl(ep, EPOLL_CTL_ADD, stop_fd, &stop_ev);

        gpiod_edge_event_buffer* buf = gpiod_edge_event_buffer_new(1);

        // ── Phase 1: calibration – average CALIB_SAMPLES accel readings ──
        float bias_x = 0.f, bias_y = 0.f, bias_z = 0.f;
        int   calib_count = 0;
        bool  calibrated  = false;

        // ── Position detector – recalibrates bias at start position ───────
        PositionDetector detector;
        bool  needs_recal    = false;
        float recal_bias_x   = 0.f, recal_bias_y = 0.f, recal_bias_z = 0.f;
        int   recal_count    = 0;

        detector.setCallback([&](bool upright) {
            if (upright && !needs_recal) {
                needs_recal  = true;
                recal_bias_x = 0.f;
                recal_bias_y = 0.f;
                recal_bias_z = 0.f;
                recal_count  = 0;
                std::printf("Start position detected — recalibrating bias...\n");
            }
        });

        std::printf("Calibrating — keep device still for ~2 s...\n");

        while (running) {
            epoll_event fired{};
            int n = epoll_wait(ep, &fired, 1, -1);
            if (n <= 0 || !running) break;
            if (fired.data.fd == stop_fd) break;
            if (fired.data.fd != gpio_fd) continue;

            int got = gpiod_line_request_read_edge_events(request, buf, 1);
            if (got <= 0) continue;

            gpiod_edge_event* ev = gpiod_edge_event_buffer_get_event(buf, 0);
            if (gpiod_edge_event_get_event_type(ev) != GPIOD_EDGE_EVENT_RISING_EDGE)
                continue;

            if (!imu.read_accel_gyro()) continue;

            // ── Phase 1: initial bias calibration ─────────────────────────
            if (!calibrated) {
                bias_x += imu.accel[0];
                bias_y += imu.accel[1];
                bias_z += imu.accel[2];
                ++calib_count;
                if (calib_count == CALIB_SAMPLES) {
                    bias_x /= CALIB_SAMPLES;
                    bias_y /= CALIB_SAMPLES;
                    bias_z /= CALIB_SAMPLES;
                    calibrated = true;
                    std::printf("Ready — return to start position between swings to recalibrate.\n");
                }
                continue;
            }

            // ── Feed position detector (heading unused – pass 0,0) ────────
            detector.onSample(imu.accel[0], imu.accel[1], imu.accel[2], 0.f, 0.f);

            // ── Phase 2a: recalibration burst when start pos confirmed ─────
            if (needs_recal) {
                recal_bias_x += imu.accel[0];
                recal_bias_y += imu.accel[1];
                recal_bias_z += imu.accel[2];
                ++recal_count;
                if (recal_count == RECAL_SAMPLES) {
                    bias_x = recal_bias_x / RECAL_SAMPLES;
                    bias_y = recal_bias_y / RECAL_SAMPLES;
                    bias_z = recal_bias_z / RECAL_SAMPLES;
                    needs_recal = false;
                    std::printf("Bias updated.\n");
                }
                continue;  // skip output during recalibration
            }

            // ── Phase 2b: subtract bias, compute magnitude, print ─────────
            float lin_ax = imu.accel[0] - bias_x;
            float lin_ay = imu.accel[1] - bias_y;
            float lin_az = imu.accel[2] - bias_z;

            float lin_mag = std::sqrt(lin_ax*lin_ax + lin_ay*lin_ay + lin_az*lin_az);
            if (lin_mag < ACCEL_DEADBAND) lin_mag = 0.f;

            float gyro_mag = std::sqrt(imu.gyro[0]*imu.gyro[0] +
                                       imu.gyro[1]*imu.gyro[1] +
                                       imu.gyro[2]*imu.gyro[2]);

            std::printf("accel: %8.4f m/s²  |  gyro: %8.4f rad/s\n",
                        lin_mag, gyro_mag);
        }

        gpiod_edge_event_buffer_free(buf);
        close(ep);
    });

    // ── main thread waits for SIGINT / SIGHUP (no spin, no sleep loop) ────
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGHUP);
    sigprocmask(SIG_BLOCK, &mask, nullptr);

    int sfd = signalfd(-1, &mask, 0);

    signalfd_siginfo fdsi;
    while (true) {
        ssize_t s = ::read(sfd, &fdsi, sizeof(fdsi));
        if (s != static_cast<ssize_t>(sizeof(fdsi))) break;
        if (fdsi.ssi_signo == SIGINT || fdsi.ssi_signo == SIGHUP) break;
    }

    // ── clean shutdown ─────────────────────────────────────────────────────
    running = false;
    uint64_t val = 1;
    ::write(stop_fd, &val, sizeof(val));  // wake the worker
    if (worker.joinable()) worker.join();

    ::close(sfd);
    ::close(stop_fd);
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);

    return EXIT_SUCCESS;
}
