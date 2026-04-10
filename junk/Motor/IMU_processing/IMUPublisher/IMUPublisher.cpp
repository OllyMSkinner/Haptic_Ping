#include "IMUPublisher.hpp"

#include <iostream>
#include <utility>
#include <ctime>

#include <gpiod.h>

namespace icm20948
{
    IMUPublisher::IMUPublisher(unsigned i2c_bus,
                               unsigned i2c_address,
                               const char* gpiochip,
                               unsigned rdy_line)
        : imu_(i2c_bus, i2c_address),
          running_(false),
          gpiochip_path_(gpiochip),
          rdy_line_(rdy_line)
    {
    }

    IMUPublisher::~IMUPublisher()
    {
        stop();
    }

    bool IMUPublisher::init()
    {
        return imu_.init();
    }

    void IMUPublisher::registerCallback(SampleCallback cb)
    {
        callback_ = std::move(cb);
    }

    bool IMUPublisher::start()
    {
        if (running_) {
            return false;
        }

        running_ = true;
        worker_thread_ = std::thread(&IMUPublisher::worker, this);
        return true;
    }

    void IMUPublisher::stop()
    {
        running_ = false;

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void IMUPublisher::worker()
{
    gpiod_chip* chip = gpiod_chip_open(gpiochip_path_);
    if (!chip) {
        std::cerr << "[IMUPublisher] Failed to open GPIO chip: "
                  << gpiochip_path_ << "\n";
        running_ = false;
        return;
    }

    // --- Build line config: one line, rising-edge events ---
    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    unsigned int offset = static_cast<unsigned int>(rdy_line_);
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "imu-rdy");

    gpiod_line_request* request =
        gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    // Clean up config objects — no longer needed after request is made
    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!request) {
        std::cerr << "[IMUPublisher] Failed to request RDY GPIO line "
                  << rdy_line_ << "\n";
        gpiod_chip_close(chip);
        running_ = false;
        return;
    }

    std::cout << "[IMUPublisher] Waiting on RDY pin GPIO "
              << rdy_line_ << "\n";

    // Pre-allocate an edge event buffer (capacity 1 is fine here)
    gpiod_edge_event_buffer* buffer = gpiod_edge_event_buffer_new(1);

    while (running_) {
        // Timeout in nanoseconds: 1 second
        int rv = gpiod_line_request_wait_edge_events(request, 1'000'000'000LL);

        if (!running_) {
            break;
        }

        if (rv < 0) {
            std::cerr << "[IMUPublisher] Error while waiting for RDY event\n";
            break;
        }

        if (rv == 0) {
            // Timed out — loop back and check running_
            continue;
        }

        int n = gpiod_line_request_read_edge_events(request, buffer, 1);
        if (n < 0) {
            std::cerr << "[IMUPublisher] Failed to read GPIO event\n";
            continue;
        }

        for (int i = 0; i < n; ++i) {
            gpiod_edge_event* event = gpiod_edge_event_buffer_get_event(buffer, i);
            if (gpiod_edge_event_get_event_type(event) == GPIOD_EDGE_EVENT_RISING_EDGE) {
                IMUSample sample{};
                if (imu_.read_sample(sample)) {
                    if (callback_) {
                        callback_(sample);
                    }
                } else {
                    std::cerr << "[IMUPublisher] Failed to read IMU sample\n";
                }
            }
        }
    }

    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);
    }
}