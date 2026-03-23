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

        gpiod_line* line = gpiod_chip_get_line(chip, static_cast<unsigned int>(rdy_line_));
        if (!line) {
            std::cerr << "[IMUPublisher] Failed to get RDY GPIO line "
                      << rdy_line_ << "\n";
            gpiod_chip_close(chip);
            running_ = false;
            return;
        }

        if (gpiod_line_request_rising_edge_events(line, "imu-rdy") < 0) {
            std::cerr << "[IMUPublisher] Failed to request rising-edge events on GPIO "
                      << rdy_line_ << "\n";
            gpiod_chip_close(chip);
            running_ = false;
            return;
        }

        std::cout << "[IMUPublisher] Waiting on RDY pin GPIO "
                  << rdy_line_ << "\n";

        while (running_) {
            timespec timeout{};
            timeout.tv_sec = 1;
            timeout.tv_nsec = 0;

            int rv = gpiod_line_event_wait(line, &timeout);

            if (!running_) {
                break;
            }

            if (rv < 0) {
                std::cerr << "[IMUPublisher] Error while waiting for RDY event\n";
                break;
            }

            if (rv == 0) {
                continue;
            }

            gpiod_line_event event{};
            if (gpiod_line_event_read(line, &event) < 0) {
                std::cerr << "[IMUPublisher] Failed to read GPIO event\n";
                continue;
            }

            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
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

        gpiod_line_release(line);
        gpiod_chip_close(chip);
    }
}