/*
    This header declares a simple software PWM class for the Raspberry Pi.
    It controls a GPIO output using a timed loop, allows the duty cycle
    to be updated, and manages the resources needed to run the PWM signal.
*/

#pragma once

#include <atomic>
#include <chrono>
#include <gpiod.h>
#include <iostream>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

// Defines the GPIO line and PWM frequency used by the software PWM output.
static constexpr unsigned int PWM_GPIO_LINE = 18;
static constexpr int PWM_FREQ_HZ = 30;

// Declares the PWM class and its main functions for starting, stopping, and updating the duty cycle.
class RPI_PWM {
public:
    RPI_PWM() : duty_(0.0f), running_(false), chip_(nullptr), request_(nullptr) {}

    bool start() {
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!chip_) return false;

        gpiod_line_settings* settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config* line_cfg = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(line_cfg, &PWM_GPIO_LINE, 1, settings);

        gpiod_request_config* req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "soft_pwm");

        request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);

        if (!request_) {
            gpiod_chip_close(chip_);
            return false;
        }

        running_ = true;
        thread_ = std::thread(&RPI_PWM::loop, this);
        std::cout << "Software PWM started on GPIO " << PWM_GPIO_LINE << "\n";
        return true;
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (request_) {
            gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
            gpiod_line_request_release(request_);
            request_ = nullptr;
        }
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    }

    void setDutyCycle(float percent) {
        if (percent < 0.0f)   percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        duty_ = percent;
    }

    float getDutyCycle() const { return duty_.load(); }

    ~RPI_PWM() { stop(); }

private:
    // Declares the helper functions and private members used for timing, GPIO control, thread handling, and PWM state.
    static itimerspec usToTimerspec(long us) {
        itimerspec ts{};
        ts.it_value.tv_sec     = us / 1000000;
        ts.it_value.tv_nsec    = (us % 1000000) * 1000;
        ts.it_interval.tv_sec  = 0;
        ts.it_interval.tv_nsec = 0;
        return ts;
    }

    void waitUs(int tfd, int us) {
        itimerspec ts = usToTimerspec(us);
        timerfd_settime(tfd, 0, &ts, nullptr);
        uint64_t exp;
        ::read(tfd, &exp, sizeof(exp));
    }

    void loop() {
        const int period_us = 1000000 / PWM_FREQ_HZ;

        int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd < 0) return;

        while (running_) {
            float d    = duty_.load();
            int on_us  = (int)(period_us * d / 100.0f);
            int off_us = period_us - on_us;

            if (d >= 100.0f) {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_ACTIVE);
                waitUs(tfd, period_us);
            } else if (d <= 0.0f) {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
                waitUs(tfd, period_us);
            } else {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_ACTIVE);
                waitUs(tfd, on_us);
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
                waitUs(tfd, off_us);
            }
        }

        ::close(tfd);
    }

    std::atomic<float>  duty_;
    std::atomic<bool>   running_;
    std::thread         thread_;
    gpiod_chip*         chip_;
    gpiod_line_request* request_;
};
