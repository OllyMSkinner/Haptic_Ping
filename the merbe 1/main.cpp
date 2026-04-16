#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <gpiod.h>
#include <iostream>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

static constexpr unsigned int PWM_GPIO_LINE = 18;
static constexpr int DEFAULT_PWM_FREQ_HZ = 50;

class RPI_PWM {
public:
    RPI_PWM()
        : duty_(0.0f),
          freq_hz_(DEFAULT_PWM_FREQ_HZ),
          running_(false),
          chip_(nullptr),
          request_(nullptr),
          min_speed_(20.0f),   // tune these to your IMU units
          max_speed_(300.0f),
          min_duty_(20.0f),
          max_duty_(100.0f),
          min_freq_hz_(50.0f),
          max_freq_hz_(200.0f) {}

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
            chip_ = nullptr;
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

    void setFrequency(float hz) {
        if (hz < 1.0f) hz = 1.0f;
        freq_hz_ = hz;
    }

    float getDutyCycle() const { return duty_.load(); }
    float getFrequency() const { return freq_hz_.load(); }

    void configureSpeedMapping(float min_speed,
                               float max_speed,
                               float min_duty = 20.0f,
                               float max_duty = 100.0f,
                               float min_freq = 50.0f,
                               float max_freq = 200.0f) {
        if (max_speed <= min_speed) max_speed = min_speed + 1.0f;

        min_speed_   = min_speed;
        max_speed_   = max_speed;
        min_duty_    = clamp(min_duty, 0.0f, 100.0f);
        max_duty_    = clamp(max_duty, min_duty_, 100.0f);
        min_freq_hz_ = (min_freq < 1.0f) ? 1.0f : min_freq;
        max_freq_hz_ = (max_freq < min_freq_hz_) ? min_freq_hz_ : max_freq;
    }

    // Call this from your IMU code using a swing-speed value
    void setFromSwingSpeed(float speed) {
        if (speed < 0.0f) speed = -speed;

        float norm;
        if (speed <= min_speed_) {
            norm = 0.0f;
        } else if (speed >= max_speed_) {
            norm = 1.0f;
        } else {
            norm = (speed - min_speed_) / (max_speed_ - min_speed_);
        }

        // Linear mapping from speed -> duty/frequency
        float target_duty = min_duty_ + norm * (max_duty_ - min_duty_);
        float target_freq = min_freq_hz_ + norm * (max_freq_hz_ - min_freq_hz_);

        // If below threshold, turn motor off completely
        if (norm <= 0.0f) {
            target_duty = 0.0f;
        }

        setDutyCycle(target_duty);
        setFrequency(target_freq);
    }

    ~RPI_PWM() { stop(); }

private:
    static float clamp(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static itimerspec usToTimerspec(long us) {
        itimerspec ts{};
        ts.it_value.tv_sec     = us / 1000000;
        ts.it_value.tv_nsec    = (us % 1000000) * 1000;
        ts.it_interval.tv_sec  = 0;
        ts.it_interval.tv_nsec = 0;
        return ts;
    }

    void waitUs(int tfd, int us) {
        if (us <= 0) return;
        itimerspec ts = usToTimerspec(us);
        timerfd_settime(tfd, 0, &ts, nullptr);
        uint64_t exp;
        ::read(tfd, &exp, sizeof(exp));
    }

    void loop() {
        int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd < 0) return;

        while (running_) {
            float d = duty_.load();
            float f = freq_hz_.load();

            if (f < 1.0f) f = 1.0f;
            int period_us = static_cast<int>(1000000.0f / f);
            if (period_us < 1) period_us = 1;

            int on_us  = static_cast<int>(period_us * d / 100.0f);
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

    std::atomic<float> duty_;
    std::atomic<float> freq_hz_;
    std::atomic<bool>  running_;
    std::thread        thread_;
    gpiod_chip*        chip_;
    gpiod_line_request* request_;

    // Speed mapping parameters
    float min_speed_;
    float max_speed_;
    float min_duty_;
    float max_duty_;
    float min_freq_hz_;
    float max_freq_hz_;
};