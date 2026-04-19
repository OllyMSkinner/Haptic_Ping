#pragma once

#include <atomic>
#include <chrono>
#include <gpiod.h>
#include <iostream>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

//Fixed hardware/software config forPWM output
//Keeping these constant avoids mageic numbers and 
//makes the class behaviour easier to maintain and reason with.
static constexpr unsigned int PWM_GPIO_LINE = 18;
static constexpr int PWM_FREQ_HZ = 30;

/*
 * RPI_PWM
 * -------
 * This class uses a dedicated worker thread and lipgiod to provide 
 * a software PWM output on the Rapsberry Pi GPIO pin.
 * 
 * Main Responsibility:
 *  - provides a safe interface to start and stop PWM generation
 *  - to allow the client code to update the duty cycle without needing to know 
 *     any GPIO or timing details.
 *  - manage internally the GPIO line request and time loop.
 *  
 * SOLID principles:
 *  - S - this class his only responsible for generating PWM on one GPIO line. Higher
 *   level classification decides when feedback should happen and why the duty cycle changes.
 * 
 *  - O - Client code can reuse this class for different duty cycle values without the 
 *        internal logic being modified itself. This behaviour is extended by calling its public interface rather than rewriting the timing logic 
 *        
 *
 *  Realtime design: PWM generation runs in its own thread and uses timerfd-based waits rather than a crude busy loop, 
 *  giving more predictable timing.
 *
 * Encapsulation: all GPIO handles, thread state, and timing details are private.
 *
 * Reliability and resource management: resources are acquired in start() and released in stop()/destructor().
 */
class RPI_PWM {
public:
 /*
     * Constructor
     * -----------
     * Initialises the class into a safe default state:
     * - 0% duty cycle
     * - not running
     * - no GPIO chip or line request acquired yet
     *
     * (This is safer than leaving pointers uninitialised).
     */
    RPI_PWM() : duty_(0.0f), running_(false), chip_(nullptr), request_(nullptr) {}
/*
 * start()
 * -------
 * Initialises the GPIO line for output and starts the PWM worker thread.
 *
 * Returns true if setup succeeds, or false if the GPIO cannot be configured.
 *
 * All low-level GPIO setup is handled internally so the caller only needs
 * to start the PWM without dealing with libgpiod directly.
 */
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
        // Mark the PWM engine as active and launch the worker thread.
        running_ = true;
        thread_ = std::thread(&RPI_PWM::loop, this);
        std::cout << "Software PWM started on GPIO " << PWM_GPIO_LINE << "\n";
        return true;
    }

/*
     * stop()
     * ------
     * Safely stops PWM generation, joins the worker thread,
     * sets the output low, and releases GPIO resources.
     */
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

    /*
     * getDutyCycle()
     * --------------
     * Returns the current duty cycle request.
     *
     * A getter is provided instead of exposing the variable publicly.
     */
    float getDutyCycle() const { return duty_.load(); }
    /*
     * Destructor
     * ----------
     * Ensures that resources are released automatically if the object
     * goes out of scope.
     */

    ~RPI_PWM() { stop(); }

private:
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
