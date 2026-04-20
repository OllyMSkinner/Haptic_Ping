<<<<<<< HEAD
/*
    This header declares a simple software PWM class for the Raspberry Pi.
    It controls a GPIO output using a timed loop, allows the duty cycle
    to be updated, and manages the resources needed to run the PWM signal.
*/
=======
/**  This class uses a dedicated worker thread and libgpiod to provide
  *  a software PWM output on a Raspberry Pi GPIO pin. Its main
  *  responsibility is to provide a safe interface for starting and
  *  stopping PWM generation, allow client code to update the duty cycle
  *  without needing to manage any GPIO or timing details, and internally
  *  handle the GPIO line request and timing loop. 
  *
  *
  * SOLID principles:
  *   S - this class his only responsible for generating PWM on one GPIO line. Higher
  *       level classification decides when feedback should happen and why the duty cycle changes. 
  *   O - Client code can reuse this class for different duty cycle values without the 
  *       internal logic being modified itself. This behaviour is extended by calling its public interface rather than rewriting the timing logic. */

>>>>>>> BeforeSubmission

#pragma once

#include <atomic>
#include <chrono>
#include <gpiod.h>
#include <iostream>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>

<<<<<<< HEAD
// Defines the GPIO line and PWM frequency used by the software PWM output.
=======
/// Fixed hardware/software config forPWM output
/// Keeping these constant avoids mageic numbers and makes the class behaviour easier to maintain and reason with.
>>>>>>> BeforeSubmission
static constexpr unsigned int PWM_GPIO_LINE = 18;
static constexpr int PWM_FREQ_HZ = 30;

// Declares the PWM class and its main functions for starting, stopping, and updating the duty cycle.
class RPI_PWM {
public:

    /**
    * Initialises the PWM class to a safe default state with
    * no active output or GPIO resources in use. */
    RPI_PWM() : duty_(0.0f), running_(false), chip_(nullptr), request_(nullptr) {}

    /**
      *  Sets up the GPIO output and starts the PWM worker thread.
      *  Returns true if the PWM starts successfully. */

    bool start() {
        /// Open the GPIO chip used for the PWM output pin.
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!chip_) return false;

        /// Create and configure the GPIO line as an output, starting inactive.
        gpiod_line_settings* settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        /// Apply the line settings to the chosen PWM GPIO line.
        gpiod_line_config* line_cfg = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(line_cfg, &PWM_GPIO_LINE, 1, settings);

        /// Create the request configuration and name this GPIO user.
        gpiod_request_config* req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "soft_pwm");

        /// Request control of the GPIO line.
        request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

        /// Free the temporary configuration objects after the request is made.
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);

        /// If the line request fails, close the chip and report failure.
        if (!request_) {
            gpiod_chip_close(chip_);
            return false;
        }

        /// Mark the PWM engine as active and launch the worker thread.
        running_ = true;
        thread_ = std::thread(&RPI_PWM::loop, this);

        /// Print a message to confirm that PWM has started.
        std::cout << "Software PWM started on GPIO " << PWM_GPIO_LINE << "\n";
        return true;
    }

    /// Stops the PWM, turns the output off, and releases the GPIO resources.
        
    void stop() {
        /// Mark the PWM as no longer running.
        running_ = false;

        /// Wait for the worker thread to finish if it is active.
        if (thread_.joinable()) thread_.join();

        /// Set the GPIO output low, then release the line request.
        if (request_) {
            gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
            gpiod_line_request_release(request_);
            request_ = nullptr;
        }

        /// Close the GPIO chip and clear the stored pointer.
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    }

    /// Updates the PWM duty cycle and keeps it within a valid range.
         
    void setDutyCycle(float percent) {
        /// Clamp the value so it stays between 0% and 100%.
        if (percent < 0.0f)   percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;

        /// Store the new duty cycle value.
        duty_ = percent;
    }

    /// Returns the current duty cycle value.
    float getDutyCycle() const { return duty_.load(); }

    /// Stops the PWM automatically when the object is destroyed.
    ~RPI_PWM() { stop(); }

private:
<<<<<<< HEAD
    // Declares the helper functions and private members used for timing, GPIO control, thread handling, and PWM state.
=======
    /// Converts a time value in microseconds into a timer specification.
>>>>>>> BeforeSubmission
    static itimerspec usToTimerspec(long us) {
        itimerspec ts{};

        /// Store the whole seconds part of the delay.
        ts.it_value.tv_sec     = us / 1000000;

        /// Store the remaining time in nanoseconds.
        ts.it_value.tv_nsec    = (us % 1000000) * 1000;

        /// Disable repeating so the timer only runs once.
        ts.it_interval.tv_sec  = 0;
        ts.it_interval.tv_nsec = 0;
        return ts;
    }

    /// Waits for the given number of microseconds using a timer file descriptor.
        
    void waitUs(int tfd, int us) {
        /// Convert the delay into a timer format.
        itimerspec ts = usToTimerspec(us);

        /// Start the timer with the requested delay.
        timerfd_settime(tfd, 0, &ts, nullptr);

        /// Read from the timer so the function blocks until the timer expires.
        uint64_t exp;
        ::read(tfd, &exp, sizeof(exp));
    }

    /// Runs the PWM loop by switching the GPIO on and off based on the duty cycle.
    void loop() {
        /// Work out the full PWM period in microseconds.
        const int period_us = 1000000 / PWM_FREQ_HZ;

        /// Create a timer used for accurate PWM delays.
        int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd < 0) return;

        /// Keep generating PWM while the class is marked as running.
        while (running_) {
            // Read the current duty cycle.
            float d    = duty_.load();

            /// Calculate the on and off times for this PWM cycle.
            int on_us  = (int)(period_us * d / 100.0f);
            int off_us = period_us - on_us;

            /// Keep the output fully on for a 100% duty cycle.
            if (d >= 100.0f) {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_ACTIVE);
                waitUs(tfd, period_us);

            /// Keep the output fully off for a 0% duty cycle.
            } else if (d <= 0.0f) {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
                waitUs(tfd, period_us);

            /// Otherwise, switch on for part of the period and off for the rest.
            } else {
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_ACTIVE);
                waitUs(tfd, on_us);
                gpiod_line_request_set_value(request_, PWM_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
                waitUs(tfd, off_us);
            }
        }

        /// Close the timer once the loop finishes.
        ::close(tfd);
    }

    std::atomic<float>  duty_;
    std::atomic<bool>   running_;
    std::thread         thread_;
    gpiod_chip*         chip_;
    gpiod_line_request* request_;
};
