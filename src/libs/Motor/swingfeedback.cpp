    // This file implements the swing feedback class.
    // It converts feedback levels into PWM output patterns,
    // manages timing and time-outs, and resets the output when needed.

#include "swingfeedback.h"

#include <atomic>
#include <chrono>
#include <string_view>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace {

// Maximum duty cycle is capped at 80% to reduce peak back-EMF spike.
// Crashes were observed predominantly at 100% duty cycle and once at 66%,
// so 80% sits below the most problematic threshold while retaining
// meaningful haptic feedback strength.
// Defines the maximum duty cycle and helper functions used to rank levels, wait for timed delays, ramp the PWM output, and play each feedback pattern.

static constexpr float MAX_DUTY = 80.0f;

int levelRank(std::string_view level)
{
    if (level == "Low Duty Cycle")     return 1;
    if (level == "Medium Duty Cycle")  return 2;
    if (level == "Highest Duty Cycle") return 3;
    return 0;
}

void waitMs(int tfd, int ms)
{
    itimerspec ts{};
    ts.it_value.tv_sec  = ms / 1000;
    ts.it_value.tv_nsec = (ms % 1000) * 1000000L;
    timerfd_settime(tfd, 0, &ts, nullptr);
    uint64_t exp;
    ::read(tfd, &exp, sizeof(exp));
}


   // Ramp duty cycle up from 0 -> 33% -> 66% -> MAX_DUTY.
   // Each step uses a timerfd wait so the thread stays event-driven and non-blocking throughout.
   // This spreads the magnetic field build-up over ~90ms, reducing inrush current spikes.

void rampUp(RPI_PWM& pwm, int tfd)
{
    pwm.setDutyCycle(MAX_DUTY * 0.33f); waitMs(tfd, 30);
    pwm.setDutyCycle(MAX_DUTY * 0.66f); waitMs(tfd, 30);
    pwm.setDutyCycle(MAX_DUTY);         waitMs(tfd, 30);
}

   // Ramp duty cycle down from MAX_DUTY -> 66% -> 33% -> 0.
   // Spreading the field collapse over ~60ms significantly reduces the peak back-EMF spike that was causing the Pi to crash.

void rampDown(RPI_PWM& pwm, int tfd)
{
    pwm.setDutyCycle(MAX_DUTY * 0.66f); waitMs(tfd, 30);
    pwm.setDutyCycle(MAX_DUTY * 0.33f); waitMs(tfd, 30);
    pwm.setDutyCycle(0.0f);
}

void playRank(RPI_PWM& pwm, int rank)
{
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd < 0) return;

    if (rank == 1) {
        rampUp(pwm, tfd);
        waitMs(tfd, 180);
        rampDown(pwm, tfd);
    } else if (rank == 2) {
        rampUp(pwm, tfd);
        waitMs(tfd, 180);
        rampDown(pwm, tfd);
        waitMs(tfd, 100);
        rampUp(pwm, tfd);
        waitMs(tfd, 180);
        rampDown(pwm, tfd);
    } else if (rank == 3) {
        rampUp(pwm, tfd);
        waitMs(tfd, 450);
        rampDown(pwm, tfd);
    }

    pwm.setDutyCycle(0.0f);
    ::close(tfd);
}

} 

// Returns the current time in milliseconds for timeout tracking.
static int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Initialises the feedback object and sets the PWM output to off.
SwingFeedback::SwingFeedback(RPI_PWM& pwm)
    : pwm_(pwm)
{
    pwm_.setDutyCycle(0.0f);
}

// Stores the callback used when the feedback state is reset.
void SwingFeedback::setResetCallback(ResetCallback cb)
{
    reset_cb_ = std::move(cb);
}
// Checks whether the feedback has been active for too long and forces it off if needed.
void SwingFeedback::checkTimeout()
{
    int64_t start = busy_since_ms_.load();
    if (start == 0) return;

    if (nowMs() - start > TIMEOUT_MS) {
        forceOff();
    }
}

// Runs the feedback output, updates the busy state, and calls the reset callback when finished.
void SwingFeedback::worker()
{
    if (busy_.exchange(true)) return;

    busy_since_ms_.store(nowMs());

    int rank = pending_level_.exchange(0);
    if (rank > 0) {
        playRank(pwm_, rank);
    }

    busy_since_ms_.store(0);
    busy_ = false;

    if (reset_cb_) {
        reset_cb_();
    }

    // Clear any level that accumulated during playback so it doesn't
    // contaminate the next swing.
    pending_level_.store(0);
}

// Receives a feedback level, stores the highest pending level, and starts the worker if needed.
void SwingFeedback::onLevel(const char* level)
{
    if (!level) return;

    const int rank = levelRank(level);

    if (rank <= 0) {
        // Level dropped to zero — swing has ended. Fire the worker now so it
        // plays the actual peak that was accumulated, not just the first edge.
        if (!busy_.load() && pending_level_.load() > 0) {
            std::thread(&SwingFeedback::worker, this).detach();
        }
        return;
    }

    // Accumulate the highest level reached during the swing.
    // Do NOT spawn the worker yet — wait for the swing to end (rank == 0 above).
    int current = pending_level_.load();
    while (rank > current && !pending_level_.compare_exchange_weak(current, rank)) {
    }
}

// Forces the PWM output off, clears the busy state, and triggers the reset callback.
void SwingFeedback::forceOff()
{
    busy_since_ms_.store(0);
    busy_.store(false);
    pending_level_.store(0);
    pwm_.setDutyCycle(0.0f);
    if (reset_cb_) reset_cb_();
}
