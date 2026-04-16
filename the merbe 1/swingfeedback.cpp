#include "swingfeedback.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string_view>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace {

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

void playRank(RPI_PWM& pwm, int rank)
{
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd < 0) return;

    auto pulse = [&](int ms) {
        pwm.setDutyCycle(100.0f);
        waitMs(tfd, ms);
        pwm.setDutyCycle(0.0f);
    };

    if (rank == 1) {
        std::printf("[feedback] LOW pattern\n");
        pulse(180);
    } else if (rank == 2) {
        std::printf("[feedback] MEDIUM pattern\n");
        pulse(180);
        waitMs(tfd, 100);
        pulse(180);
    } else if (rank == 3) {
        std::printf("[feedback] HIGH pattern\n");
        pulse(450);
    }

    pwm.setDutyCycle(0.0f);
    ::close(tfd);
}

} // namespace

static int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

SwingFeedback::SwingFeedback(RPI_PWM& pwm)
    : pwm_(pwm)
{
    pwm_.setDutyCycle(0.0f);
}

void SwingFeedback::setResetCallback(ResetCallback cb)
{
    reset_cb_ = std::move(cb);
}

void SwingFeedback::checkTimeout()
{
    int64_t start = busy_since_ms_.load();
    if (start == 0) return;

    if (nowMs() - start > TIMEOUT_MS) {
        std::printf("[feedback] timeout — forcing reset\n");
        forceOff();
    }
}

void SwingFeedback::worker()
{
    if (busy_.exchange(true)) return;

    busy_since_ms_.store(nowMs());

    int rank = pending_level_.exchange(0);
    if (rank > 0) {
        playRank(pwm_, rank);
    }

    std::printf("[feedback] pattern done -> duty 0.000\n");

    busy_since_ms_.store(0);
    busy_ = false;

    if (reset_cb_) {
        reset_cb_();
    }
}

void SwingFeedback::onLevel(const char* level)
{
    if (!level) return;

    const int rank = levelRank(level);
    if (rank <= 0) return;

    int current = pending_level_.load();
    while (rank > current && !pending_level_.compare_exchange_weak(current, rank)) {
    }

    if (!busy_.load()) {
        std::thread(&SwingFeedback::worker, this).detach();
    }
}

void SwingFeedback::forceOff()
{
    busy_since_ms_.store(0);
    busy_.store(false);
    pwm_.setDutyCycle(0.0f);
    std::printf("[feedback] force off\n");
    if (reset_cb_) reset_cb_();
}