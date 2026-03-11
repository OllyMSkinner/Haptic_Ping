#include "swing.h"

#include <cmath>
#include <sstream>

SwingDetector::SwingDetector()
    : in_swing_(false),
      have_prev_sample_(false),
      swing_start_ts_ms_(0),
      swing_end_ts_ms_(0),
      peak_gyro_mag_(0.0),
      gyro_sum_(0.0),
      gyro_energy_(0.0),
      peak_accel_delta_(0.0),
      sample_count_(0),
      // These are starting guesses. Tune them on real data.
      start_gyro_threshold_(1.2),
      end_gyro_threshold_(0.5),
      min_duration_ms_(80.0),
      max_duration_ms_(1200.0),
      min_peak_gyro_(2.0),
      min_gyro_energy_(120.0),
      min_peak_accel_delta_(0.8) {
}

void SwingDetector::reset() {
    in_swing_ = false;
    have_prev_sample_ = false;
    swing_start_ts_ms_ = 0;
    swing_end_ts_ms_ = 0;
    peak_gyro_mag_ = 0.0;
    gyro_sum_ = 0.0;
    gyro_energy_ = 0.0;
    peak_accel_delta_ = 0.0;
    sample_count_ = 0;
}

void SwingDetector::setStartGyroThreshold(double v) { start_gyro_threshold_ = v; }
void SwingDetector::setEndGyroThreshold(double v)   { end_gyro_threshold_ = v; }
void SwingDetector::setMinDurationMs(double v)      { min_duration_ms_ = v; }
void SwingDetector::setMaxDurationMs(double v)      { max_duration_ms_ = v; }
void SwingDetector::setMinPeakGyro(double v)        { min_peak_gyro_ = v; }
void SwingDetector::setMinGyroEnergy(double v)      { min_gyro_energy_ = v; }
void SwingDetector::setMinPeakAccelDelta(double v)  { min_peak_accel_delta_ = v; }

double SwingDetector::gyroMagnitude(const IMUSample& s) {
    return std::sqrt(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
}

double SwingDetector::accelMagnitude(const IMUSample& s) {
    return std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
}

SwingResult SwingDetector::makeNoResult() const {
    return SwingResult{};
}

SwingResult SwingDetector::update(const IMUSample& s) {
    const double gyro_mag = gyroMagnitude(s);

    double accel_delta = 0.0;
    if (have_prev_sample_) {
        const double prev_acc_mag = accelMagnitude(prev_sample_);
        const double curr_acc_mag = accelMagnitude(s);
        accel_delta = std::fabs(curr_acc_mag - prev_acc_mag);
    }

    if (!in_swing_) {
        if (gyro_mag >= start_gyro_threshold_) {
            in_swing_ = true;
            swing_start_ts_ms_ = s.ts_ms;
            start_sample_ = s;
            last_sample_ = s;

            peak_gyro_mag_ = gyro_mag;
            gyro_sum_ = gyro_mag;
            gyro_energy_ = gyro_mag * gyro_mag;
            peak_accel_delta_ = accel_delta;
            sample_count_ = 1;
        }

        prev_sample_ = s;
        have_prev_sample_ = true;
        return makeNoResult();
    }

    // We are inside a swing
    if (gyro_mag > peak_gyro_mag_) {
        peak_gyro_mag_ = gyro_mag;
    }
    if (accel_delta > peak_accel_delta_) {
        peak_accel_delta_ = accel_delta;
    }

    gyro_sum_ += gyro_mag;
    gyro_energy_ += gyro_mag * gyro_mag;
    sample_count_++;
    last_sample_ = s;

    // End swing when movement drops back down
    if (gyro_mag <= end_gyro_threshold_) {
        swing_end_ts_ms_ = s.ts_ms;

        prev_sample_ = s;
        have_prev_sample_ = true;
        return finalizeSwing();
    }

    prev_sample_ = s;
    have_prev_sample_ = true;
    return makeNoResult();
}

SwingResult SwingDetector::finalizeSwing() {
    SwingResult r;
    r.swing_finished = true;
    r.start_ts_ms = swing_start_ts_ms_;
    r.end_ts_ms = swing_end_ts_ms_;
    r.duration_ms = static_cast<double>(swing_end_ts_ms_ - swing_start_ts_ms_);
    r.peak_gyro_mag = peak_gyro_mag_;
    r.avg_gyro_mag = (sample_count_ > 0) ? (gyro_sum_ / static_cast<double>(sample_count_)) : 0.0;
    r.gyro_energy = gyro_energy_;
    r.peak_accel_delta = peak_accel_delta_;

    std::ostringstream why;
    bool good = true;

    if (r.duration_ms < min_duration_ms_) {
        good = false;
        why << "too short; ";
    }
    if (r.duration_ms > max_duration_ms_) {
        good = false;
        why << "too long; ";
    }
    if (r.peak_gyro_mag < min_peak_gyro_) {
        good = false;
        why << "peak gyro too low; ";
    }
    if (r.gyro_energy < min_gyro_energy_) {
        good = false;
        why << "gyro energy too low; ";
    }
    if (r.peak_accel_delta < min_peak_accel_delta_) {
        good = false;
        why << "acceleration change too small; ";
    }

    if (good) {
        r.label = SwingLabel::Good;
        r.reason = "swing matched thresholds";
    } else {
        r.label = SwingLabel::Bad;
        r.reason = why.str();
        if (r.reason.empty()) {
            r.reason = "did not match swing thresholds";
        }
    }

    // Reset state for next swing
    in_swing_ = false;
    swing_start_ts_ms_ = 0;
    swing_end_ts_ms_ = 0;
    peak_gyro_mag_ = 0.0;
    gyro_sum_ = 0.0;
    gyro_energy_ = 0.0;
    peak_accel_delta_ = 0.0;
    sample_count_ = 0;

    return r;
}