#pragma once

#include <cstdint>
#include <string>

struct IMUSample {
    std::uint64_t ts_ms = 0;

    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;

    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;

    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
};

enum class SwingLabel {
    None,
    Good,
    Bad
};

struct SwingResult {
    SwingLabel label = SwingLabel::None;
    bool swing_finished = false;

    std::uint64_t start_ts_ms = 0;
    std::uint64_t end_ts_ms = 0;
    double duration_ms = 0.0;

    double peak_gyro_mag = 0.0;
    double avg_gyro_mag = 0.0;
    double gyro_energy = 0.0;
    double peak_accel_delta = 0.0;

    std::string reason;
};

class SwingDetector {
public:
    SwingDetector();

    // Call this for every new IMU sample.
    // Returns a completed result only when a swing has just finished.
    SwingResult update(const IMUSample& s);

    void reset();

    // Optional threshold tuning
    void setStartGyroThreshold(double v);
    void setEndGyroThreshold(double v);
    void setMinDurationMs(double v);
    void setMaxDurationMs(double v);
    void setMinPeakGyro(double v);
    void setMinGyroEnergy(double v);
    void setMinPeakAccelDelta(double v);

private:
    bool in_swing_;
    bool have_prev_sample_;

    IMUSample prev_sample_;
    IMUSample start_sample_;
    IMUSample last_sample_;

    std::uint64_t swing_start_ts_ms_;
    std::uint64_t swing_end_ts_ms_;

    double peak_gyro_mag_;
    double gyro_sum_;
    double gyro_energy_;
    double peak_accel_delta_;
    std::uint64_t sample_count_;

    // Tunable thresholds
    double start_gyro_threshold_;
    double end_gyro_threshold_;
    double min_duration_ms_;
    double max_duration_ms_;
    double min_peak_gyro_;
    double min_gyro_energy_;
    double min_peak_accel_delta_;

private:
    static double gyroMagnitude(const IMUSample& s);
    static double accelMagnitude(const IMUSample& s);

    SwingResult makeNoResult() const;
    SwingResult finalizeSwing();
};