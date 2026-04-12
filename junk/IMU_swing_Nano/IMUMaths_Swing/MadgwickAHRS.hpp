#pragma once

#include <math.h>

// Madgwick AHRS filter – IMU variant (accelerometer + gyroscope, no magnetometer).
//
// Based on Blake Johnson's implementation of Madgwick (2010).
// Quaternion convention: q1 = scalar (real), q2/q3/q4 = imaginary (x, y, z).

#ifndef GYRO_MEAN_ERROR
    #define GYRO_MEAN_ERROR (3.14159265f * (5.0f / 180.0f))  // 5 deg/s in rad/s
#endif

#ifndef BETA
    #define BETA (sqrtf(3.0f / 4.0f) * GYRO_MEAN_ERROR)
#endif

struct quaternion {
    float q1;  // scalar (real)
    float q2;  // x
    float q3;  // y
    float q4;  // z
};

class MadgwickAHRS {
public:
    struct quaternion q_est;  // current orientation estimate

    explicit MadgwickAHRS(float beta = BETA);

    // Feed one sample.
    //   ax/ay/az – acceleration in any unit (normalised internally)
    //   gx/gy/gz – angular velocity in rad/s
    //   dt       – elapsed time since last call, in seconds
    void update(float ax, float ay, float az,
                float gx, float gy, float gz, float dt);

    // Returns the gravity vector in the sensor frame (m/s²).
    // Subtract this from raw accel readings to get linear acceleration.
    void gravity(float& gx, float& gy, float& gz,
                 float g_magnitude = 9.80665f) const;

private:
    float _beta;

    static struct quaternion quat_mult(struct quaternion L, struct quaternion R);
    static void quat_normalize(struct quaternion* q);
};
