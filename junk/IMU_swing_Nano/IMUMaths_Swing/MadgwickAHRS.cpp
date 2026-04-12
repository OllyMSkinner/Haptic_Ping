#include "MadgwickAHRS.hpp"

MadgwickAHRS::MadgwickAHRS(float beta)
    : q_est{1.f, 0.f, 0.f, 0.f}, _beta(beta)
{}

// -- private helpers ---------------------------------------------------------

struct quaternion MadgwickAHRS::quat_mult(struct quaternion L, struct quaternion R)
{
    struct quaternion p;
    p.q1 = (L.q1*R.q1) - (L.q2*R.q2) - (L.q3*R.q3) - (L.q4*R.q4);
    p.q2 = (L.q1*R.q2) + (L.q2*R.q1) + (L.q3*R.q4) - (L.q4*R.q3);
    p.q3 = (L.q1*R.q3) - (L.q2*R.q4) + (L.q3*R.q1) + (L.q4*R.q2);
    p.q4 = (L.q1*R.q4) + (L.q2*R.q3) - (L.q3*R.q2) + (L.q4*R.q1);
    return p;
}

void MadgwickAHRS::quat_normalize(struct quaternion* q)
{
    float norm = sqrtf(q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3 + q->q4*q->q4);
    q->q1 /= norm;
    q->q2 /= norm;
    q->q3 /= norm;
    q->q4 /= norm;
}

// -- public interface --------------------------------------------------------

// Follows Blake Johnson's implementation of Madgwick (2010).
// Equations referenced match the original paper.
void MadgwickAHRS::update(float ax, float ay, float az,
                           float gx, float gy, float gz, float dt)
{
    struct quaternion q_prev = q_est;

    // -- Gyroscope: rate of change of quaternion (eq. 12) --------------------
    struct quaternion q_w = { 0.f, gx, gy, gz };
    q_w.q1 *= 0.5f;  q_w.q2 *= 0.5f;
    q_w.q3 *= 0.5f;  q_w.q4 *= 0.5f;
    struct quaternion q_dot = quat_mult(q_prev, q_w);

    // -- Accelerometer: gradient-descent correction (eq. 15/25) --------------
    if (!((ax == 0.f) && (ay == 0.f) && (az == 0.f))) {

        struct quaternion q_a = { 0.f, ax, ay, az };
        quat_normalize(&q_a);  // normalise accelerometer to unit vector (eq. 24)

        // Objective function for gravity (eq. 25)
        float F_g[3];
        F_g[0] = 2.f*(q_prev.q2*q_prev.q4 - q_prev.q1*q_prev.q3) - q_a.q2;
        F_g[1] = 2.f*(q_prev.q1*q_prev.q2 + q_prev.q3*q_prev.q4) - q_a.q3;
        F_g[2] = 2.f*(0.5f - q_prev.q2*q_prev.q2 - q_prev.q3*q_prev.q3) - q_a.q4;

        // Jacobian for gravity (eq. 26)
        float J_g[3][4];
        J_g[0][0] = -2.f*q_prev.q3;  J_g[0][1] =  2.f*q_prev.q4;
        J_g[0][2] = -2.f*q_prev.q1;  J_g[0][3] =  2.f*q_prev.q2;

        J_g[1][0] =  2.f*q_prev.q2;  J_g[1][1] =  2.f*q_prev.q1;
        J_g[1][2] =  2.f*q_prev.q4;  J_g[1][3] =  2.f*q_prev.q3;

        J_g[2][0] =  0.f;            J_g[2][1] = -4.f*q_prev.q2;
        J_g[2][2] = -4.f*q_prev.q3;  J_g[2][3] =  0.f;

        // Gradient = J_g^T * F_g  (eq. 20)
        struct quaternion gradient;
        gradient.q1 = J_g[0][0]*F_g[0] + J_g[1][0]*F_g[1] + J_g[2][0]*F_g[2];
        gradient.q2 = J_g[0][1]*F_g[0] + J_g[1][1]*F_g[1] + J_g[2][1]*F_g[2];
        gradient.q3 = J_g[0][2]*F_g[0] + J_g[1][2]*F_g[1] + J_g[2][2]*F_g[2];
        gradient.q4 = J_g[0][3]*F_g[0] + J_g[1][3]*F_g[1] + J_g[2][3]*F_g[2];

        quat_normalize(&gradient);  // eq. 44

        // Fuse: subtract beta-scaled gradient from gyro rate (eq. 42–44)
        q_dot.q1 -= _beta * gradient.q1;
        q_dot.q2 -= _beta * gradient.q2;
        q_dot.q3 -= _beta * gradient.q3;
        q_dot.q4 -= _beta * gradient.q4;
    }

    // -- Integrate and normalise ---------------------------------------------
    q_est.q1 = q_prev.q1 + q_dot.q1 * dt;
    q_est.q2 = q_prev.q2 + q_dot.q2 * dt;
    q_est.q3 = q_prev.q3 + q_dot.q3 * dt;
    q_est.q4 = q_prev.q4 + q_dot.q4 * dt;
    quat_normalize(&q_est);
}

void MadgwickAHRS::gravity(float& gvx, float& gvy, float& gvz,
                            float g_magnitude) const
{
    // Estimated gravity direction in sensor frame from the objective function
    // (eq. 25 evaluated at current q_est, with F_g set to zero = converged).
    // q1=scalar, q2=x, q3=y, q4=z
    gvx = g_magnitude * 2.f * (q_est.q2*q_est.q4 - q_est.q1*q_est.q3);
    gvy = g_magnitude * 2.f * (q_est.q1*q_est.q2 + q_est.q3*q_est.q4);
    gvz = g_magnitude * 2.f * (0.5f - q_est.q2*q_est.q2 - q_est.q3*q_est.q3);
}
