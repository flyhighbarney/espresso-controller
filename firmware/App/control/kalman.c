#include "kalman.h"
#include <string.h>
#include <math.h>

static void mat_zero(float m[KF_NUM_STATES][KF_NUM_STATES])
{
    memset(m, 0, sizeof(float) * KF_NUM_STATES * KF_NUM_STATES);
}

void kalman_init(kalman_filter_t *kf, float dt)
{
    memset(kf, 0, sizeof(kalman_filter_t));
    kf->dt = dt;

    kf->x[0] = 0.0f;   // pressure starts at 0
    kf->x[1] = 0.0f;   // flow starts at 0
    kf->x[2] = 10.0f;  // initial puck resistance guess (bar·s/mL)

    // Initial covariance — high uncertainty on resistance
    mat_zero(kf->P);
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 1.0f;
    kf->P[2][2] = 50.0f;

    kalman_set_process_noise(kf, 0.1f, 0.05f, 0.5f);
    kalman_set_measurement_noise(kf, 0.2f, 0.1f);
}

void kalman_set_process_noise(kalman_filter_t *kf, float q_pressure, float q_flow, float q_resistance)
{
    mat_zero(kf->Q);
    kf->Q[0][0] = q_pressure;
    kf->Q[1][1] = q_flow;
    kf->Q[2][2] = q_resistance;
}

void kalman_set_measurement_noise(kalman_filter_t *kf, float r_pressure, float r_flow)
{
    memset(kf->R, 0, sizeof(kf->R));
    kf->R[0][0] = r_pressure;
    kf->R[1][1] = r_flow;
}

void kalman_predict(kalman_filter_t *kf, float pump_duty)
{
    /*
     * State transition model:
     *   pressure_dot = K_pump * pump_duty - flow * resistance  (pump builds pressure, puck drops it)
     *   flow_dot     = (pressure - flow * resistance) / inertia
     *   resistance_dot = 0 (random walk — the Kalman tracks drift)
     *
     * Linearized for the predict step with constants tuned to the plant.
     */
    static const float K_PUMP = 12.0f;     // bar per unit duty at full flow
    static const float INERTIA = 0.5f;     // hydraulic inertia (damping constant)

    float p = kf->x[0];
    float f = kf->x[1];
    float r = kf->x[2];
    float dt = kf->dt;

    float p_new = p + dt * (K_PUMP * pump_duty - f * r);
    float f_new = f + dt * (p - f * r) / INERTIA;
    float r_new = r; // random walk

    if (p_new < 0.0f) p_new = 0.0f;
    if (f_new < 0.0f) f_new = 0.0f;
    if (r_new < 0.1f) r_new = 0.1f;

    kf->x[0] = p_new;
    kf->x[1] = f_new;
    kf->x[2] = r_new;

    // Jacobian F = dF/dx (linearized around current state)
    float F[KF_NUM_STATES][KF_NUM_STATES];
    F[0][0] = 1.0f;
    F[0][1] = -r * dt;
    F[0][2] = -f * dt;
    F[1][0] = dt / INERTIA;
    F[1][1] = 1.0f - r * dt / INERTIA;
    F[1][2] = -f * dt / INERTIA;
    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;

    // P = F * P * F' + Q
    float FP[KF_NUM_STATES][KF_NUM_STATES];
    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_STATES; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < KF_NUM_STATES; k++)
                FP[i][j] += F[i][k] * kf->P[k][j];
        }

    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_STATES; j++) {
            kf->P[i][j] = kf->Q[i][j];
            for (int k = 0; k < KF_NUM_STATES; k++)
                kf->P[i][j] += FP[i][k] * F[j][k]; // F[j][k] = F transposed
        }
}

void kalman_update(kalman_filter_t *kf, float z_pressure, float z_flow)
{
    // Measurement model: H = [[1, 0, 0], [0, 1, 0]]
    // Innovation: y = z - H*x
    float y[KF_NUM_MEAS];
    y[0] = z_pressure - kf->x[0];
    y[1] = z_flow - kf->x[1];

    // S = H * P * H' + R  (2x2, since H picks rows 0,1 of P)
    float S[KF_NUM_MEAS][KF_NUM_MEAS];
    S[0][0] = kf->P[0][0] + kf->R[0][0];
    S[0][1] = kf->P[0][1];
    S[1][0] = kf->P[1][0];
    S[1][1] = kf->P[1][1] + kf->R[1][1];

    // Invert 2x2: S_inv = adj(S) / det(S)
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    // V-14 fix: use a larger threshold to prevent numerical instability
    if (det < 1e-4f && det > -1e-4f)
        return; // near-singular — skip update

    float inv_det = 1.0f / det;
    float S_inv[KF_NUM_MEAS][KF_NUM_MEAS];
    S_inv[0][0] =  S[1][1] * inv_det;
    S_inv[0][1] = -S[0][1] * inv_det;
    S_inv[1][0] = -S[1][0] * inv_det;
    S_inv[1][1] =  S[0][0] * inv_det;

    // K = P * H' * S_inv  (3x2)
    // P * H' picks columns 0,1 of P
    float K[KF_NUM_STATES][KF_NUM_MEAS];
    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_MEAS; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < KF_NUM_MEAS; k++)
                K[i][j] += kf->P[i][k] * S_inv[k][j];
        }

    // x = x + K * y
    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_MEAS; j++)
            kf->x[i] += K[i][j] * y[j];

    // Enforce physical bounds
    if (kf->x[0] < 0.0f) kf->x[0] = 0.0f;
    if (kf->x[1] < 0.0f) kf->x[1] = 0.0f;
    if (kf->x[2] < 0.1f) kf->x[2] = 0.1f;

    // P = (I - K*H) * P
    float KH[KF_NUM_STATES][KF_NUM_STATES];
    mat_zero(KH);
    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_MEAS; j++)
            KH[i][j] += K[i][j]; // H is identity for first 2 rows

    float P_new[KF_NUM_STATES][KF_NUM_STATES];
    for (int i = 0; i < KF_NUM_STATES; i++)
        for (int j = 0; j < KF_NUM_STATES; j++) {
            P_new[i][j] = 0.0f;
            for (int k = 0; k < KF_NUM_STATES; k++)
                P_new[i][j] += ((i == k ? 1.0f : 0.0f) - KH[i][k]) * kf->P[k][j];
        }
    memcpy(kf->P, P_new, sizeof(kf->P));
}

float kalman_get_pressure(const kalman_filter_t *kf)   { return kf->x[0]; }
float kalman_get_flow(const kalman_filter_t *kf)       { return kf->x[1]; }
float kalman_get_resistance(const kalman_filter_t *kf) { return kf->x[2]; }

void kalman_reset(kalman_filter_t *kf)
{
    float dt = kf->dt;
    kalman_init(kf, dt);
}
