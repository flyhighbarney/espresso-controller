#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

/*
 * 3-state Kalman filter for espresso extraction:
 *   x[0] = pressure (bar)
 *   x[1] = flow rate (mL/s)
 *   x[2] = puck hydraulic resistance (bar·s/mL)
 *
 * Measurements:
 *   z[0] = pressure sensor reading
 *   z[1] = flow sensor reading
 *
 * The puck resistance is not directly measured — it's estimated
 * from the relationship: pressure = flow * resistance
 */

#define KF_NUM_STATES   3
#define KF_NUM_MEAS     2

typedef struct {
    float x[KF_NUM_STATES];                             // state estimate
    float P[KF_NUM_STATES][KF_NUM_STATES];              // error covariance
    float Q[KF_NUM_STATES][KF_NUM_STATES];              // process noise
    float R[KF_NUM_MEAS][KF_NUM_MEAS];                  // measurement noise
    float dt;
} kalman_filter_t;

void kalman_init(kalman_filter_t *kf, float dt);
void kalman_set_process_noise(kalman_filter_t *kf, float q_pressure, float q_flow, float q_resistance);
void kalman_set_measurement_noise(kalman_filter_t *kf, float r_pressure, float r_flow);
void kalman_predict(kalman_filter_t *kf, float pump_duty);
void kalman_update(kalman_filter_t *kf, float z_pressure, float z_flow);
float kalman_get_pressure(const kalman_filter_t *kf);
float kalman_get_flow(const kalman_filter_t *kf);
float kalman_get_resistance(const kalman_filter_t *kf);
void kalman_reset(kalman_filter_t *kf);

#endif
