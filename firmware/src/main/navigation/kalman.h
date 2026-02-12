/*
 * kalman.h
 *
 *  Created on: 2026. 2. 11.
 *      Author: 왕학승
 */

#ifndef SRC_MAIN_NAVIGATION_KALMAN_H_
#define SRC_MAIN_NAVIGATION_KALMAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw.h"
#include "common/time.h"
#include "common/axis.h"


#define STATE_DIM 6
#define MEAS_DIM  2

#define INNOVATION_GATE 5.99f   // Chi-square 95% (2 DOF)


/* ============================================================ */
/* ======================= STRUCT ============================== */
/* ============================================================ */

typedef struct
{
  float flowVel_x, flowVel_y;

  float x[STATE_DIM];
  float P[STATE_DIM][STATE_DIM];
  float Q[STATE_DIM][STATE_DIM];

  float R[MEAS_DIM];

  float accel_noise;      // accel white noise std (m/s^2)
  float bias_noise;       // bias random walk std (m/s^2)

} EKF_State;

void ekf_init(EKF_State *f);
void ekf_predict(EKF_State *f, float ax, float ay, float dt);
void ekf_update(EKF_State *f, float flow_vx, float flow_vy);


#ifdef __cplusplus
}
#endif
#endif /* SRC_MAIN_NAVIGATION_KALMAN_H_ */
