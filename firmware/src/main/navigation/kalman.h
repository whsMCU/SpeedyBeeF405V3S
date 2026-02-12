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

#define STATE_DIM 4

// EKF 상태 변수 구조체
typedef struct {
    float flowVel_x, flowVel_y;

    float x[STATE_DIM];               // 상태: [0]=x, [1]=y, [2]=vx, [3]=vy
    float P[STATE_DIM][STATE_DIM];    // 오차 공분산 행렬
    float Q[STATE_DIM][STATE_DIM];    // 프로세스 노이즈
    float R;                          // 광류 센서 측정 노이즈
} EKF_State;

void init_ekf(EKF_State *f);
void predict(EKF_State *f, float ax, float ay, float dt);
void update(EKF_State *f, float flow_vx, float flow_vy);


#ifdef __cplusplus
}
#endif
#endif /* SRC_MAIN_NAVIGATION_KALMAN_H_ */
