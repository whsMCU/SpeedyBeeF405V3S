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

// EKF 상태 변수 구조체
typedef struct {
    float flowVel_x, flowVel_y;

    float x, y;       // 위치
    float vx, vy;     // 속도
    float P[4][4];    // 오차 공분산 행렬
    float Q[4][4];    // 프로세스 노이즈 (가속도 노이즈 등)
    float R[2][2];    // 측정 노이즈 (광류 센서 노이즈)
} EKF_State;

void init_ekf(EKF_State *filter);
void predict(EKF_State *f, float ax, float ay, float dt);
void update(EKF_State *f, float flow_vx, float flow_vy);


#ifdef __cplusplus
}
#endif
#endif /* SRC_MAIN_NAVIGATION_KALMAN_H_ */
