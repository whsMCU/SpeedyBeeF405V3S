/*
 * kalman.c
 *
 *  Created on: 2026. 2. 11.
 *      Author: 왕학승
 */


#include "navigation/kalman.h"


// 초기화 함수
void init_ekf(EKF_State *filter) {
    filter->x = 0; filter->y = 0;
    filter->vx = 0; filter->vy = 0;

    // 공분산 초기화 (Identity)
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            filter->P[i][j] = (i == j) ? 1.0f : 0.0f;
            filter->Q[i][j] = (i == j) ? 0.01f : 0.0f; // 임의 설정
        }
    }
    // 광류 센서 측정 노이즈 설정
    filter->R[0][0] = 0.1f; filter->R[1][1] = 0.1f;
}

// 1. 예측 단계 (Predict Step)
// ax, ay: 지구 고정 좌표계로 변환된 가속도
void predict(EKF_State *f, float ax, float ay, float dt) {
    // --- 상태 예측 ---
    f->x = f->x + f->vx * dt + 0.5f * ax * dt * dt;
    f->y = f->y + f->vy * dt + 0.5f * ay * dt * dt;
    f->vx = f->vx + ax * dt;
    f->vy = f->vy + ay * dt;

    // --- 오차 공분산 업데이트 (P = F*P*F' + Q) ---
    // Jacobian F (행렬: p_next = p + v*dt)
    // F = [1 0 dt 0]
    //     [0 1 0 dt]
    //     [0 0 1  0]
    //     [0 0 0  1]

    // 단순화를 위해 P 행렬 업데이트 계산 (F*P*F')
    float newP[4][4];
    newP[0][0] = f->P[0][0] + dt*(f->P[2][0] + f->P[0][2] + dt*f->P[2][2]);
    newP[1][1] = f->P[1][1] + dt*(f->P[3][1] + f->P[1][3] + dt*f->P[3][3]);
    // (나머지 성분 생략 및 Q 합산 - 실제 구현 시 전체 행렬 곱 수행 필요)

    for(int i=0; i<4; i++) f->P[i][i] += f->Q[i][i];
}

// 2. 보정 단계 (Update Step)
// flow_vx, flow_vy: 광류 센서와 고도로 계산된 속도
void update(EKF_State *f, float flow_vx, float flow_vy) {
    // 관측 행렬 H = [0 0 1 0], [0 0 0 1] (속도만 측정)

    // 1. 잔차(Residual) 계산
    float y_vx = flow_vx - f->vx;
    float y_vy = flow_vy - f->vy;

    // 2. 칼만 이득(Kalman Gain) K = P*H' / (H*P*H' + R)
    // 분모 S = H*P*H' + R
    float S_x = f->P[2][2] + f->R[0][0];
    float S_y = f->P[3][3] + f->R[1][1];

    float K_vx = f->P[2][2] / S_x;
    float K_vy = f->P[3][3] / S_y;

    // 3. 상태 업데이트
    f->vx += K_vx * y_vx;
    f->vy += K_vy * y_vy;

    // 4. 공분산 업데이트 (P = (I - KH)P)
    f->P[2][2] *= (1.0f - K_vx);
    f->P[3][3] *= (1.0f - K_vy);
}
