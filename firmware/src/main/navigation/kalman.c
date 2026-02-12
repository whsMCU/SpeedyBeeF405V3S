/*
 * kalman.c
 *
 *  Created on: 2026. 2. 11.
 *      Author: 왕학승
 */


#include "navigation/kalman.h"


// 초기화 함수
void init_ekf(EKF_State *f) {

  for (int i = 0; i < STATE_DIM; i++) {
      f->x[i] = 0.0f;
      for (int j = 0; j < STATE_DIM; j++) {
          f->P[i][j] = (i == j) ? 1.0f : 0.0f;  // P 초기값 (Identity)
          f->Q[i][j] = 0.0f;
      }
  }
  // 프로세스 노이즈 설정 (가속도 노이즈 영향)
  f->Q[0][0] = 0.001f; f->Q[1][1] = 0.001f;
  f->Q[2][2] = 0.01f;  f->Q[3][3] = 0.01f;

  // 광류 센서 측정 노이즈 (속도 측정의 불확실성)
  f->R = 0.1f;

}

// 1. 예측 단계 (Predict Step)
// ax, ay: 지구 좌표계 가속도(m/s^2), dt: 주기(s)
void predict(EKF_State *f, float ax, float ay, float dt) {
  // --- (1) 상태 예측 ---
  f->x[0] += f->x[2] * dt + 0.5f * ax * dt * dt; // x = x + vx*dt + 0.5*a*dt^2
  f->x[1] += f->x[3] * dt + 0.5f * ay * dt * dt; // y = y + vy*dt + 0.5*a*dt^2
  f->x[2] += ax * dt;                            // vx = vx + a*dt
  f->x[3] += ay * dt;                            // vy = vy + a*dt

  // --- (2) 공분산 예측 (P = FPF' + Q) ---
  // F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
  // 행렬 곱 연산을 단순화하여 대각 및 관련 성분만 업데이트
  for (int i = 0; i < 2; i++) {
      f->P[i][i] += (f->P[i+2][i+2] * dt * dt) + (2.0f * f->P[i][i+2] * dt);
      f->P[i][i+2] += f->P[i+2][i+2] * dt;
      f->P[i+2][i] = f->P[i][i+2];
  }

  // Q 더하기
  for (int i = 0; i < STATE_DIM; i++) f->P[i][i] += f->Q[i][i];
}

// 2. 보정 단계 (Update Step)
// flow_vx, flow_vy: 광류 센서에서 계산된 속도(m/s)
void update(EKF_State *f, float flow_vx, float flow_vy) {
  float z[2] = {flow_vx, flow_vy};

  // 관측 행렬 H는 [0, 0, 1, 0] (x_dot) 및 [0, 0, 0, 1] (y_dot)
  for (int i = 0; i < 2; i++) {
      int vel_idx = i + 2; // vx는 index 2, vy는 index 3

      // --- (1) 칼만 이득 계산 ---
      // S = HPH' + R
      float S = f->P[vel_idx][vel_idx] + f->R;
      float K_pos = f->P[i][vel_idx] / S;       // 위치 수정 이득
      float K_vel = f->P[vel_idx][vel_idx] / S; // 속도 수정 이득

      // --- (2) 상태 수정 ---
      float y = z[i] - f->x[vel_idx]; // 잔차
      f->x[i] += K_pos * y;           // 위치 보정
      f->x[vel_idx] += K_vel * y;     // 속도 보정

      // --- (3) 공분산 수정 (P = (I-KH)P) ---
      // 해당 축의 위치와 속도 관련 공분산만 간략 업데이트
      f->P[i][i] -= K_pos * f->P[vel_idx][i];
      f->P[vel_idx][vel_idx] -= K_vel * f->P[vel_idx][vel_idx];
  }
}
