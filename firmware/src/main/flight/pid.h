/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "hw.h"
#include "common/time.h"
#include "common/axis.h"

// PID 구조체 정의
typedef struct _PID{
   float kp;     // 비례 게인
   float ki;     // 적분 게인
   float kd;     // 미분 게인
   float error;
   float prev_error; // 이전 오차값
   float integral;   // 적분 값
   float derivative;

   float integral_windup;

   float result_p;
   float result_i;
   float result_d;
   float result;
} PID;

typedef struct _DoublePID
{
  PID in;
  PID out;
}DoublePID;

extern DoublePID _ROLL;
extern DoublePID _PITCH;
extern PID _YAW_Heading;
extern PID _YAW_Rate;


void PID_Calculation(PID* axis, float set_point, float measured, float dt);
void Reset_All_PID_Integrator(void);

void pidInit(void);
void taskMainPidLoop(timeUs_t currentTimeUs);

extern float yaw_heading_reference;

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
