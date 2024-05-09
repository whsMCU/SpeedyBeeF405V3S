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

 typedef struct pidf_s {
     uint8_t P;
     uint8_t I;
     uint8_t D;
     uint16_t F;
 } pidf_t;

typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;
	
	float reference;
	float meas_value;
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;
	
	float p_result;
	float i_result;
	float d_result;
	
	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;


void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate, float DT);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value, float DT);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate, float DT);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);

void pidInit(void);
void taskMainPidLoop(timeUs_t currentTimeUs);

extern float yaw_heading_reference;

#ifdef __cplusplus
}
#endif
#endif /*__PID_CONTROL_H */
