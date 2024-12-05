//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

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

#include "build/debug.h"

#include "pid.h"
#include "sensors/gyro.h"
#include "flight/imu.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "drivers/motor.h"

#include "fc/runtime_config.h"

PIDDouble roll;
PIDDouble pitch;
PIDSingle yaw_heading;
PIDSingle yaw_rate;

DoublePID _ROLL;
DoublePID _PITCH;

#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

#define ERROR_SUM_MAX 500

void PID_Calculation(PID* axis, float set_point, float measured, float dt)
{
  axis->error = set_point - measured;
  axis->integral += axis->error * dt;
  if(axis->integral > ERROR_SUM_MAX) axis->integral = ERROR_SUM_MAX;
  else if(axis->integral < -ERROR_SUM_MAX) axis->integral = -ERROR_SUM_MAX;
  axis->derivative = (axis->error - axis->prev_error) / dt;
  axis->prev_error = axis->error;

  axis->result = (axis->kp * axis->error) + (axis->ki * axis->integral) + (axis->kd * axis->derivative);
}

void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/, float DT)
{
	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control
	axis->out.meas_value = angle;			//BNO080 rotation angle

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

	axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;	//Define summation of outer loop
#define OUT_ERR_SUM_MAX 500
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
  else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop

	axis->out.error_deriv = -rate;										//Define derivative of outer loop (rate = ICM-20602 Angular Rate)

#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
	/****************************************************************************************/
	
	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Define summation of inner loop
	#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(fabsf(axis->in.error_sum) > IN_ERR_SUM_MAX) axis->in.error_sum = IN_ERR_SUM_MAX;
  else if(axis->in.error_sum < IN_I_ERR_MIN) axis->in.error_sum = IN_I_ERR_MIN;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif
	
	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/, float DT)
{
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;
	
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw angle control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control
	
	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*ICM-20602 Angular Rate*/, float DT)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);

	_ROLL.out.integral = 0;
  _ROLL.in.integral = 0;
  _PITCH.out.integral = 0;
  _PITCH.in.integral = 0;
}

void pidInit(void)
{
	roll.in.kp = 10;
	roll.in.ki = 3;
	roll.in.kd = 2.5;

	roll.out.kp = 40;
	roll.out.ki = 0;
	roll.out.kd = 2.5;

	pitch.in.kp = 10;
	pitch.in.ki = 3;
	pitch.in.kd = 2.5;

	pitch.out.kp = 40;
	pitch.out.ki = 0;
	pitch.out.kd = 3;

	yaw_heading.kp = 100;
	yaw_heading.ki = 0;
	yaw_heading.kd = 20;

	yaw_rate.kp = 15;
	yaw_rate.ki = 0;
	yaw_rate.kd = 2;


  _ROLL.in.kp = 10;
  _ROLL.in.ki = 3;
  _ROLL.in.kd = 2.5;

  _ROLL.out.kp = 40;
  _ROLL.out.ki = 0;
  _ROLL.out.kd = 2.5;

  _PITCH.in.kp = 10;
  _PITCH.in.ki = 3;
  _PITCH.in.kd = 2.5;

  _PITCH.out.kp = 40;
  _PITCH.out.ki = 0;
  _PITCH.out.kd = 3;
}

float yaw_heading_reference;

// Function for loop trigger
void taskMainPidLoop(timeUs_t currentTimeUs)
{
	float imu_roll, imu_pitch, imu_yaw;
	imu_roll = (float)attitude.values.roll/10;
	imu_pitch = (float)attitude.values.pitch/10;
	imu_yaw = (float)attitude.values.yaw/10;

  static timeUs_t previousUpdateTimeUs;
  float dT = (float)US2S(currentTimeUs - previousUpdateTimeUs);
  debug[3] = currentTimeUs - previousUpdateTimeUs;
  previousUpdateTimeUs = currentTimeUs;


  //Double_Roll_Pitch_PID_Calculation(&pitch, rcCommand[PITCH], imu_pitch, bmi270.gyroADCf[Y], dT);
  Double_Roll_Pitch_PID_Calculation(&roll, rcCommand[ROLL], imu_roll, bmi270.gyroADCf[X], dT);

  PID_Calculation(&_ROLL.out, rcCommand[ROLL], imu_roll, dT);
  PID_Calculation(&_ROLL.in, _ROLL.out.result, bmi270.gyroADCf[X], dT);

  PID_Calculation(&_PITCH.out, rcCommand[PITCH], imu_pitch, dT);
  PID_Calculation(&_PITCH.in, _PITCH.out.result, bmi270.gyroADCf[Y], dT);

  if(rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))
  {
	  Reset_All_PID_Integrator();
  }

  if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
  {
	  yaw_heading_reference = imu_yaw;
	  Single_Yaw_Rate_PID_Calculation(&yaw_rate, rcCommand[YAW] * 10.f, -bmi270.gyroADCf[Z], dT); //left -, right +

	  LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - yaw_rate.pid_result;
	  LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + yaw_rate.pid_result;
	  RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - yaw_rate.pid_result;
	  RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + yaw_rate.pid_result;
  }
  else
  {
	  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, imu_yaw, -bmi270.gyroADCf[Z], dT);

	  LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - yaw_heading.pid_result;
	  LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + yaw_heading.pid_result;
	  RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - yaw_heading.pid_result;
	  RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + yaw_heading.pid_result;
  }

  motorWriteAll();
}
