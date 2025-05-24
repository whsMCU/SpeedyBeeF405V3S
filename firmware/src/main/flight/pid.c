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

DoublePID _ROLL;
DoublePID _PITCH;

PID _YAW_Heading;
PID _YAW_Rate;

PID_Test _PID_Test;

void pidInit(void)
{
  _ROLL.in.pidName = "ROLL_IN";
  _ROLL.in.kp = 10;
  _ROLL.in.ki = 0;
  _ROLL.in.kd = 0;
  _ROLL.in.integral_windup = 500;

  _ROLL.out.pidName = "ROLL_OUT";
  _ROLL.out.kp = 10;
  _ROLL.out.ki = 0;
  _ROLL.out.kd = 0;
  _ROLL.out.integral_windup = 500;

  _PITCH.in.pidName = "PITCH_IN";
  _PITCH.in.kp = 10;
  _PITCH.in.ki = 0;
  _PITCH.in.kd = 0;
  _PITCH.in.integral_windup = 500;

  _PITCH.out.pidName = "PITCH_OUT";
  _PITCH.out.kp = 10;
  _PITCH.out.ki = 0;
  _PITCH.out.kd = 0;
  _PITCH.out.integral_windup = 500;

  _YAW_Heading.pidName = "YAW_Heading";
  _YAW_Heading.kp = 10;
  _YAW_Heading.ki = 0;
  _YAW_Heading.kd = 0;
  _YAW_Heading.integral_windup = 500;

  _YAW_Rate.pidName = "YAW_Rate";
  _YAW_Rate.kp = 10;
  _YAW_Rate.ki = 0;
  _YAW_Rate.kd = 0;
  _YAW_Rate.integral_windup = 500;

  _PID_Test.pid_test_flag = 0;
  _PID_Test.pid_test_throttle = 0;
  _PID_Test.pid_test_deg = 0;
}

void PID_Calculation(PID* axis, float set_point, float measured, float dt)
{
  axis->error = set_point - measured;

  if (strcmp(axis->pidName, "YAW_Heading") == 0)
  {
    if (axis->error > 180.0f)  axis->error -= 360.0f;
    if (axis->error < -180.0f) axis->error += 360.0f;
  }

  axis->integral += axis->error * dt;
  if(axis->integral > axis->integral_windup) axis->integral = axis->integral_windup;
  else if(axis->integral < -axis->integral_windup) axis->integral = -axis->integral_windup;
  axis->derivative = -(measured - axis->prev_error) / dt;
  axis->prev_error = measured;

  axis->derivative_filter = axis->derivative_filter * 0.5f + axis->derivative * 0.5f;

  axis->result_p = axis->kp * axis->error;
  axis->result_i = axis->ki * axis->integral;
  axis->result_d = axis->kd * axis->derivative_filter;

  axis->result = axis->result_p + axis->result_i + axis->result_d;
}

void Reset_All_PID_Integrator(void)
{
	_ROLL.out.integral = 0;
  _ROLL.in.integral = 0;
  _PITCH.out.integral = 0;
  _PITCH.in.integral = 0;
  _YAW_Heading.integral = 0;
  _YAW_Rate.integral = 0;
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
  debug[0] = currentTimeUs - previousUpdateTimeUs;
  previousUpdateTimeUs = currentTimeUs;
  debug[1] = bmi270.gyroADCf[Y];
  debug[2] = _PITCH.in.result_d;
  debug[3] = _PITCH.in.result;

  PID_Calculation(&_ROLL.out, rcCommand[ROLL], imu_roll, dT);
  PID_Calculation(&_ROLL.in, _ROLL.out.result, bmi270.gyroADCf[X], dT);

  if(_PID_Test.pid_test_flag == 1)
  {
    PID_Calculation(&_PITCH.out, _PID_Test.pid_test_deg, imu_pitch, dT);
  }else
  {
    PID_Calculation(&_PITCH.out, rcCommand[PITCH], imu_pitch, dT);
  }
  PID_Calculation(&_PITCH.in, _PITCH.out.result, bmi270.gyroADCf[Y], dT);

  if((rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))&& _PID_Test.pid_test_flag == 0)
  {
	  Reset_All_PID_Integrator();
  }

  if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
  {
	  yaw_heading_reference = imu_yaw;

	  PID_Calculation(&_YAW_Rate, rcCommand[YAW] * 10.f, -bmi270.gyroADCf[Z], dT);//left -, right +

	  if(_PID_Test.pid_test_flag == 1)
	  {
	    LF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
	  }else
	  {
	    LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
	  }
  }
  else
  {
	  PID_Calculation(&_YAW_Heading, yaw_heading_reference, imu_yaw, dT);
    if(_PID_Test.pid_test_flag == 1)
    {
      LF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Heading.result;
      LR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Heading.result;
      RR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Heading.result;
      RF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Heading.result;
    }else
    {
      LF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Heading.result;
      LR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Heading.result;
      RR = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Heading.result;
      RF = 10500 + 500 + (rcData[THROTTLE] - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Heading.result;
    }
  }

  motorWriteAll();
}
