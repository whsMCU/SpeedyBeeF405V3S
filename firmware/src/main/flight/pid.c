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

#include "drivers/gps/gps.h"

#include "pid.h"

#include "sensors/gyro.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "fc/rc_controls.h"

#include "rx/rx.h"

#include "drivers/motor.h"

#include "fc/runtime_config.h"

#include "sensors/compass.h"
#include "sensors/opflow.h"
#include "sensors/rangefinder.h"
#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/sqrt_controller.h"

FAST_DATA_ZERO_INIT DoublePID _ROLL;
FAST_DATA_ZERO_INIT DoublePID _PITCH;

FAST_DATA_ZERO_INIT DoublePID _ALT;
FAST_DATA_ZERO_INIT DoublePID _POS;

FAST_DATA_ZERO_INIT PID _YAW_Heading;
FAST_DATA_ZERO_INIT PID _YAW_Rate;

FAST_DATA_ZERO_INIT PID_Test _PID_Test;

FAST_DATA_ZERO_INIT float applyCommand[4];
static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

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


  _ALT.in.pidName = "ALT_IN";
  _ALT.in.kp = 1;
  _ALT.in.ki = 0;
  _ALT.in.kd = 0;
  _ALT.in.integral_windup = 200;

  _ALT.out.pidName = "ALT_OUT";
  _ALT.out.kp = 1.5f;
  _ALT.out.ki = 0.1f;
  _ALT.out.kd = 0.2f;
  _ALT.out.integral_windup = 200;

  _POS.in.pidName = "POS_IN";
  _POS.in.kp = 1;
  _POS.in.ki = 0;
  _POS.in.kd = 0;
  _POS.in.integral_windup = 200;

  _POS.out.pidName = "POS_OUT";
  _POS.out.kp = 1.5f;
  _POS.out.ki = 0.1f;
  _POS.out.kd = 0.2f;
  _POS.out.integral_windup = 200;

  _PID_Test.pid_test_flag = 0;
  _PID_Test.pid_test_throttle = 0;
  _PID_Test.pid_test_deg = 0;
}

void PID_Calculation(PID* axis, float set_point, float measured1, float measured2, float dt)
{
  axis->error = set_point - measured1;

  if (strcmp(axis->pidName, "YAW_Heading") == 0)
  {
    if (axis->error > 180.0f)  axis->error -= 360.0f;
    if (axis->error < -180.0f) axis->error += 360.0f;
  }

  axis->integral += axis->error * dt;
  if(axis->integral > axis->integral_windup) axis->integral = axis->integral_windup;
  else if(axis->integral < -axis->integral_windup) axis->integral = -axis->integral_windup;

  if (strcmp(axis->pidName, "ROLL_OUT") == 0 || strcmp(axis->pidName, "PITCH_OUT") == 0 || strcmp(axis->pidName, "YAW_Heading") == 0)
  {
    axis->derivative = -measured2;
    axis->derivative_filter = axis->derivative_filter * 0.4f + axis->derivative * 0.6f;
  }else{
    axis->measured = measured1;
    axis->pre_measured = axis->prev_measured;

    axis->derivative = -(measured1 - axis->prev_measured) / dt;
    axis->prev_measured = measured1;

    axis->derivative_filter = axis->derivative_filter * 0.5f + axis->derivative * 0.5f;
  }

  axis->result_p = axis->kp * axis->error;
  axis->result_i = axis->ki * axis->integral;
  axis->result_d = axis->kd * axis->derivative_filter;

  axis->result = axis->result_p + axis->result_i + axis->result_d;
  if(axis->result > 5000) axis->result = 5000;
  if(axis->result < -5000) axis->result = -5000;
}

void Reset_All_PID_Integrator(void)
{
	_ROLL.out.integral = 0;
  _ROLL.in.integral = 0;
  _PITCH.out.integral = 0;
  _PITCH.in.integral = 0;
  _YAW_Heading.integral = 0;
  _YAW_Rate.integral = 0;
  _ALT.in.integral = 0;
  _ALT.out.integral = 0;
}

float yaw_heading_reference;

// Function for loop trigger
void taskMainPidLoop(timeUs_t currentTimeUs)
{
	float imu_roll, imu_pitch, imu_yaw;
	imu_roll = (float)DECIDEGREES_TO_DEGREES(attitude.values.roll);
	imu_pitch = (float)DECIDEGREES_TO_DEGREES(attitude.values.pitch);
	imu_yaw = (float)DECIDEGREES_TO_DEGREES(attitude.values.yaw);
	heading = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

  static timeUs_t previousUpdateTimeUs;
  float dT = (float)US2S(currentTimeUs - previousUpdateTimeUs);
  previousUpdateTimeUs = currentTimeUs;

#ifdef USE_GPS1
  if ( (FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME) ) {
    float sin_yaw_y = sin(heading*0.0174532925f);
    float cos_yaw_x = cos(heading*0.0174532925f);
    #if defined(NAV_SLEW_RATE)
      GpsNav.nav_rated[LON]   += constrain(wrap_18000(GpsNav.nav[LON]-GpsNav.nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
      GpsNav.nav_rated[LAT]   += constrain(wrap_18000(GpsNav.nav[LAT]-GpsNav.nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
      GpsNav.GPS_angle[ROLL]   = (GpsNav.nav_rated[LON]*cos_yaw_x - GpsNav.nav_rated[LAT]*sin_yaw_y) /10;
      GpsNav.GPS_angle[PITCH]  = (GpsNav.nav_rated[LON]*sin_yaw_y + GpsNav.nav_rated[LAT]*cos_yaw_x) /10;
    #else
      GpsNav.GPS_angle[ROLL]   = (GpsNav.nav[LON]*cos_yaw_x - GpsNav.nav[LAT]*sin_yaw_y) /10;
      GpsNav.GPS_angle[PITCH]  = (GpsNav.nav[LON]*sin_yaw_y + GpsNav.nav[LAT]*cos_yaw_x) /10;
    #endif
  } else {
    GpsNav.GPS_angle[ROLL]  = 0;
    GpsNav.GPS_angle[PITCH] = 0;
  }
#endif

  if (isRXDataNew) {
      updateWaypointsAndNavigationMode();
  }
  isRXDataNew = false;

  applyWaypointNavigationAndAltitudeHold();

  applyCommand[ROLL]      = scaleRangef(rcCommand[ROLL],  -500.0f, 500.0f, -30.0f,  30.f);
  applyCommand[PITCH]     = scaleRangef(rcCommand[PITCH], -500.0f, 500.0f, -30.0f,  30.f);
  applyCommand[YAW]       = scaleRangef(rcCommand[YAW],   -500.0f, 500.0f, -500.0f, 500.f);
  applyCommand[THROTTLE]  = scaleRangef(rcCommand[THROTTLE] + throttleAngleCorrection, 1000.0f, 2000.0f, 0.0f, 1000.0f);

  PID_Calculation(&_ROLL.out, applyCommand[ROLL], imu_roll, bmi270.gyroADCf[X], dT);
  PID_Calculation(&_ROLL.in, _ROLL.out.result, bmi270.gyroADCf[X], 0, dT);

  if(_PID_Test.pid_test_flag == 1)
  {
    PID_Calculation(&_PITCH.out, _PID_Test.pid_test_deg, imu_pitch, bmi270.gyroADCf[Y], dT);
  }else
  {
    PID_Calculation(&_PITCH.out, applyCommand[PITCH], imu_pitch, bmi270.gyroADCf[Y], dT);
  }
  PID_Calculation(&_PITCH.in, _PITCH.out.result, bmi270.gyroADCf[Y], 0, dT);

//  DEBUG_SET(DEBUG_PIDLOOP, 0, (_PID_Test.pid_test_deg));
//  DEBUG_SET(DEBUG_PIDLOOP, 1, (imu_pitch));
//  DEBUG_SET(DEBUG_PIDLOOP, 2, (_PITCH.out.error));
//  DEBUG_SET(DEBUG_PIDLOOP, 3, (_PITCH.out.result_p));
//  DEBUG_SET(DEBUG_PIDLOOP, 4, (_PITCH.out.result_i));
//  DEBUG_SET(DEBUG_PIDLOOP, 5, (_PITCH.out.result));
//  DEBUG_SET(DEBUG_PIDLOOP, 6, (bmi270.gyroADCf[Y]));
//  DEBUG_SET(DEBUG_PIDLOOP, 7, (_PITCH.in.error));
//  DEBUG_SET(DEBUG_PIDLOOP, 8, (_PITCH.in.result_p));
//  DEBUG_SET(DEBUG_PIDLOOP, 9, (_PITCH.in.result_i));
//  DEBUG_SET(DEBUG_PIDLOOP, 10, (_PITCH.in.result_d));
//  DEBUG_SET(DEBUG_PIDLOOP, 11, (_PITCH.in.result));
//  DEBUG_SET(DEBUG_PIDLOOP, 12, (_PITCH.in.pre_measured));
//  DEBUG_SET(DEBUG_PIDLOOP, 13, (_PITCH.in.measured));
//  DEBUG_SET(DEBUG_PIDLOOP, 14, (_PITCH.in.derivative));
//  DEBUG_SET(DEBUG_PIDLOOP, 15, (_PITCH.in.derivative_filter));

  if((rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))&& _PID_Test.pid_test_flag == 0)
  {
	  Reset_All_PID_Integrator();
  }

  if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
  {
	  yaw_heading_reference = imu_yaw;

	  PID_Calculation(&_YAW_Rate, applyCommand[YAW], -bmi270.gyroADCf[Z], 0, dT);//left -, right +

	  if(_PID_Test.pid_test_flag == 1)
	  {
	    LF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
	  }else
	  {
	    LF = 10500 + 500 + applyCommand[THROTTLE] * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + applyCommand[THROTTLE] * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + applyCommand[THROTTLE] * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + applyCommand[THROTTLE] * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
	  }
  }
  else
  {
	  PID_Calculation(&_YAW_Heading, yaw_heading_reference, imu_yaw, -bmi270.gyroADCf[Z], dT);

    DEBUG_SET(DEBUG_PIDLOOP, 0, (yaw_heading_reference));
    DEBUG_SET(DEBUG_PIDLOOP, 1, (imu_yaw));
	  DEBUG_SET(DEBUG_PIDLOOP, 2, (_YAW_Heading.error));
	  DEBUG_SET(DEBUG_PIDLOOP, 3, (_YAW_Heading.result_p));
	  DEBUG_SET(DEBUG_PIDLOOP, 4, (_YAW_Heading.result_i));
    DEBUG_SET(DEBUG_PIDLOOP, 5, (_YAW_Heading.result_d));
	  DEBUG_SET(DEBUG_PIDLOOP, 6, (_YAW_Heading.result));

    if(_PID_Test.pid_test_flag == 1)
    {
      LF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Heading.result;
      LR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Heading.result;
      RR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Heading.result;
      RF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Heading.result;
    }else
    {
      LF = 10500 + 500 + applyCommand[THROTTLE] * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Heading.result;
      LR = 10500 + 500 + applyCommand[THROTTLE] * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Heading.result;
      RR = 10500 + 500 + applyCommand[THROTTLE] * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Heading.result;
      RF = 10500 + 500 + applyCommand[THROTTLE] * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Heading.result;
    }
  }

  motorWriteAll();

#if defined(USE_GPS) || defined(USE_MAG)
    if (sensors(SENSOR_GPS) || sensors(SENSOR_MAG)) {
        updateMagHold();
    }
#endif
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination)
{
    angleDeciDegrees = constrainf(angleDeciDegrees, (float) -maxInclination, (float) maxInclination);
    return scaleRangef((float) angleDeciDegrees, (float) -maxInclination, (float) maxInclination, -500.0f, 500.0f);
}
