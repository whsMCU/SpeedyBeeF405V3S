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

DoublePID _ROLL;
DoublePID _PITCH;

PID _YAW_Heading;
PID _YAW_Rate;

PID _ALT;

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

  _ALT.pidName = "ALT";
  _ALT.kp = 5;
  _ALT.ki = 0;
  _ALT.kd = 0;
  _ALT.integral_windup = 200;

  _PID_Test.pid_test_flag = 0;
  _PID_Test.pid_test_throttle = 0;
  _PID_Test.pid_test_deg = 0;
}

void PID_Calculation(PID* axis, float set_point, float measured, float dt)
{
  axis->error = set_point - measured;

  if (strcmp(axis->pidName, "YAW_Heading") == 0)
  {
    if (axis->error >= 180.0f)  axis->error -= 360.0f;
    if (axis->error <= -180.0f) axis->error += 360.0f;
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
  _ALT.integral = 0;
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
  //debug[0] = currentTimeUs - previousUpdateTimeUs;
  previousUpdateTimeUs = currentTimeUs;
  //debug[1] = bmi270.gyroADCf[Y];
  //debug[2] = _PITCH.in.result_d;
  //debug[3] = _PITCH.in.result;

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

	  PID_Calculation(&_YAW_Rate, rcCommand[YAW], -bmi270.gyroADCf[Z], dT);//left -, right +

	  if(_PID_Test.pid_test_flag == 1)
	  {
	    LF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + (_PID_Test.pid_test_throttle - 1000) * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
	  }else
	  {
	    LF = 10500 + 500 + rcCommand[THROTTLE] * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Rate.result;
	    LR = 10500 + 500 + rcCommand[THROTTLE] * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Rate.result;
	    RR = 10500 + 500 + rcCommand[THROTTLE] * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Rate.result;
	    RF = 10500 + 500 + rcCommand[THROTTLE] * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Rate.result;
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
      LF = 10500 + 500 + rcCommand[THROTTLE] * 10 - _PITCH.in.result + _ROLL.in.result - _YAW_Heading.result;
      LR = 10500 + 500 + rcCommand[THROTTLE] * 10 + _PITCH.in.result + _ROLL.in.result + _YAW_Heading.result;
      RR = 10500 + 500 + rcCommand[THROTTLE] * 10 + _PITCH.in.result - _ROLL.in.result - _YAW_Heading.result;
      RF = 10500 + 500 + rcCommand[THROTTLE] * 10 - _PITCH.in.result - _ROLL.in.result + _YAW_Heading.result;
    }
  }

  motorWriteAll();

#if defined(USE_GPS) || defined(USE_MAG)
    if (sensors(SENSOR_GPS) || sensors(SENSOR_MAG)) {
        updateMagHold();
    }
#endif
}

void updateAltHold(timeUs_t currentTimeUs)
{
  static timeUs_t previousUpdateTimeUs;
  float dT = (float)US2S(currentTimeUs - previousUpdateTimeUs);
  //debug[0] = currentTimeUs - previousUpdateTimeUs;
  previousUpdateTimeUs = currentTimeUs;

  float targetVel = constrain(AltHold - getEstimatedAltitudeCm(), -100, 100);
  PID_Calculation(&_ALT, targetVel, (float)getEstimatedVario(), dT);
  _ALT.result = constrain(_ALT.result, -200, 200);

  debug[0] = initialThrottleHold;
  debug[1] = (int32_t)rcCommand[THROTTLE];

  if(FLIGHT_MODE(BARO_MODE))
  {
    static uint8_t isAltHoldChanged = 0;
    #if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
      if (abs(rcCommand[THROTTLE]-initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
       //errorAltitudeI = 0;
        isAltHoldChanged = 1;
        rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -ALT_HOLD_THROTTLE_NEUTRAL_ZONE : ALT_HOLD_THROTTLE_NEUTRAL_ZONE;
       // initialThrottleHold += (rcCommand[THROTTLE] > initialThrottleHold) ? -ALT_HOLD_THROTTLE_NEUTRAL_ZONE : ALT_HOLD_THROTTLE_NEUTRAL_ZONE;; //++hex nano
      } else {
        if (isAltHoldChanged) {
          AltHold = getEstimatedAltitudeCm();
          isAltHoldChanged = 0;
        }
        rcCommand[THROTTLE] = initialThrottleHold + _ALT.result;
      }
    #else
      static int16_t AltHoldCorr = 0;
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
        AltHoldCorr+= rcCommand[THROTTLE] - initialThrottleHold;
        if(abs(AltHoldCorr) > 500) {
          AltHold += AltHoldCorr/500;
          AltHoldCorr %= 500;
        }
        //errorAltitudeI = 0;
        isAltHoldChanged = 1;
      } else if (isAltHoldChanged) {
        AltHold = getEstimatedAltitudeCm();
        isAltHoldChanged = 0;
      }
      rcCommand[THROTTLE] = initialThrottleHold + _ALT.result;
    #endif
  }
  debug[2] = (int32_t)_ALT.result;
  debug[3] = (int32_t)rcCommand[THROTTLE];
}
