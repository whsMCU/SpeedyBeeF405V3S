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

FAST_DATA_ZERO_INIT PID _YAW_Heading;
FAST_DATA_ZERO_INIT PID _YAW_Rate;

FAST_DATA_ZERO_INIT PID_Test _PID_Test;

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

int16_t altHoldThrottleRCZero = 1500;

#ifdef USE_RANGEFINDER

static inline float pt1_apply(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

static inline float apply_deadband(float v, float db)
{
    return (fabsf(v) < db) ? 0.0f : v;
}

#define MAX_DERIVATIVE        200.0f    // max |dz/dt| in cm/s used by D (after filtering)
#define ALT_ERR_DEADBAND      1.0f      // cm, small error deadband to avoid chatter
#define ALT_TARGET_SOFTLOCK   10.0f    // cm, limit target within plausible range quickly
#define ALT_RESULT_LIMIT      200.0f    // mixer units (matches your original)
#define ALT_DZ_FILTER_ALPHA   0.2f      // PT1 for altitude rate estimate (0..1, lower = smoother)
#define ALT_DERIV_CLAMP_ENABLE 1        // 1: clamp derivative spikes to 0 as original logic

#define alt_hold_deadband     50
#define maxthrottle           2000
#define idlethrottle          1050

#define THROTTLE_SLEW_US_PER_S 20000.0f   // 초당 20000us → 1ms당 ≈20us

//#define ALT_BACK_CALC_ENABLE

static void updateAltHold_RANGEFINDER(rangefinder_t *alt_sensor, timeUs_t currentTimeUs);

#endif
#ifdef USE_OPFLOW
static void updatePosHold(timeUs_t currentTimeUs);
#endif

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

  rangefinder.althold.K_BACK_CALC = 2.0f;
  rangefinder.althold.integral_windup = 200;

  rangefinder.althold.target_Height = 0;
  rangefinder.althold.error_Height = 0.0f;
  rangefinder.althold.proportional_Height = 0.0f;
  rangefinder.althold.integral_Height = 0.0f;
  rangefinder.althold.derivative_Height = 0.0f;
  rangefinder.althold.result = 0.0f;
  rangefinder.althold.pre_Height = 0;
  rangefinder.althold.dz_filtered = 0.0f;

  opflow.poshold.KP = 1.0f;
  opflow.poshold.KI = 0.1f;
  opflow.poshold.KD = 0.15f;
  opflow.poshold.integral_windup = 5;

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
      //updateWaypointsAndNavigationMode();
  }
  isRXDataNew = false;

  #ifdef USE_RANGEFINDER
    updateAltHold_RANGEFINDER(&rangefinder, currentTimeUs);

    DEBUG_SET(DEBUG_RANGEFINDER, 0, (rangefinder.althold.target_Height));
    DEBUG_SET(DEBUG_RANGEFINDER, 1, (navGetCurrentActualPositionAndVelocity()->pos.z));
    DEBUG_SET(DEBUG_RANGEFINDER, 2, (rangefinder.althold.error_Height));
    DEBUG_SET(DEBUG_RANGEFINDER, 3, (rangefinder.althold.proportional_Height));
    DEBUG_SET(DEBUG_RANGEFINDER, 4, (rangefinder.althold.integral_Height));
    DEBUG_SET(DEBUG_RANGEFINDER, 5, (rangefinder.althold.derivative_Height));
    DEBUG_SET(DEBUG_RANGEFINDER, 6, (rangefinder.althold.result));
    DEBUG_SET(DEBUG_RANGEFINDER, 7, (rcCommand[THROTTLE]));

  #endif
  #ifdef USE_OPFLOW
    updatePosHold(currentTimeUs);
  #endif
  PID_Calculation(&_ROLL.out, rcCommand[ROLL] + GpsNav.GPS_angle[ROLL], imu_roll, bmi270.gyroADCf[X], dT);
  PID_Calculation(&_ROLL.in, _ROLL.out.result, bmi270.gyroADCf[X], 0, dT);

  if(_PID_Test.pid_test_flag == 1)
  {
    PID_Calculation(&_PITCH.out, _PID_Test.pid_test_deg, imu_pitch, bmi270.gyroADCf[Y], dT);
  }else
  {
    PID_Calculation(&_PITCH.out, rcCommand[PITCH] + GpsNav.GPS_angle[PITCH], imu_pitch, bmi270.gyroADCf[Y], dT);
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

  throttle = rcCommand[THROTTLE] + throttleAngleCorrection;

  if((rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))&& _PID_Test.pid_test_flag == 0)
  {
	  Reset_All_PID_Integrator();
  }

  if(rcData[YAW] < 1485 || rcData[YAW] > 1515)
  {
	  yaw_heading_reference = imu_yaw;

	  PID_Calculation(&_YAW_Rate, rcCommand[YAW], -bmi270.gyroADCf[Z], 0, dT);//left -, right +

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

#ifdef USE_OPFLOW
#define FLOW_LPF_ALPHA 0.2f  // 0.0 ~ 1.0 (낮을수록 부드러움)
void updatePosHold(timeUs_t currentTimeUs)
{
  opflow_poshold_t *poshold = &opflow.poshold;
  if(FLIGHT_MODE(OPFLOW_HOLD_MODE) && abs(rcCommand[ROLL]) < 2 && abs(rcCommand[PITCH]) < 2)
  {
    static uint32_t pre_time = 0;
    uint32_t now_time = currentTimeUs;
    poshold->dt = (float)US2S(now_time - pre_time);
    pre_time = now_time;

    for (int i = 0; i < 2; i++) {
      poshold->filteredFlowRate[i] = poshold->filteredFlowRate[i] * (1 - FLOW_LPF_ALPHA) + RADIANS_TO_DEGREES(opflow.flowRate[i]) * FLOW_LPF_ALPHA;
    }
    // FlowRate[X] : 오른 -, 왼쪽+, +-30
    // FlowRate[Y] : 앞 -, 뒤 +, +-30

    //Position(Pixcl Coordinate)
    poshold->Pixel[X] += poshold->filteredFlowRate[X] * poshold->dt;
    poshold->Pixel[Y] += poshold->filteredFlowRate[Y] * poshold->dt;

    if(rcData[THROTTLE] < 1030)
    {
      poshold->Pixel[X] = 0;
      poshold->Pixel[Y] = 0;
    }

    poshold->target_Pixel[X] = 0;
    poshold->target_Pixel[Y] = 0;
    poshold->error_Pixel[X] = poshold->target_Pixel[X] - poshold->Pixel[X];
    poshold->error_Pixel[Y] = poshold->target_Pixel[Y] - poshold->Pixel[Y];

    poshold->proportional_Pixel[X] = poshold->KP * poshold->error_Pixel[X];
    poshold->proportional_Pixel[Y] = poshold->KP * poshold->error_Pixel[Y];

    poshold->derivative_Pixel[X] = poshold->KD * poshold->filteredFlowRate[X];
    poshold->derivative_Pixel[Y] = poshold->KD * poshold->filteredFlowRate[Y];

    poshold->integral_Pixel[X] += poshold->KI * poshold->error_Pixel[X] * poshold->dt;
    poshold->integral_Pixel[Y] += poshold->KI * poshold->error_Pixel[Y] * poshold->dt;

    if(poshold->integral_Pixel[X] > poshold->integral_windup) poshold->integral_Pixel[X] = poshold->integral_windup;
    else if(poshold->integral_Pixel[X] < -poshold->integral_windup) poshold->integral_Pixel[X] = -poshold->integral_windup;

    if(poshold->integral_Pixel[Y] > poshold->integral_windup) poshold->integral_Pixel[Y] = poshold->integral_windup;
    else if(poshold->integral_Pixel[Y] < -poshold->integral_windup) poshold->integral_Pixel[Y] = -poshold->integral_windup;


    if(rcData[THROTTLE] < 1030)
    {
      poshold->integral_Pixel[X] = 0;
      poshold->integral_Pixel[Y] = 0;
    }

    poshold->target_Angle[X] = poshold->proportional_Pixel[X] + poshold->integral_Pixel[X] + poshold->derivative_Pixel[X];
    poshold->target_Angle[Y] = poshold->proportional_Pixel[Y] + poshold->integral_Pixel[Y] + poshold->derivative_Pixel[Y];

    if(rcData[THROTTLE] < 1030)
    {
      poshold->target_Angle[X] = 0;
      poshold->target_Angle[Y] = 0;
    }

    poshold->target_Angle[X] = -constrain(poshold->target_Angle[X], -10, 10);
    poshold->target_Angle[Y] = -constrain(poshold->target_Angle[Y], -10, 10);

    rcCommand[ROLL] = poshold->target_Angle[X];
    rcCommand[PITCH] = poshold->target_Angle[Y];

    DEBUG_SET(DEBUG_POS_HOLD, 2, poshold->filteredFlowRate[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 0, poshold->Pixel[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 1, poshold->error_Pixel[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 3, poshold->proportional_Pixel[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 4, poshold->integral_Pixel[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 5, poshold->derivative_Pixel[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 6, poshold->target_Angle[X]);
    DEBUG_SET(DEBUG_POS_HOLD, 7, rcCommand[ROLL]);
  }
}
#endif

#ifdef USE_RANGEFINDER
static sqrt_controller_t alt_hold_sqrt_controller;

void updateAltHold_RANGEFINDER(rangefinder_t *alt_sensor, timeUs_t currentTimeUs)
{
  rangefinder_althold_t *althold = &alt_sensor->althold;
  const navEstimatedPosVel_t *posToUse = navGetCurrentActualPositionAndVelocity();

  if (!FLIGHT_MODE(RANGEFINDER_MODE)) {
      return;
  }

  static float pilot_Throttle = 1500;
  static float throttleOut = 1500;
  static uint32_t pre_time = 0;
  uint32_t now_time = currentTimeUs;
  althold->dt = (pre_time == 0) ? (float)US2S(1000) : (float)US2S(now_time - pre_time);
  pre_time = now_time;

  // 센서 값 검증
  if (!isfinite(posToUse->pos.z) ||
      posToUse->pos.z < 0.0f ||
      posToUse->pos.z > 200.0f) {
      return;
  }

  pilot_Throttle = rcData[THROTTLE];

  // Disarm 상태 → Soft-lock 적용
  if(rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))
  {
    althold->integral_Height = 0;
    althold->dz_filtered = 0.0f;
    althold->pre_Height = posToUse->pos.z;

    const float err_soft = posToUse->pos.z - althold->target_Height;
    const float max_pull = ALT_TARGET_SOFTLOCK * althold->dt; // cm per dt
    if (fabsf(err_soft) > max_pull) {
        althold->target_Height += (err_soft > 0 ? max_pull : -max_pull);
    } else {
        althold->target_Height = posToUse->pos.z;
    }
    posControl.desiredState.vel.z = posToUse->vel.z;   // Gradually transition from current climb
    althold->proportional_Height = 0;
    althold->derivative_Height = 0;
    althold->result = 0;

//    float nav_speed_up = 0.0f;
//    float nav_speed_down = 0.0f;
//    float nav_accel_z = 0.0f;
//
//    nav_speed_up = navConfig.general.max_manual_speed;
//    nav_accel_z = navConfig.general.max_manual_speed;
//    nav_speed_down = navConfig.general.max_manual_climb_rate;

//    sqrtControllerInit(
//        &alt_hold_sqrt_controller,
//        posControl.pids.pos[Z].param.kP,
//        -fabsf(nav_speed_down),
//        nav_speed_up,
//        nav_accel_z
//    );
    return;
  }

  // 스틱 입력 → 목표 고도 갱신
  althold->rcThrottleAdjustment = applyDeadbandRescaled(pilot_Throttle - altHoldThrottleRCZero, alt_hold_deadband, -500, 500);

  if (althold->rcThrottleAdjustment) {
      // set velocity proportional to stick movement

      // Make sure we can satisfy max_manual_climb_rate in both up and down directions
      if (althold->rcThrottleAdjustment > 0) {
          // Scaling from altHoldThrottleRCZero to maxthrottle
        althold->rcClimbRate = althold->rcThrottleAdjustment * navConfig.general.max_manual_climb_rate / (float)(maxthrottle - altHoldThrottleRCZero - alt_hold_deadband);
      }
      else {
          // Scaling from minthrottle to altHoldThrottleRCZero
        althold->rcClimbRate = althold->rcThrottleAdjustment * navConfig.general.max_manual_climb_rate / (float)(altHoldThrottleRCZero - idlethrottle - alt_hold_deadband);
      }
      updateClimbRateToAltitudeController(althold->rcClimbRate, ROC_TO_ALT_NORMAL);
  }
  else {
      // Adjusting finished - reset desired position to stay exactly where pilot released the stick
    updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
    althold->rcClimbRate = 0;
  }

  // Execute actual altitude controllers
//  float targetVel = sqrtControllerApply(
//      &alt_hold_sqrt_controller,
//      posControl.desiredState.pos.z,
//      navGetCurrentActualPositionAndVelocity()->pos.z,
//      althold->dt
//  );

  althold->target_Height += althold->rcClimbRate * althold->dt;

  // 고도 오차
  float error = althold->target_Height - posToUse->pos.z; // cm
  if (fabsf(error) < ALT_ERR_DEADBAND) error = 0.0f;
  althold->error_Height = error;

  // P항
  althold->proportional_Height = althold->KP * althold->error_Height;

  // I항 (조건부 적분, anti-windup)
  althold->integral_Height += althold->KI * (althold->error_Height * althold->dt);
  if(althold->integral_Height > althold->integral_windup) althold->integral_Height = althold->integral_windup;
  else if(althold->integral_Height < -althold->integral_windup) althold->integral_Height = -althold->integral_windup;

  // D항 (속도 기반, 노이즈 필터)
  float dz = (posToUse->pos.z - althold->pre_Height) / althold->dt; // cm/s (positive = going up)
  althold->pre_Height = posToUse->pos.z;

  // Simple PT1 filter on dz to reduce RF noise
  althold->dz_filtered = pt1_apply(althold->dz_filtered, dz, ALT_DZ_FILTER_ALPHA);

  float derivative = -althold->dz_filtered; // negative sign: oppose motion toward error

#if ALT_DERIV_CLAMP_ENABLE
  if (fabsf(derivative) > MAX_DERIVATIVE) {
      derivative = 0.0f; // reject spikes as in original code
  }
#endif
  althold->derivative_Height = althold->KD * derivative;

  // PID 합산
  float outputBeforeSaturation  = althold->proportional_Height + althold->integral_Height + althold->derivative_Height;
  althold->result = constrainf(outputBeforeSaturation, -ALT_RESULT_LIMIT, ALT_RESULT_LIMIT);

  // Back-calculation Anti-windup
#ifdef ALT_BACK_CALC_ENABLE
  float anti_windup_error = althold->result - outputBeforeSaturation;
  althold->integral_Height += althold->K_BACK_CALC * anti_windup_error * althold->dt;
#endif

  // 스로틀 출력 계산 (ESC 범위에서 Slew 제한)
  float throttle_cmd = (float) constrainf(pilot_Throttle + althold->result, 1000.0f, 2000.0f);
  float maxStep = THROTTLE_SLEW_US_PER_S * althold->dt;
  float step    = (float)throttle_cmd - (float)throttleOut;
  if (step >  maxStep) step =  maxStep;
  if (step < -maxStep) step = -maxStep;
  throttleOut += step;

  rcCommand[THROTTLE] = scaleRangef(throttleOut, 1000.0f, 2000.0f, 0.0f, 1000.0f);
}
#endif

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}
