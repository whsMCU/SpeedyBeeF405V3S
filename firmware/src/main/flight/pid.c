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

FAST_DATA_ZERO_INIT DoublePID _ROLL;
FAST_DATA_ZERO_INIT DoublePID _PITCH;

FAST_DATA_ZERO_INIT PID _YAW_Heading;
FAST_DATA_ZERO_INIT PID _YAW_Rate;

FAST_DATA_ZERO_INIT PID _ALT;

FAST_DATA_ZERO_INIT PID_Test _PID_Test;

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

#ifdef USE_RANGEFINDER

static inline float pt1_apply(float prev, float input, float alpha)
{
    return prev + alpha * (input - prev);
}

static inline float apply_deadband(float v, float db)
{
    return (fabsf(v) < db) ? 0.0f : v;
}

#define MAX_DERIVATIVE        30.0f      // max |dz/dt| in cm/s used by D (after filtering)
#define ALT_ERR_DEADBAND      0.5f      // cm, small error deadband to avoid chatter
#define ALT_MAX_CLIMB_RATE    50.0f     // cm/s, limit for stick-driven target rate
#define ALT_TARGET_SOFTLOCK   100.0f    // cm, limit target within plausible range quickly
#define ALT_RESULT_LIMIT      100.0f    // mixer units (matches your original)
#define ALT_DZ_FILTER_ALPHA   0.2f      // PT1 for altitude rate estimate (0..1, lower = smoother)
#define ALT_DERIV_CLAMP_ENABLE 1        // 1: clamp derivative spikes to 0 as original logic

#define STICK_deadband        0.1      //+-5%
#define climb_rate_fullstick  20

#define THROTTLE_SLEW_US_PER_S 20000.0f   // 초당 20000us → 1ms당 ≈20us

static void updateAltHold_RANGEFINDER(timeUs_t currentTimeUs);

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

  _ALT.pidName = "ALT";
  _ALT.kp = 5;
  _ALT.ki = 0;
  _ALT.kd = 0;
  _ALT.integral_windup = 500;

  rangefinder.althold.KP = 1.0f;
  rangefinder.althold.KI = 0.3f;
  rangefinder.althold.KD = 0.4f;
  rangefinder.althold.integral_windup = 200;

  rangefinder.althold.stick_deadband = fminf(fmaxf(STICK_deadband, 0.0f), 0.2f);
  rangefinder.althold.climb_rate_scale = fminf(fabsf(climb_rate_fullstick), ALT_MAX_CLIMB_RATE);

  rangefinder.althold.target_Height = rangefinder.calculatedAltitude;
  rangefinder.althold.error_Height = 0.0f;
  rangefinder.althold.proportional_Height = 0.0f;
  rangefinder.althold.integral_Height = 0.0f;
  rangefinder.althold.derivative_Height = 0.0f;
  rangefinder.althold.result = 0.0f;
  rangefinder.althold.pre_Height = rangefinder.calculatedAltitude;
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
    if (axis->error >= 180.0f)  axis->error -= 360.0f;
    if (axis->error <= -180.0f) axis->error += 360.0f;
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
  previousUpdateTimeUs = currentTimeUs;

  /* Update estimate */
  updateEstimatedTopic(currentTimeUs);

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

  #ifdef USE_RANGEFINDER
    updateAltHold_RANGEFINDER(currentTimeUs);
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

  DEBUG_SET(DEBUG_PIDLOOP, 0, (_PID_Test.pid_test_deg));
  DEBUG_SET(DEBUG_PIDLOOP, 1, (imu_pitch));
  DEBUG_SET(DEBUG_PIDLOOP, 2, (_PITCH.out.error));
  DEBUG_SET(DEBUG_PIDLOOP, 3, (_PITCH.out.result_p));
  DEBUG_SET(DEBUG_PIDLOOP, 4, (_PITCH.out.result_i));
  DEBUG_SET(DEBUG_PIDLOOP, 5, (_PITCH.out.result));
  DEBUG_SET(DEBUG_PIDLOOP, 6, (bmi270.gyroADCf[Y]));
  DEBUG_SET(DEBUG_PIDLOOP, 7, (_PITCH.in.error));
  DEBUG_SET(DEBUG_PIDLOOP, 8, (_PITCH.in.result_p));
  DEBUG_SET(DEBUG_PIDLOOP, 9, (_PITCH.in.result_i));
  DEBUG_SET(DEBUG_PIDLOOP, 10, (_PITCH.in.result_d));
  DEBUG_SET(DEBUG_PIDLOOP, 11, (_PITCH.in.result));
  DEBUG_SET(DEBUG_PIDLOOP, 12, (_PITCH.in.pre_measured));
  DEBUG_SET(DEBUG_PIDLOOP, 13, (_PITCH.in.measured));
  DEBUG_SET(DEBUG_PIDLOOP, 14, (_PITCH.in.derivative));
  DEBUG_SET(DEBUG_PIDLOOP, 15, (_PITCH.in.derivative_filter));

  throttle = rcCommand[THROTTLE] + throttleAngleCorrection;

  DEBUG_SET(DEBUG_NONE, 0, (throttle));
  DEBUG_SET(DEBUG_NONE, 1, (throttleAngleCorrection));

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

  //motorWriteAll();

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
  PID_Calculation(&_ALT, targetVel, (float)getEstimatedVario(), 0, dT);
  _ALT.result = constrain(_ALT.result, -200, 200);

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

}

#ifdef USE_OPFLOW
#define FLOW_LPF_ALPHA 0.2f  // 0.0 ~ 1.0 (낮을수록 부드러움)
void updatePosHold(timeUs_t currentTimeUs)
{
  opflow_poshold_t *poshold = &opflow.poshold;
  if(FLIGHT_MODE(OPFLOW_HOLD_MODE) && abs(rcCommand[ROLL]) < 1 && abs(rcCommand[PITCH]) < 1)
  {
    static uint32_t pre_time = 0;
    uint32_t now_time = currentTimeUs;
    poshold->dt = (float)US2S(now_time - pre_time);
    pre_time = now_time;

    for (int i = 0; i < 2; i++) {
      poshold->filteredFlowRate[i] = poshold->filteredFlowRate[i] * (1 - FLOW_LPF_ALPHA) + opflow.flowRate[i] * FLOW_LPF_ALPHA;
    }

    poshold->Pixel[X] = poshold->Pixel[X] + poshold->filteredFlowRate[X] * poshold->dt;
    poshold->Pixel[Y] = poshold->Pixel[Y] + poshold->filteredFlowRate[Y] * poshold->dt;

    if(rcData[THROTTLE] < 1030)
    {
      poshold->Pixel[X] = 0;
      poshold->Pixel[Y] = 0;
    }

    poshold->target_Pixel[X] = 0;
    poshold->target_Pixel[Y] = 0;
    poshold->error_Pixel[X] = poshold->target_Pixel[X] - poshold->Pixel[X];
    poshold->error_Pixel[Y] = poshold->target_Pixel[Y] - poshold->Pixel[Y];

    poshold->target_Angle[X] = poshold->KP * poshold->error_Pixel[X];
    poshold->target_Angle[Y] = poshold->KP * poshold->error_Pixel[Y];

    poshold->target_Angle[X] += -poshold->KD * -poshold->filteredFlowRate[X];
    poshold->target_Angle[Y] += -poshold->KD * -poshold->filteredFlowRate[Y];

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

    poshold->target_Angle[X] += poshold->integral_Pixel[X];
    poshold->target_Angle[Y] += poshold->integral_Pixel[Y];

    if(rcData[THROTTLE] < 1030)
    {
      poshold->target_Angle[X] = 0;
      poshold->target_Angle[Y] = 0;
    }

    poshold->target_Angle[X] = constrain(poshold->target_Angle[X], -25, 25);
    poshold->target_Angle[Y] = -constrain(poshold->target_Angle[Y], -25, 25);

    rcCommand[ROLL] = poshold->target_Angle[X];
    rcCommand[PITCH] = poshold->target_Angle[Y];

    DEBUG_SET(DEBUG_NONE, 0, poshold->Pixel[X]);
    DEBUG_SET(DEBUG_NONE, 1, poshold->Pixel[Y]);
    DEBUG_SET(DEBUG_NONE, 2, rcCommand[ROLL]);
    DEBUG_SET(DEBUG_NONE, 3, rcCommand[PITCH]);
  }
}
#endif

#ifdef USE_RANGEFINDER

void updateAltHold_RANGEFINDER(timeUs_t currentTimeUs)
{
  rangefinder_althold_t *althold = &rangefinder.althold;

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
  if (!isfinite((float)rangefinder.calculatedAltitude) ||
      (float)rangefinder.calculatedAltitude < 0.0f ||
      (float)rangefinder.calculatedAltitude > 200.0f) {
      return;
  }

  pilot_Throttle = rcData[THROTTLE];

  // Disarm 상태 → Soft-lock 적용
  if(rcData[THROTTLE] < 1030 || !ARMING_FLAG(ARMED))
  {
    althold->integral_Height = 0;
    althold->dz_filtered = 0.0f;
    althold->pre_Height = (float)rangefinder.calculatedAltitude;

    const float err_soft = (float)rangefinder.calculatedAltitude - althold->target_Height;
    const float max_pull = ALT_TARGET_SOFTLOCK * althold->dt; // cm per dt
    if (fabsf(err_soft) > max_pull) {
        althold->target_Height += (err_soft > 0 ? max_pull : -max_pull);
    } else {
        althold->target_Height = (float)rangefinder.calculatedAltitude;
    }
    althold->proportional_Height = 0;
    althold->derivative_Height = 0;
    althold->result = 0;
    return;
  }

  // 스틱 입력 → 목표 고도 갱신
  const float throttleStick = (float)(rcData[THROTTLE] - 1500) / 500.0f; // -1..+1
  const float stick = apply_deadband(throttleStick, althold->stick_deadband);
  const float climbRateCmd = stick * fminf(althold->climb_rate_scale, ALT_MAX_CLIMB_RATE);
  althold->target_Height += climbRateCmd * althold->dt;

  // 고도 오차
  float error = althold->target_Height - (float)rangefinder.calculatedAltitude; // cm
  if (fabsf(error) < ALT_ERR_DEADBAND) error = 0.0f;
  althold->error_Height = error;

  // P항
  althold->proportional_Height = althold->KP * althold->error_Height;

  // I항 (조건부 적분, anti-windup)
  althold->integral_Height += althold->KI * (althold->error_Height * althold->dt);
  if(althold->integral_Height > althold->integral_windup) althold->integral_Height = althold->integral_windup;
  else if(althold->integral_Height < -althold->integral_windup) althold->integral_Height = -althold->integral_windup;

  // D항 (속도 기반, 노이즈 필터)
  float dz = ((float)rangefinder.calculatedAltitude - althold->pre_Height) / althold->dt; // cm/s (positive = going up)
  althold->pre_Height = (float)rangefinder.calculatedAltitude;

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
  althold->result = althold->proportional_Height + althold->integral_Height + althold->derivative_Height;
  althold->result = constrainf(althold->result, -ALT_RESULT_LIMIT, ALT_RESULT_LIMIT);

  // 스로틀 출력 계산 (ESC 범위에서 Slew 제한)
  float throttle_cmd = (float) constrainf(pilot_Throttle + althold->result, 1000.0f, 2000.0f);
  float maxStep = THROTTLE_SLEW_US_PER_S * althold->dt;
  float step    = (float)throttle_cmd - (float)throttleOut;
  if (step >  maxStep) step =  maxStep;
  if (step < -maxStep) step = -maxStep;
  throttleOut += step;

  rcCommand[THROTTLE] = scaleRangef(throttleOut, 1000.0f, 2000.0f, 0.0f, 1000.0f);

  DEBUG_SET(DEBUG_RANGEFINDER, 0, (althold->dt / 1e-6f));
  DEBUG_SET(DEBUG_RANGEFINDER, 1, (althold->target_Height));
  DEBUG_SET(DEBUG_RANGEFINDER, 2, (rangefinder.calculatedAltitude));
  DEBUG_SET(DEBUG_RANGEFINDER, 3, (althold->error_Height));
  DEBUG_SET(DEBUG_RANGEFINDER, 4, (althold->proportional_Height));
  DEBUG_SET(DEBUG_RANGEFINDER, 5, (althold->integral_Height));
  DEBUG_SET(DEBUG_RANGEFINDER, 6, (althold->derivative_Height));
  DEBUG_SET(DEBUG_RANGEFINDER, 7, (althold->result));
  DEBUG_SET(DEBUG_RANGEFINDER, 8, (rcCommand[THROTTLE]));
}
#endif

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}
