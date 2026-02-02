/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/gps/gps.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

//#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"

#include "navigation/navigation_pos_estimator_private.h"


typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altSource_e;


positionConfig_t positionConfig;

void positionConfig_Init(void)
{
	positionConfig.altSource = DEFAULT;
	positionConfig.altNumSatsGpsUse = POSITION_DEFAULT_ALT_NUM_SATS_GPS_USE;
	positionConfig.altNumSatsBaroFallback = POSITION_DEFAULT_ALT_NUM_SATS_BARO_FALLBACK;
}

static uint8_t averagingCount;
laggedMovingAverageCombined_t  accAvg[XYZ_AXIS_COUNT];

KalmanState kf;

void kalmanInit(KalmanState* state) {
    state->altitude = 0.0f;
    state->velocity = 0.0f;

    state->P[0][0] = 1000.0f;
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 1000.0f;
}

void kalmanUpdate(KalmanState* state, float acc_z_cmps2, float pressure_alt_cm, float dt) {
    // 상태 예측
    state->altitude += state->velocity * dt + 0.5f * acc_z_cmps2 * dt * dt;
    state->velocity += acc_z_cmps2 * dt;

    // 공분산 예측
    //float q_alt = 200.0f;  // 고도 예측 노이즈
    //float q_vel = 400.0f;  // 속도 예측 노이즈

    float acc_var = 50.0f; // (cm/s²)²

    float q_alt = 0.25f * acc_var * dt*dt*dt*dt;
    float q_vel = acc_var * dt*dt;

    state->P[0][0] += dt * (state->P[0][1] + state->P[1][0]) + state->P[1][1] * dt *dt + q_alt;
    state->P[0][1] += dt * state->P[1][1];
    state->P[1][0] += dt * state->P[1][1];
    state->P[1][1] += q_vel;

    // 측정 업데이트 (기압센서 고도)
    float R = 900.0f;  // 측정 노이즈 공분산 (cm^2)
    float z = pressure_alt_cm;
    float y = z - state->altitude;  // innovation

    float S = state->P[0][0] + R;

    float K0 = state->P[0][0] / S;
    float K1 = state->P[1][0] / S;

    // 상태 보정
    state->altitude += K0 * y;
    state->velocity += K1 * y;
    state->altitude_est = state->altitude;

    // 공분산 보정
    float P00 = state->P[0][0];
    float P01 = state->P[0][1];
    float P10 = state->P[1][0];
    float P11 = state->P[1][1];

    state->P[0][0] = P00 - K0 * P00;
    state->P[0][1] = P01 - K0 * P01;
    state->P[1][0] = P10 - K1 * P00;
    state->P[1][1] = P11 - K1 * P01;
}

void position_Init(void)
{

  kalmanInit(&kf);

  averagingCount = 10;
  for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
    laggedMovingAverageInit(&accAvg[i].filter, averagingCount, (float *)&accAvg[i].buf[0]);
  }
}


static int32_t estimatedAltitudeCm = 0;                // in cm

#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#ifdef USE_VARIO
static int16_t estimatedVario = 0;                   // in cm/s

int16_t calculateEstimatedVario(int32_t baroAlt, const uint32_t dTime) {
    static float vel = 0;
    static int32_t lastBaroAlt = 0;

    int32_t baroVel = 0;

    baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = baroAlt;

    baroVel = constrain(baroVel, -1500.0f, 1500.0f);
    baroVel = applyDeadband(baroVel, 10.0f);

    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig.baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig.baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);
    vel_tmp = applyDeadband(vel_tmp, 5.0f);

    return constrain(vel_tmp, SHRT_MIN, SHRT_MAX);
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static bool altitudeOffsetSetBaro = false;
static bool altitudeOffsetSetGPS = false;
float pos_z, vel_z;
float altitude_dt;
void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    static int32_t baroAltOffset = 0;
    static int32_t gpsAltOffset = 0;

    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        schedulerIgnoreTaskExecTime();
        return;
    }
    previousTimeUs = currentTimeUs;
    altitude_dt = US2S(dTime);
    int32_t baroAlt = 0;
    int32_t gpsAlt = 0;
    uint8_t gpsNumSat = 0;

#if defined(USE_GPS) && defined(USE_VARIO)
    int16_t gpsVertSpeed = 0;
#endif
    float gpsTrust = 0.3; //conservative default
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;
#ifdef USE_BARO
			if (!baroIsCalibrationComplete()) {
					performBaroCalibrationCycle();
			} else {
					baroAlt = baroCalculateAltitude();
					haveBaroAlt = true;
			}
#endif

//#ifdef USE_GPS
//    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
//        gpsAlt = posllh.height*10;
//        gpsNumSat = GpsNav.GPS_numSat;
//#ifdef USE_VARIO
//        gpsVertSpeed = GPS_verticalSpeedInCmS;
//#endif
//        haveGpsAlt = true;
//
//        if (gpsSol.hdop != 0) {
//            gpsTrust = 100.0 / gpsSol.hdop;
//        }
//        // always use at least 10% of other sources besides gps if available
//        gpsTrust = MIN(gpsTrust, 0.9f);
//    }
//#endif

    if (ARMING_FLAG(ARMED) && !altitudeOffsetSetBaro) {
        baroAltOffset = baroAlt;
        altitudeOffsetSetBaro = true;
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSetBaro) {
        altitudeOffsetSetBaro = false;
    }

    baroAlt -= baroAltOffset;

    int goodGpsSats = 0;
    int badGpsSats = -1;

    if (haveBaroAlt) {
        goodGpsSats = positionConfig.altNumSatsGpsUse;
        badGpsSats = positionConfig.altNumSatsBaroFallback;
    }

    if (ARMING_FLAG(ARMED)) {
        if (!altitudeOffsetSetGPS && gpsNumSat >= goodGpsSats) {
            gpsAltOffset = gpsAlt - baroAlt;
            altitudeOffsetSetGPS = true;
        } else if (gpsNumSat <= badGpsSats) {
            altitudeOffsetSetGPS = false;
        }
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSetGPS) {
        altitudeOffsetSetGPS = false;
    }

    gpsAlt -= gpsAltOffset;

    if (!altitudeOffsetSetGPS) {
        haveGpsAlt = false;
        gpsTrust = 0.0f;
    }

    if (haveGpsAlt && haveBaroAlt && positionConfig.altSource == DEFAULT) {
        if (ARMING_FLAG(ARMED)) {
            estimatedAltitudeCm = gpsAlt * gpsTrust + baroAlt * (1 - gpsTrust);
        } else {
            estimatedAltitudeCm = gpsAlt; //absolute altitude is shown before arming, ignore baro
        }
#ifdef USE_VARIO
        // baro is a better source for vario, so ignore gpsVertSpeed
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    } else if (haveGpsAlt && (positionConfig.altSource == GPS_ONLY || positionConfig.altSource == DEFAULT )) {
        estimatedAltitudeCm = gpsAlt;
#if defined(USE_VARIO) && defined(USE_GPS)
        estimatedVario = gpsVertSpeed;
#endif
    } else if (haveBaroAlt && (positionConfig.altSource == BARO_ONLY || positionConfig.altSource == DEFAULT)) {
        estimatedAltitudeCm = baroAlt;
#ifdef USE_VARIO
        estimatedVario = calculateEstimatedVario(baroAlt, dTime);
#endif
    }

    vel_z += bmi270.accADCf[YAW] * US2S(dTime);
    pos_z += vel_z * US2S(dTime);

    kalmanUpdate(&kf, posEstimator.imu.accelNEU.z, estimatedAltitudeCm, US2S(dTime));



    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAlt);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAlt);
    DEBUG_SET(DEBUG_ALTITUDE, 3, (int32_t)kf.altitude_est);

#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 4, estimatedVario);
#endif
}

bool isAltitudeOffset(void)
{
    return altitudeOffsetSetBaro || altitudeOffsetSetGPS;
}
#endif

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
