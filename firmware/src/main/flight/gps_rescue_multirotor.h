/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef USE_WING

#include <stdbool.h>

#include "common/axis.h"

#include "hw.h"

//#include "pg/gps_rescue.h"

#define TASK_GPS_RESCUE_RATE_HZ 100  // in sync with altitude task rate

#ifdef USE_MAG
#define GPS_RESCUE_USE_MAG  true
#else
#define GPS_RESCUE_USE_MAG  false
#endif

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY,
    RESCUE_SANITY_COUNT
} gpsRescueSanity_e;

typedef enum {
    GPS_RESCUE_ALT_MODE_MAX = 0,
    GPS_RESCUE_ALT_MODE_FIXED,
    GPS_RESCUE_ALT_MODE_CURRENT,
    GPS_RESCUE_ALT_MODE_COUNT
} gpsRescueAltitudeMode_e;

typedef struct gpsRescue_s {
    uint8_t landingAltitudeM;   // altitude below which landing behaviours can change, metres
    uint16_t maxRescueAngle; // degrees
    uint16_t returnAltitudeM; // meters
    uint16_t descentDistanceM; // meters
    uint16_t groundSpeedCmS; // centimeters per second
    uint8_t  yawP;
    uint8_t  minSats;
    uint8_t  velP, velI, velD;
    uint16_t minStartDistM; // meters
    uint8_t  sanityChecks;
    uint8_t  allowArmingWithoutFix;
    uint8_t  useMag;
    uint8_t  altitudeMode;
    uint16_t ascendRate;
    uint16_t descendRate;
    uint16_t initialClimbM; // meters
    uint8_t  rollMix;
    uint8_t  disarmThreshold;
    uint8_t  pitchCutoffHz;
    uint8_t  imuYawGain;
} gpsRescueConfig_t;

extern float gpsRescueAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void gpsRescueConfig_Init(void);

void gpsRescueInit(void);
void gpsRescueUpdate(void);
float gpsRescueGetYawRate(void);
bool gpsRescueIsConfigured(void);
bool gpsRescueIsAvailable(void);
bool gpsRescueIsDisabled(void);
bool gpsRescueDisableMag(void);
float gpsRescueGetImuYawCogGain(void);

#endif // !USE_WING
