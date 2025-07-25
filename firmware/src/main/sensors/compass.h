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

#pragma once

#include "hw.h"
#include "common/axis.h"
#include "common/maths.h"
#include "sensors/sensors.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "common/time.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_QMC5883 = 5,
    MAG_LIS3MDL = 6,
    MAG_MPU925X_AK8963 = 7
} magSensor_e;

typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
} mag_t;

extern mag_t mag;

typedef struct compassConfig_s {
  uint8_t mag_alignment;                  // mag alignment
  uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
  flightDynamicsTrims_t magZero;
  sensorAlignment_t mag_customAlignment;
} compassConfig_t;

extern compassConfig_t compassConfig;

void compassConfig_Init(void);
bool compassIsHealthy(void);
uint32_t compassUpdate(uint32_t currentTime);
void taskUpdateMag(uint32_t currentTimeUs);
bool compassInit(void);
void compassStartCalibration(void);
bool compassIsCalibrationComplete(void);
void updateMagHold(void);

