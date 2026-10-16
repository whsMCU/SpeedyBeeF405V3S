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
#include "common/time.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/sensor_alignment.h"
#include "common/vector.h"
#include "sensors/sensors.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "common/time.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

#define TASK_COMPASS_RATE_HZ 40 // the base mag update rate; faster intervals will apply for higher ODR mags

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_QMC5883L = 5,
    MAG_LIS2MDL = 6,
    MAG_LIS3MDL = 7,
    MAG_MPU925X_AK8963 = 8,
    MAG_IST8310 = 9,
    MAG_MMC560X = 10,
    MAG_QMC5883P = 11,
    MAG_DRONECAN = 12,
    MAG_HARDWARE_COUNT
} magSensor_e;

typedef struct mag_s {
    bool isNewMagADCFlag;
	vector3_t magADC;
} mag_t;

extern mag_t mag;

typedef struct compassConfig_s {
  uint8_t mag_alignment;                  // mag alignment
  uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
  flightDynamicsTrims_t magZero;
  sensorAlignment_t mag_customAlignment;
} compassConfig_t;

typedef struct compassBiasEstimator_s {
    float lambda_min, lambda;
    float b[3];
    float theta[4];
    float U[4][4];
    float D[4];
} compassBiasEstimator_t;

extern compassConfig_t compassConfig;

bool compassEnabledAndCalibrated(void);
uint32_t compassUpdate(timeUs_t currentTime);
bool compassInit(void);
void compassConfig_Init(void);
bool compassIsHealthy(void);

void taskUpdateMag(timeUs_t currentTimeUs);

void compassStartCalibration(void);
bool compassIsCalibrationComplete(void);
void updateMagHold(void);


void compassBiasEstimatorInit(compassBiasEstimator_t *compassBiasEstimator, const float lambda_min, const float p0);
void compassBiasEstimatorUpdate(compassBiasEstimator_t *compassBiasEstimator, const float lambda_min, const float p0);
void compassBiasEstimatorApply(compassBiasEstimator_t *cBE, vector3_t *mag);

