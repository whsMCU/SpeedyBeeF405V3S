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
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "build/debug.h"

#include "hw.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

//#include "config/config.h"

#include "fc/runtime_config.h"

#include "drivers/accgyro/accgyro_spi_bmi270.h"
//#include "drivers/accgyro/gyro_sync.h"


#include "sensors/gyro.h"
#include "sensors/sensors.h"


// The gyro buffer is split 50/50, the first half for the transmit buffer, the second half for the receive buffer
// This buffer is large enough for the gyros currently supported in accgyro_mpu.c but should be reviewed id other
// gyro types are supported with SPI DMA.
#define GYRO_BUF_SIZE 32

void gyroConfig_init(void)
{
  bmi270.gyro_high_fsr = false;
  bmi270.gyroSampleRateHz = 3200;
  bmi270.gyroRateKHz = GYRO_RATE_3200_Hz;
  bmi270.hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
  bmi270.gyro_offset_yaw = 0;
  bmi270.gyroCalibrationDuration = 125;
  bmi270.sampleLooptime = 312;
  bmi270.targetLooptime = 312;
  bmi270.sampleRateHz = 3200;
  bmi270.scale = GYRO_SCALE_2000DPS;

  bmi270.accSampleRateHz = 800;
  bmi270.acc_1G = 512 * 4;
  bmi270.acc_1G_rec = 1.0f / bmi270.acc_1G;
  bmi270.acc_high_fsr = false;

  resetFlightDynamicsTrims(&bmi270.accelerationTrims);
  bmi270.accelerationTrims.values.roll = 29;
  bmi270.accelerationTrims.values.pitch = -35;
  bmi270.accelerationTrims.values.yaw = -9;
}

bool gyroInit(void)
{
	static uint8_t gyroBuf1[GYRO_BUF_SIZE];
	// SPI DMA buffer required per device
	bmi270.txBuf = gyroBuf1;
	bmi270.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];

	bmi270Config();

  filterApplyFnPtr *lowpassFilterApplyFn;
  gyroLowpassFilter_t *lowpassFilter = NULL;

  lowpassFilterApplyFn = &bmi270.lowpassFilterApplyFn;
  lowpassFilter = bmi270.lowpassFilter;

  float lpf1Hz = 250.f;
  float lpf2Hz = 500.f;
  float gyroDt = 312 * 1e-6f;

	float gain = pt1FilterGain(lpf1Hz, gyroDt);
	*lowpassFilterApplyFn = nullFilterApply;
	*lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
  }

  gain = pt1FilterGain(lpf2Hz, gyroDt);
  lowpassFilterApplyFn = &bmi270.lowpass2FilterApplyFn;
  lowpassFilter = bmi270.lowpass2Filter;

  *lowpassFilterApplyFn = nullFilterApply;
  *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
  }

  return true;
}

int16_t gyroRateDps(int axis)
{
    return lrintf(bmi270.gyroADCf[axis] / bmi270.scale);
}
