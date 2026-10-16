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

#include "flight/dyn_notch_filter.h"

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
  bmi270.gyro.gyro_high_fsr = false;
  bmi270.gyro.sampleRateHz = 3200;
  bmi270.gyro.gyroRateKHz = GYRO_RATE_3200_Hz;
  bmi270.gyro.hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
  bmi270.gyro.gyro_offset_yaw = 0;
  bmi270.gyro.gyroCalibrationDuration = 125;
  bmi270.gyro.sampleLooptime = 312;
  bmi270.gyro.targetLooptime = 312;
  bmi270.gyro.scale = GYRO_SCALE_2000DPS;
  bmi270.gyro.downsampleFilterEnabled = true;

  bmi270.gyro.gyro_soft_notch_hz_1 = 0;
  bmi270.gyro.gyro_soft_notch_cutoff_1 = 0;
  bmi270.gyro.gyro_soft_notch_hz_2 = 0;
  bmi270.gyro.gyro_soft_notch_cutoff_2 = 0;

  bmi270.gyro.dynNotchConfig.dyn_notch_count = 3;
  bmi270.gyro.dynNotchConfig.dyn_notch_max_hz = 600;
  bmi270.gyro.dynNotchConfig.dyn_notch_min_hz = 150;
  bmi270.gyro.dynNotchConfig.dyn_notch_q = 300;

  bmi270.acc.sampleRateHz = 800;
  bmi270.acc.acc_1G = 512 * 4;
  bmi270.acc.acc_1G_rec = 1.0f / bmi270.acc.acc_1G;
  bmi270.acc.acc_high_fsr = false;
  bmi270.acc.acc_lpf_hz = 25;

  resetFlightDynamicsTrims(&bmi270.acc.accelerationTrims);
  bmi270.acc.accelerationTrims.values.roll = 21;
  bmi270.acc.accelerationTrims.values.pitch = -55;
  bmi270.acc.accelerationTrims.values.yaw = -6;

  bmi270.gyro.init_gyro_cal_enabled = true;
  bmi270.acc.gravity_cmss_cal = 0;
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / bmi270.gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

static void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    bmi270.gyro.notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        bmi270.gyro.notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&bmi270.gyro.notchFilter1[axis], notchHz, bmi270.gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    bmi270.gyro.notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
      bmi270.gyro.notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&bmi270.gyro.notchFilter2[axis], notchHz, bmi270.gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void accInitFilters(void)
{
    // Only set the lowpass cutoff if the ACC sample rate is detected otherwise
    // the filter initialization is not defined (sample rate = 0)
  bmi270.acc.accLpfCutHz = (bmi270.acc.sampleRateHz) ? bmi270.acc.acc_lpf_hz : 0;
    if (bmi270.acc.accLpfCutHz) {
        const float k = pt2FilterGain(bmi270.acc.accLpfCutHz, HZ_TO_INTERVAL(bmi270.acc.sampleRateHz));
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt2FilterInit(&bmi270.acc.accFilter[axis], k);
        }
    }
}

static void initGyroLPF(float cutoffHz, float gyroDt,
                        filterApplyFnPtr *applyFn, gyroLowpassFilter_t *filters)
{
    *applyFn = (filterApplyFnPtr) pt1FilterApply;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterInit(&filters[axis].pt1FilterState, pt1FilterGain(cutoffHz, gyroDt));
    }
}

//static DMA_DATA uint8_t gyroBuf[GYRO_COUNT][2][GYRO_BUF_SIZE / 2];
//
//for (int i = 0; i < GYRO_COUNT; i++) {
//    if (gyroDetectedFlags & GYRO_MASK(i)) {  // Only initialize detected gyros
//        // SPI DMA buffer required per device
//        gyro.gyroSensor[i].gyroDev.dev.txBuf = gyroBuf[i][0];
//        gyro.gyroSensor[i].gyroDev.dev.rxBuf = gyroBuf[i][1];

bool gyroInit(void)
{
	static uint8_t gyroBuf1[GYRO_BUF_SIZE];
	// SPI DMA buffer required per device
	bmi270.txBuf = gyroBuf1;
	bmi270.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];

	bmi270Config();
	sensorsSet(SENSOR_ACC);
	sensorsSet(SENSOR_GYRO);

  float lpf1Hz = 250.f;
  float lpf2Hz = 500.f;
  float gyroDt = bmi270.gyro.sampleLooptime * 1e-6f;

	initGyroLPF(lpf1Hz, gyroDt, &bmi270.gyro.lowpassFilterApplyFn, bmi270.gyro.lowpassFilter);
	initGyroLPF(lpf2Hz, gyroDt, &bmi270.gyro.lowpass2FilterApplyFn, bmi270.gyro.lowpass2Filter);

  gyroInitFilterNotch1(bmi270.gyro.gyro_soft_notch_hz_1, bmi270.gyro.gyro_soft_notch_cutoff_1);
  gyroInitFilterNotch2(bmi270.gyro.gyro_soft_notch_hz_2, bmi270.gyro.gyro_soft_notch_cutoff_2);

#ifdef USE_DYN_LPF
    dynLpfFilterInit();
#endif

#ifdef USE_DYN_NOTCH_FILTER
    dynNotchInit(&bmi270.gyro.dynNotchConfig, bmi270.gyro.targetLooptime * 1e-6f);
#endif

  const float k = pt1FilterGain(GYRO_IMU_DOWNSAMPLE_CUTOFF_HZ, bmi270.gyro.targetLooptime * 1e-6f);
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      pt1FilterInit(&bmi270.gyro.imuGyroFilter[axis], k);
  }

    accInitFilters();

  return true;
}

int16_t gyroRateDps(int axis)
{
    return lrintf(bmi270.gyro.gyroADCf[axis] / bmi270.gyro.scale);
}
