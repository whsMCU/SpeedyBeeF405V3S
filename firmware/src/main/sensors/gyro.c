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

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/sensor_alignment.h"

//#include "config/feature.h"
//#include "config/config.h"

#include "fc/runtime_config.h"
#include "fc/init.h"

#include "flight/pid.h"
#include "flight/position.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/opflow.h"
#include "sensors/boardalignment.h"

#include "drivers/accgyro/accgyro_spi_bmi270.h"

FAST_DATA_ZERO_INIT imu_t bmi270;

static FAST_DATA_ZERO_INIT bool overflowDetected = 0;
#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_DATA_ZERO_INIT timeUs_t overflowTimeUs;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_DATA_ZERO_INIT bool yawSpinRecoveryEnabled;
static FAST_DATA_ZERO_INIT int yawSpinRecoveryThreshold;
static FAST_DATA_ZERO_INIT bool yawSpinDetected;
static FAST_DATA_ZERO_INIT timeUs_t yawSpinTimeUs;
#endif

static FAST_DATA_ZERO_INIT float gyroFilteredDownsampled[XYZ_AXIS_COUNT];

static FAST_DATA_ZERO_INIT int16_t gyroSensorTemperature;

FAST_DATA uint8_t activePidLoopDenom = 1;

static bool firstArmingCalibrationWasStarted = false;

#define DEBUG_GYRO_CALIBRATION 3

FAST_CODE bool isGyroSensorCalibrationComplete(const imu_t *gyroSensor)
{
    return gyroSensor->gyro.calibration.cyclesRemaining == 0;
}

FAST_CODE bool gyroIsCalibrationComplete(void)
{

	return isGyroSensorCalibrationComplete(&bmi270);

}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (bmi270.gyro.gyroCalibrationDuration * 10000) / bmi270.gyro.sampleLooptime; //gyroCalibrationDuration
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(imu_t *gyroSensor)
{
#if defined(USE_FAKE_GYRO) && !defined(UNIT_TEST)
    if (gyroSensor->gyroDev.gyroHardware == GYRO_FAKE) {
        gyroSensor->calibration.cyclesRemaining = 0;
        return;
    }
#endif
    gyroSensor->gyro.calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (isFirstArmingCalibration && firstArmingCalibrationWasStarted) {
        return;
    }

    gyroSetCalibrationCycles(&bmi270);

    if (isFirstArmingCalibration) {
        firstArmingCalibrationWasStarted = true;
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !gyroIsCalibrationComplete();
}

static void performGyroCalibration(imu_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->gyro.calibration)) {
            gyroSensor->gyro.calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->gyro.calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyro.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->gyro.calibration.sum[axis] += bmi270.gyro.gyroADCRaw[axis];
        devPush(&gyroSensor->gyro.calibration.var[axis], bmi270.gyro.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->gyro.calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->gyro.calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            }

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyro.gyroZero[axis] = gyroSensor->gyro.calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyro.gyroZero[axis] -= ((float)gyroSensor->gyro.gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->gyro.calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        // if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
        //     beeper(BEEPER_GYRO_CALIBRATED);
        // }
    }

    --gyroSensor->gyro.calibration.cyclesRemaining;
}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE int32_t gyroSlewLimiter(imu_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroADCRaw[axis];
//    if (gyroConfig()->checkOverflow || gyro.gyroHasOverflowProtection) {
//        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
//        return ret;
//    }
    if (abs(ret - gyroSensor->gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

#define gyroMovementCalibrationThreshold 48

static FAST_CODE void gyroUpdateSensor(void)
{
	if (!bmi270SpiGyroRead(&bmi270)) {
		return;
	}
    bmi270.gyro.dataReady = false;
    if (isGyroSensorCalibrationComplete(&bmi270)) {
    // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
      bmi270.gyro.gyroADC[X] = gyroSlewLimiter(&bmi270, X) - bmi270.gyro.gyroZero[X];
      bmi270.gyro.gyroADC[Y] = gyroSlewLimiter(&bmi270, Y) - bmi270.gyro.gyroZero[Y];
      bmi270.gyro.gyroADC[Z] = gyroSlewLimiter(&bmi270, Z) - bmi270.gyro.gyroZero[Z];
#else
    	bmi270.gyro.gyroADC.x = bmi270.gyro.gyroADCRaw[X] - bmi270.gyro.gyroZero[X];
    	bmi270.gyro.gyroADC.y = bmi270.gyro.gyroADCRaw[Y] - bmi270.gyro.gyroZero[Y];
    	bmi270.gyro.gyroADC.z = bmi270.gyro.gyroADCRaw[Z] - bmi270.gyro.gyroZero[Z];
#endif

			alignSensorViaRotation(&bmi270.gyro.gyroADC, CW0_DEG);

    }else {
        performGyroCalibration(&bmi270, gyroMovementCalibrationThreshold);
    }
}

#define GYRO_SAMPLES_MEDIAN 3

static float applyGyroMedianFilter(int axis, float newGyroReading)
{
    static float gyroFilterSamples[XYZ_AXIS_COUNT][GYRO_SAMPLES_MEDIAN];

    for(int i = GYRO_SAMPLES_MEDIAN - 1; i>0; i--)
    {
      gyroFilterSamples[axis][i] = gyroFilterSamples[axis][i-1];
    }
    gyroFilterSamples[axis][0] = newGyroReading;

    return quickMedianFilter3f(gyroFilterSamples[axis]);
}

FAST_CODE void gyroUpdate(timeUs_t currentTimeUs)
{
  static timeUs_t previousIMUUpdateTime;
  const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
  previousIMUUpdateTime = currentTimeUs;
	UNUSED(currentTimeUs);
	gyroUpdateSensor();

	bmi270.gyro.gyroADC.x = bmi270.gyro.gyroADC.x * bmi270.gyro.scale;
	bmi270.gyro.gyroADC.y = bmi270.gyro.gyroADC.y * bmi270.gyro.scale;
	bmi270.gyro.gyroADC.z = bmi270.gyro.gyroADC.z * bmi270.gyro.scale;

//  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//    bmi270.gyroADCf[axis] = applyGyroMedianFilter(axis, bmi270.gyroADC[axis]);
//  }

  if (bmi270.gyro.downsampleFilterEnabled) {
      // using gyro lowpass 2 filter for downsampling
    bmi270.gyro.sampleSum[X] = bmi270.gyro.lowpass2FilterApplyFn((filter_t *)&bmi270.gyro.lowpass2Filter[X], bmi270.gyro.gyroADC.x);
    bmi270.gyro.sampleSum[Y] = bmi270.gyro.lowpass2FilterApplyFn((filter_t *)&bmi270.gyro.lowpass2Filter[Y], bmi270.gyro.gyroADC.y);
    bmi270.gyro.sampleSum[Z] = bmi270.gyro.lowpass2FilterApplyFn((filter_t *)&bmi270.gyro.lowpass2Filter[Z], bmi270.gyro.gyroADC.z);
  } else {
      // using simple averaging for downsampling
    bmi270.gyro.sampleSum[X] += bmi270.gyro.gyroADC.x;
    bmi270.gyro.sampleSum[Y] += bmi270.gyro.gyroADC.y;
    bmi270.gyro.sampleSum[Z] += bmi270.gyro.gyroADC.z;
    bmi270.gyro.sampleCount++;
  }

  DEBUG_SET(DEBUG_GYRO_RAW, 0, (deltaT));
  DEBUG_SET(DEBUG_GYRO_RAW, 1, (bmi270.gyro.gyroADC.x));
  DEBUG_SET(DEBUG_GYRO_RAW, 2, (bmi270.gyro.gyroADC.y));
  DEBUG_SET(DEBUG_GYRO_RAW, 3, (bmi270.gyro.gyroADC.z));

#ifdef USE_OPTICALFLOW
  // getTaskDeltaTime() returns delta time frozen at the moment of entering the scheduler. currentTime is frozen at the very same point.
  // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
  const timeDelta_t currentDeltaTime = getTaskDeltaTimeUs(TASK_SELF);

    if (sensors(SENSOR_OPFLOW)) {
        opflowGyroUpdateCallback(currentDeltaTime);
    }
#endif
}

static FAST_CODE void filterGyro(void)
{
	float gyroADCfVec[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

		// downsample the individual gyro samples
		if (bmi270.gyro.downsampleFilterEnabled) {
				// using gyro lowpass 2 filter for downsampling
			gyroADCfVec[axis] = bmi270.gyro.sampleSum[axis];
			} else {
					// using simple average for downsampling
					if (bmi270.gyro.sampleCount) {
						gyroADCfVec[axis] = bmi270.gyro.sampleSum[axis] / bmi270.gyro.sampleCount;
					}
					bmi270.gyro.sampleSum[axis] = 0;
			}

#ifdef USE_RPM_FILTER
		}

		rpmFilterRun(gyroADCfVec);

		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
#endif
      float gyroADCf = gyroADCfVec[axis];

			// apply static notch filters and software lowpass filters
			gyroADCf = bmi270.gyro.notchFilter1ApplyFn((filter_t *)&bmi270.gyro.notchFilter1[axis], gyroADCf);
			gyroADCf = bmi270.gyro.notchFilter2ApplyFn((filter_t *)&bmi270.gyro.notchFilter2[axis], gyroADCf);
			gyroADCf = bmi270.gyro.lowpassFilterApplyFn((filter_t *)&bmi270.gyro.lowpassFilter[axis], gyroADCf);

#ifdef USE_DYN_NOTCH_FILTER
			if (isDynNotchActive()) {
					dynNotchPush(axis, gyroADCf);
					gyroADCf = dynNotchFilter(axis, gyroADCf);
			}
#endif
			bmi270.gyro.gyroADCf[axis] = gyroADCf;
	}
	DEBUG_SET(DEBUG_GYRO_RAW, 4, (bmi270.gyro.gyroADCf[X]));
	DEBUG_SET(DEBUG_GYRO_RAW, 5, (bmi270.gyro.gyroADCf[Y]));
	DEBUG_SET(DEBUG_GYRO_RAW, 6, (bmi270.gyro.gyroADCf[Z]));
	bmi270.gyro.sampleCount = 0;
}

FAST_CODE void gyroFiltering(timeUs_t currentTimeUs)
{
  filterGyro();

#ifdef USE_DYN_NOTCH_FILTER
    if (isDynNotchActive()) {
        dynNotchUpdate();
    }
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyro.gyroHasOverflowProtection) {
        checkForOverflow(currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinRecoveryEnabled) {
        checkForYawSpin(currentTimeUs);
    }
#endif

    if (!overflowDetected) {
//      for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//          // integrate using trapezium rule to avoid bias
//          bmi270.gyro.gyro_accumulatedMeasurements[axis] += 0.5f * (bmi270.gyro.gyroPrevious[axis] + bmi270.gyroADCf[axis]) * bmi270.targetLooptime;
//          bmi270.gyro.gyroPrevious[axis] = bmi270.gyro.gyroADCf[axis];
//      }
//      bmi270.gyro.gyro_accumulatedMeasurementCount++;

      for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          gyroFilteredDownsampled[axis] = pt1FilterApply(&bmi270.gyro.imuGyroFilter[axis], bmi270.gyro.gyroADCf[axis]);
      }
    }
}

float gyroGetFilteredDownsampled(int axis)
{
    return gyroFilteredDownsampled[axis];
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (bmi270.gyro.gyro_accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const timeUs_t accumulatedMeasurementTimeUs = bmi270.gyro.gyro_accumulatedMeasurementCount * bmi270.gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        	accumulationAverage[axis] = bmi270.gyro.gyro_accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
        	bmi270.gyro.gyro_accumulatedMeasurements[axis] = 0.0f;
        }
        bmi270.gyro.gyro_accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CALIBRATING_ACC_CYCLES              400

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    bmi270.acc.accADC.x -= accelerationTrims->raw[X];
    bmi270.acc.accADC.y -= accelerationTrims->raw[Y];
    bmi270.acc.accADC.z -= accelerationTrims->raw[Z];
}

static void setConfigCalibrationCompleted(void)
{
	bmi270.acc.accelerationTrims.values.calibrationCompleted = 1;
}

bool accHasBeenCalibrated(void)
{
    return bmi270.acc.accelerationTrims.values.calibrationCompleted;
}

void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
    accZero->values.calibrationCompleted = 0;
}

void accStartCalibration(void)
{
    bmi270.acc.calibratingA = CALIBRATING_ACC_CYCLES;
}

bool accIsCalibrationComplete(void)
{
    return bmi270.acc.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return bmi270.acc.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return bmi270.acc.calibratingA == CALIBRATING_ACC_CYCLES;
}

void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += bmi270.acc.accADC.v[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        bmi270.acc.accADC.v[axis] = 0;
        bmi270.acc.accelerationTrims.raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    	bmi270.acc.accelerationTrims.raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
    	bmi270.acc.accelerationTrims.raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
    	bmi270.acc.accelerationTrims.raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - bmi270.acc.acc_1G;

        setConfigCalibrationCompleted();

        //saveConfigAndNotify();
    }

    bmi270.acc.calibratingA--;
}

#define acc_lpf_factor 4

void taskAccUpdate(timeUs_t currentTimeUs)
{
  static timeUs_t previousIMUUpdateTime;
  const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
  previousIMUUpdateTime = currentTimeUs;

	UNUSED(currentTimeUs);
	if (!bmi270SpiAccRead(&bmi270)) {
			return;
	}

	bmi270.acc.isAccelUpdatedAtLeastOnce = true;

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			bmi270.acc.accADC.v[axis] = bmi270.acc.accADCRaw[axis];
	}

  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.acc.accADC.v[axis] = pt2FilterApply(&bmi270.acc.accFilter[axis], bmi270.acc.accADC.v[axis]);
  }

	alignSensorViaRotation(&bmi270.acc.accADC, CW0_DEG);

  DEBUG_SET(DEBUG_ACCELEROMETER, 0, (deltaT));
  DEBUG_SET(DEBUG_ACCELEROMETER, 1, (bmi270.acc.accADC.x));
  DEBUG_SET(DEBUG_ACCELEROMETER, 2, (bmi270.acc.accADC.y));
  DEBUG_SET(DEBUG_ACCELEROMETER, 3, (bmi270.acc.accADC.z));

  if (!accIsCalibrationComplete()) {
      performAcclerationCalibration(&bmi270.acc.rollAndPitchTrims);
  }

  applyAccelerationTrims(&bmi270.acc.accelerationTrims);

  DEBUG_SET(DEBUG_ACCELEROMETER, 4, (bmi270.acc.accADC.x));
  DEBUG_SET(DEBUG_ACCELEROMETER, 5, (bmi270.acc.accADC.y));
  DEBUG_SET(DEBUG_ACCELEROMETER, 6, (bmi270.acc.accADC.z));

  static vector3_t accAdcPrev;
  vector3_t accADC;

  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    accADC.v[axis] = bmi270.acc.accADC.v[axis];
    bmi270.acc.jerk.v[axis] = (bmi270.acc.accADC.v[axis] - accAdcPrev.v[axis]) * bmi270.acc.sampleRateHz;
    accAdcPrev.v[axis] = bmi270.acc.accADC.v[axis];
  }

  bmi270.acc.accMagnitude = vector3Norm(&accADC) * bmi270.acc.acc_1G_rec;
  bmi270.acc.jerkMagnitude = vector3Norm(&bmi270.acc.jerk) * bmi270.acc.acc_1G_rec;

  // Calculate acceleration readings in G's
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.acc.accADCf[axis] = (float)bmi270.acc.accADC.v[axis] / bmi270.acc.acc_1G;
  }

  // Before filtering check for clipping and vibration levels
  if (fabsf(bmi270.acc.accADCf[X]) > ACC_CLIPPING_THRESHOLD_G || fabsf(bmi270.acc.accADCf[Y]) > ACC_CLIPPING_THRESHOLD_G || fabsf(bmi270.acc.accADCf[Z]) > ACC_CLIPPING_THRESHOLD_G) {
    bmi270.acc.isClipped = true;
    bmi270.acc.accClipCount++;
  }
  else {
    bmi270.acc.isClipped = false;
  }


//  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//    bmi270.acc.accADCf[axis] = laggedMovingAverageUpdate(&accAvg[axis].filter, (float)bmi270.acc.accADC[axis]);
//  }
//  // Calculate acceleration readings in G's
//  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//    bmi270.acc.accADCf[axis] = ((bmi270.acc.accADCf[axis] * bmi270.acc.acc_1G_rec) - 1.0f) * GRAVITY_CMSS;
//  }

  ++bmi270.acc.acc_accumulatedMeasurementCount;
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
  	bmi270.acc.acc_accumulatedMeasurements[axis] += bmi270.acc.accADC.v[axis];
  }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (bmi270.acc.acc_accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = bmi270.acc.acc_accumulatedMeasurements[axis] / bmi270.acc.acc_accumulatedMeasurementCount;
            bmi270.acc.acc_accumulatedMeasurements[axis] = 0.0f;
        }
        bmi270.acc.acc_accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

uint32_t accGetClipCount(void)
{
    return bmi270.acc.accClipCount;
}

bool accIsClipped(void)
{
    return bmi270.acc.isClipped;
}

// Record extremes: min/max for each axis and acceleration vector modulus
void updateAccExtremes(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        if (bmi270.acc.accADCf[axis] < bmi270.acc.extremes[axis].min) bmi270.acc.extremes[axis].min = bmi270.acc.accADCf[axis];
        if (bmi270.acc.accADCf[axis] > bmi270.acc.extremes[axis].max) bmi270.acc.extremes[axis].max = bmi270.acc.accADCf[axis];
    }

    float gforce = calc_length_pythagorean_3D(bmi270.acc.accADCf[X], bmi270.acc.accADCf[Y], bmi270.acc.accADCf[Z]);
    if (gforce > bmi270.acc.maxG) bmi270.acc.maxG = gforce;
}
