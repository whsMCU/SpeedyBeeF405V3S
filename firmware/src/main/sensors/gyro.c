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

imu_t bmi270;

static bool overflowDetected;
#ifdef USE_GYRO_OVERFLOW_CHECK
static timeUs_t overflowTimeUs;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
static bool yawSpinRecoveryEnabled;
static int yawSpinRecoveryThreshold;
static bool yawSpinDetected;
static timeUs_t yawSpinTimeUs;
#endif

uint8_t activePidLoopDenom = 1;

static bool firstArmingCalibrationWasStarted = false;

#define DEBUG_GYRO_CALIBRATION 3

bool isGyroSensorCalibrationComplete(const imu_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

bool gyroIsCalibrationComplete(void)
{

	return isGyroSensorCalibrationComplete(&bmi270);

}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (bmi270.gyroCalibrationDuration * 10000) / bmi270.sampleLooptime; //gyroCalibrationDuration
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
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
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

void performGyroCalibration(imu_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += bmi270.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], bmi270.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
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
            gyroSensor->gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroZero[axis] -= ((float)gyroSensor->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        //schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        // if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
        //     beeper(BEEPER_GYRO_CALIBRATED);
        // }
    }

    --gyroSensor->calibration.cyclesRemaining;
}

#define gyroMovementCalibrationThreshold 48

static void gyroUpdateSensor()
{
	if (!bmi270SpiGyroRead(&bmi270)) {
		return;
	}
    bmi270.dataReady = false;

    if (isGyroSensorCalibrationComplete(&bmi270)) {
    // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations
    	bmi270.gyroADC[X] = bmi270.gyroADCRaw[X] - bmi270.gyroZero[X];
    	bmi270.gyroADC[Y] = bmi270.gyroADCRaw[Y] - bmi270.gyroZero[Y];
    	bmi270.gyroADC[Z] = bmi270.gyroADCRaw[Z] - bmi270.gyroZero[Z];

			alignSensorViaRotation(bmi270.gyroADC, CW0_DEG);

    }else {
        performGyroCalibration(&bmi270, gyroMovementCalibrationThreshold);
    }
}

#define GYRO_SAMPLES_MEDIAN 3

static float applyGyrorMedianFilter(int axis, float newGyroReading)
{
    static float gyroFilterSamples[XYZ_AXIS_COUNT][GYRO_SAMPLES_MEDIAN];

    for(int i = GYRO_SAMPLES_MEDIAN - 1; i>0; i--)
    {
      gyroFilterSamples[axis][i] = gyroFilterSamples[axis][i-1];
    }
    gyroFilterSamples[axis][0] = newGyroReading;

    return quickMedianFilter3f(gyroFilterSamples[axis]);
}

void taskGyroUpdate(timeUs_t currentTimeUs)
{
  static timeUs_t previousIMUUpdateTime;
  const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
  previousIMUUpdateTime = currentTimeUs;
	UNUSED(currentTimeUs);
	gyroUpdateSensor();
	bmi270.gyroADC[X] = bmi270.gyroADC[X] * bmi270.scale;
	bmi270.gyroADC[Y] = bmi270.gyroADC[Y] * bmi270.scale;
	bmi270.gyroADC[Z] = bmi270.gyroADC[Z] * bmi270.scale;

  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.gyroADCf[axis] = applyGyrorMedianFilter(axis, bmi270.gyroADC[axis]);
  }
  DEBUG_SET(DEBUG_NONE, 0, (bmi270.gyroADCf[Y]));
  if (bmi270.downsampleFilterEnabled) {
      // using gyro lowpass 2 filter for downsampling
    bmi270.sampleSum[X] = bmi270.lowpass2FilterApplyFn((filter_t *)&bmi270.lowpass2Filter[X], bmi270.gyroADCf[X]);
    bmi270.sampleSum[Y] = bmi270.lowpass2FilterApplyFn((filter_t *)&bmi270.lowpass2Filter[Y], bmi270.gyroADCf[Y]);
    bmi270.sampleSum[Z] = bmi270.lowpass2FilterApplyFn((filter_t *)&bmi270.lowpass2Filter[Z], bmi270.gyroADCf[Z]);
  } else {
      // using simple averaging for downsampling
    bmi270.sampleSum[X] += bmi270.gyroADCf[X];
    bmi270.sampleSum[Y] += bmi270.gyroADCf[Y];
    bmi270.sampleSum[Z] += bmi270.gyroADCf[Z];
    bmi270.sampleCount++;
  }

  DEBUG_SET(DEBUG_NONE, 1, (bmi270.sampleSum[Y]));

  DEBUG_SET(DEBUG_GYRO_RAW, 0, (deltaT));
  DEBUG_SET(DEBUG_GYRO_RAW, 1, (bmi270.gyroADCf[X]));
  DEBUG_SET(DEBUG_GYRO_RAW, 2, (bmi270.gyroADCf[Y]));
  DEBUG_SET(DEBUG_GYRO_RAW, 3, (bmi270.gyroADCf[Z]));

#ifdef USE_OPFLOW
  // getTaskDeltaTime() returns delta time frozen at the moment of entering the scheduler. currentTime is frozen at the very same point.
  // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
  const timeDelta_t currentDeltaTime = getTaskDeltaTimeUs(TASK_SELF);

    if (sensors(SENSOR_OPFLOW)) {
        opflowGyroUpdateCallback(currentDeltaTime);
    }
#endif
}

void filterGyro(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

      // downsample the individual gyro samples
        float gyroADCf = 0;
        if (bmi270.downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            gyroADCf = bmi270.sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (bmi270.sampleCount) {
                gyroADCf = bmi270.sampleSum[axis] / bmi270.sampleCount;
            }
            bmi270.sampleSum[axis] = 0;
        }

#ifdef USE_RPM_FILTER
        gyroADCf = rpmFilterGyro(axis, gyroADCf);
#endif

        // apply static notch filters and software lowpass filters
        gyroADCf = bmi270.notchFilter1ApplyFn((filter_t *)&bmi270.notchFilter1[axis], gyroADCf);
        gyroADCf = bmi270.notchFilter2ApplyFn((filter_t *)&bmi270.notchFilter2[axis], gyroADCf);
        gyroADCf = bmi270.lowpassFilterApplyFn((filter_t *)&bmi270.lowpassFilter[axis], gyroADCf);

#ifdef USE_DYN_NOTCH_FILTER
        if (isDynNotchActive()) {
            dynNotchPush(axis, gyroADCf);
            gyroADCf = dynNotchFilter(axis, gyroADCf);
        }
#endif
        bmi270.gyroADCf[axis] = gyroADCf;
    }
    DEBUG_SET(DEBUG_NONE, 2, (bmi270.gyroADCf[Y]));
    bmi270.sampleCount = 0;
}

void gyroFiltering(timeUs_t currentTimeUs)
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
      for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
          // integrate using trapezium rule to avoid bias
          bmi270.gyro_accumulatedMeasurements[axis] += 0.5f * (bmi270.gyroPrevious[axis] + bmi270.gyroADCf[axis]) * bmi270.targetLooptime;
          bmi270.gyroPrevious[axis] = bmi270.gyroADCf[axis];
      }
      bmi270.gyro_accumulatedMeasurementCount++;
    }
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (bmi270.gyro_accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const timeUs_t accumulatedMeasurementTimeUs = bmi270.gyro_accumulatedMeasurementCount * bmi270.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        	accumulationAverage[axis] = bmi270.gyro_accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
        	bmi270.gyro_accumulatedMeasurements[axis] = 0.0f;
        }
        bmi270.gyro_accumulatedMeasurementCount = 0;
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
    bmi270.accADC[X] -= accelerationTrims->raw[X];
    bmi270.accADC[Y] -= accelerationTrims->raw[Y];
    bmi270.accADC[Z] -= accelerationTrims->raw[Z];
}

static void setConfigCalibrationCompleted(void)
{
	bmi270.accelerationTrims.values.calibrationCompleted = 1;
}

bool accHasBeenCalibrated(void)
{
    return bmi270.accelerationTrims.values.calibrationCompleted;
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
    bmi270.calibratingA = CALIBRATING_ACC_CYCLES;
}

bool accIsCalibrationComplete(void)
{
    return bmi270.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return bmi270.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return bmi270.calibratingA == CALIBRATING_ACC_CYCLES;
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
        a[axis] += bmi270.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        bmi270.accADC[axis] = 0;
        bmi270.accelerationTrims.raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    	bmi270.accelerationTrims.raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
    	bmi270.accelerationTrims.raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
    	bmi270.accelerationTrims.raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - bmi270.acc_1G;

        setConfigCalibrationCompleted();

        //saveConfigAndNotify();
    }

    bmi270.calibratingA--;
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

	bmi270.isAccelUpdatedAtLeastOnce = true;

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			bmi270.accADC[axis] = bmi270.accADCRaw[axis];
	}

  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.accADC[axis] = (bmi270.accADC[axis] * 0.9) + (bmi270.accPrevious[axis] * 0.1);
    bmi270.accPrevious[axis] = bmi270.accADC[axis];
  }

	alignSensorViaRotation(bmi270.accADC, CW0_DEG);

  DEBUG_SET(DEBUG_ACCELEROMETER, 0, (deltaT));
  DEBUG_SET(DEBUG_ACCELEROMETER, 1, (bmi270.accADC[X]));
  DEBUG_SET(DEBUG_ACCELEROMETER, 2, (bmi270.accADC[Y]));
  DEBUG_SET(DEBUG_ACCELEROMETER, 3, (bmi270.accADC[Z]));

  if (!accIsCalibrationComplete()) {
      performAcclerationCalibration(&bmi270.rollAndPitchTrims);
  }

  applyAccelerationTrims(&bmi270.accelerationTrims);

  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.accADCf[axis] = laggedMovingAverageUpdate(&accAvg[axis].filter, (float)bmi270.accADC[axis]);
  }
  // Calculate acceleration readings in G's
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    bmi270.accADCf[axis] = ((bmi270.accADCf[axis] * bmi270.acc_1G_rec) - 1.0f) * GRAVITY_CMSS;
  }

  ++bmi270.acc_accumulatedMeasurementCount;
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
  	bmi270.acc_accumulatedMeasurements[axis] += bmi270.accADC[axis];
  }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (bmi270.acc_accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = bmi270.acc_accumulatedMeasurements[axis] / bmi270.acc_accumulatedMeasurementCount;
            bmi270.acc_accumulatedMeasurements[axis] = 0.0f;
        }
        bmi270.acc_accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
