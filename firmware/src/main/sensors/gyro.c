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

//#include "fc/runtime_config.h"

//#include "flight/pid.h"

//#include "scheduler/scheduler.h"

#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "drivers/accgyro/accgyro_spi_bmi270.h"

imu_t bmi270;

static int16_t gyroSensorTemperature;

uint8_t activePidLoopDenom = 1;

static bool firstArmingCalibrationWasStarted = false;

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyro.gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyro.gyroSensor1.gyroDev;
#endif


#define DEBUG_GYRO_CALIBRATION 3

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

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
//		if (gyroSensor->gyroDev.gyroAlign == ALIGN_CUSTOM) {
//			alignSensorViaMatrix(gyroSensor->gyroDev.gyroADC, &gyroSensor->gyroDev.rotationMatrix);
//		} else {
//			alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
//		}
    }else {
        performGyroCalibration(&bmi270, gyroMovementCalibrationThreshold);
    }
}

void gyroUpdate(void)
{

	gyroUpdateSensor();
	bmi270.gyroADC[X] = bmi270.gyroADC[X] * bmi270.scale;
	bmi270.gyroADC[Y] = bmi270.gyroADC[Y] * bmi270.scale;
	bmi270.gyroADC[Z] = bmi270.gyroADC[Z] * bmi270.scale;

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			bmi270.gyroADCf[axis] = bmi270.gyroADC[axis];
	}
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      // integrate using trapezium rule to avoid bias
  		bmi270.gyro_accumulatedMeasurements[axis] += 0.5f * (bmi270.gyroPrevious[axis] + bmi270.gyroADCf[axis]) * bmi270.targetLooptime;
  		bmi270.gyroPrevious[axis] = bmi270.gyroADCf[axis];
  }
  bmi270.gyro_accumulatedMeasurementCount++;
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

#define acc_lpf_factor 4

void accUpdate()
{
	if (!bmi270SpiAccRead(&bmi270)) {
			return;
	}

	bmi270.isAccelUpdatedAtLeastOnce = true;

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			bmi270.accADC[axis] = bmi270.accADCRaw[axis];
	}

	bmi270.accADC[X] -= 29;
	bmi270.accADC[Y] -= -35;
	bmi270.accADC[Z] -= -9;

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
