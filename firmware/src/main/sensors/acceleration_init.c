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


#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

//#include "config/config_reset.h"
//#include "config/feature.h"
//#include "config/config.h"

#include "drivers/accgyro/accgyro_spi_bmi270.h"

//#include "fc/runtime_config.h"

//#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/acceleration_init.h"
#include "sensors/acceleration.h"

#define CALIBRATING_ACC_CYCLES              400

accelerationRuntime_t accelerationRuntime;
accelerometerConfig_t accelerometerConfig;

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
	rollAndPitchTrims->values.roll = 0;
	rollAndPitchTrims->values.pitch = 0;
}

static void setConfigCalibrationCompleted(void)
{
	accelerometerConfig.accZero.values.calibrationCompleted = 1;
}

bool accHasBeenCalibrated(void)
{
    return accelerometerConfig.accZero.values.calibrationCompleted;
}

void accResetRollAndPitchTrims(void)
{
    resetRollAndPitchTrims(&accelerometerConfig.accelerometerTrims);
}

static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
    accZero->values.calibrationCompleted = 0;
}

void accelerometerConfig_init(void)
{

    accelerometerConfig.acc_lpf_hz = 10,
    accelerometerConfig.acc_hardware = ACC_BMI270,
	  accelerometerConfig.acc_high_fsr = false,

    resetRollAndPitchTrims(&accelerometerConfig.accelerometerTrims);
    resetFlightDynamicsTrims(&accelerometerConfig.accZero);
    accelerometerConfig.accZero.values.roll = 29;
    accelerometerConfig.accZero.values.pitch = -35;
    accelerometerConfig.accZero.values.yaw = -9;
}

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;


void accInitFilters(void)
{
    // Only set the lowpass cutoff if the ACC sample rate is detected otherwise
    // the filter initialization is not defined (sample rate = 0)
    accelerationRuntime.accLpfCutHz = (acc.sampleRateHz) ? accelerometerConfig.acc_lpf_hz : 0;
    if (accelerationRuntime.accLpfCutHz) {
        const uint32_t accSampleTimeUs = 1e6 / acc.sampleRateHz;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accelerationRuntime.accFilter[axis], accelerationRuntime.accLpfCutHz, accSampleTimeUs);
        }
    }
}

void accStartCalibration(void)
{
    accelerationRuntime.calibratingA = CALIBRATING_ACC_CYCLES;
}

bool accIsCalibrationComplete(void)
{
    return accelerationRuntime.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == CALIBRATING_ACC_CYCLES;
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
        a[axis] += acc.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
        accelerationRuntime.accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationRuntime.accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);
        setConfigCalibrationCompleted();

        //saveConfigAndNotify();
    }

    accelerationRuntime.calibratingA--;
}

void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };

    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationRuntime.accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationRuntime.accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationRuntime.accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (int axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += acc.accADC[axis];
            // Clear global variables for next reading
            acc.accADC[axis] = 0;
            accelerationRuntime.accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            //beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationRuntime.accelerationTrims->raw[X] = accZero_saved[X];
            accelerationRuntime.accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationRuntime.accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationRuntime.accelerationTrims->raw[X] = b[X] / 50;
        accelerationRuntime.accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationRuntime.accelerationTrims->raw[Z] = b[Z] / 50 - acc.dev.acc_1G;    // for nunchuck 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);
        setConfigCalibrationCompleted();

        //saveConfigAndNotify();
    }
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationRuntime.accelerationTrims = accelerationTrimsToUse;
}

 void applyAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
 {
     accelerometerConfig.accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
     accelerometerConfig.accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;
 }
