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

#include "common/axis.h"
#include "common/filter.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"

#include "sensors/sensors.h"

//#include "flight/pid.h"

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

typedef struct gyro_s {
    uint16_t sampleRateHz;
    uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float scale;
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
    float gyro_accumulatedMeasurements[XYZ_AXIS_COUNT];
    float gyroPrevious[XYZ_AXIS_COUNT];
    int gyro_accumulatedMeasurementCount;
    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    uint8_t *txBuf, *rxBuf;
    float gyroZero[XYZ_AXIS_COUNT];
    int16_t gyro_offset_yaw;
    uint16_t gyroCalibrationDuration;   // Gyro calibration duration in 1/100 second
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
    int16_t temperature;
    uint8_t hardware_lpf;
    bool gyro_high_fsr;

    gyroRateKHz_e gyroRateKHz;
    gyroModeSPI_e gyroModeSPI;

    uint32_t detectedEXTI;
    uint32_t gyroLastEXTI;
    uint32_t gyroSyncEXTI;
    int32_t gyroShortPeriod;
    int32_t gyroDmaMaxDuration;
    uint32_t exit_callback_dt;
    uint32_t rx_callback_dt;

    volatile bool dataReady;

    uint16_t gyroSampleRateHz;

    float acc_1G_rec;
    uint16_t acc_1G;
    int16_t accADCRaw[XYZ_AXIS_COUNT];
    float accADC[XYZ_AXIS_COUNT];
    flightDynamicsTrims_t accelerationTrims;
    rollAndPitchTrims_t rollAndPitchTrims;
    int acc_accumulatedMeasurementCount;
    float acc_accumulatedMeasurements[XYZ_AXIS_COUNT];
    uint16_t calibratingA;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.


    bool isAccelUpdatedAtLeastOnce;
    uint16_t accSampleRateHz;
    bool acc_high_fsr;


} imu_t;

extern imu_t bmi270;

void taskGyroUpdate(timeUs_t currentTimeUs);
void gyroFiltering(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulationAverage);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool gyroIsCalibrationComplete(void);
void gyroReadTemperature(void);

bool accIsCalibrationComplete(void);
bool accHasBeenCalibrated(void);
void accStartCalibration(void);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void taskAccUpdate(timeUs_t currentTimeUs);
bool accGetAccumulationAverage(float *accumulation);

void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero);
void setAccelerationTrims(union flightDynamicsTrims_u *accelerationTrimsToUse);
