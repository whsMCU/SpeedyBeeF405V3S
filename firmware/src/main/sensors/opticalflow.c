/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

//#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

//#include "config/feature.h"
//#include "config/parameter_group.h"
//#include "config/parameter_group_ids.h"

//#include "drivers/io.h"
//#include "drivers/light_led.h"
//#include "drivers/time.h"

#include "drivers/opflow/opflow.h"
#include "drivers/opflow/opflow_fake.h"
#include "drivers/opflow/opflow_virtual.h"

//#include "fc/config.h"
#include "fc/runtime_config.h"
//#include "fc/settings.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/opticalflow.h"

#include "scheduler/scheduler.h"

#include "io/opflow.h"

#include "build/debug.h"

opflow_t opflow;

static float cosRotAngle = 1.0f;
static float sinRotAngle = 0.0f;
static pt2Filter_t xFlowLpf, yFlowLpf;

#ifdef USE_OPTICALFLOW
static bool opflowIsCalibrating = false;
static timeMs_t opflowCalibrationStartedAt;
static float opflowCalibrationBodyAcc;
static float opflowCalibrationFlowAcc;

#define OPFLOW_SQUAL_THRESHOLD_HIGH     35      // TBD
#define OPFLOW_SQUAL_THRESHOLD_LOW      10      // TBD
#define OPFLOW_UPDATE_TIMEOUT_US        200000  // At least 5Hz updates required
#define OPFLOW_CALIBRATE_TIME_MS        30000   // 30 second calibration time

#define SETTING_OPFLOW_HARDWARE_DEFAULT 0
#define SETTING_ALIGN_OPFLOW_DEFAULT 0
#define SETTING_OPFLOW_SCALE_DEFAULT 10.5

#define OPTICALFLOW_CALIBRATION_DURATION_MS 30000
#define RATE_SCALE_RESOLUTION (1000.0f)

#define ROTATION_GYRO_LIMIT (float)200.0

#define MAX_GYRO_SAMPLE_DELAY 10

opticalFlowConfig_t opticalFlowConfig;

// static prototypes
static void applySensorRotation(vector2_t * dst, vector2_t * src);
static void applyLPF(vector2_t * flowRates);

void opflow_Init(void)
{
  opticalFlowConfig.opflow_hardware = SETTING_OPFLOW_HARDWARE_DEFAULT;
  opticalFlowConfig.opflow_align = SETTING_ALIGN_OPFLOW_DEFAULT;
  opticalFlowConfig.opflow_scale = 10.785f;
  opticalFlowConfig.rotation = 0;
  opflow.rawFlowRates.x = 0;
  opflow.rawFlowRates.y = 0;
  opflow.processedFlowRates.x = 0;
  opflow.processedFlowRates.y = 0;
  opflow.timeStampUs = micros();

  cosRotAngle = cosf(DEGREES_TO_RADIANS(opticalFlowConfig.rotation));
  sinRotAngle = sinf(DEGREES_TO_RADIANS(opticalFlowConfig.rotation));
  //low pass filter
  if (opticalFlowConfig.flow_lpf != 0) {
      const float flowCutoffHz = (float)opticalFlowConfig.flow_lpf / 100.0f;
      const float flowGain     = pt2FilterGain(flowCutoffHz, 20 / 1000.0f);

      pt2FilterInit(&xFlowLpf, flowGain);
      pt2FilterInit(&yFlowLpf, flowGain);
  }

  opticalFlowConfig.gyroSampleDelay = 3;
  opticalFlowConfig.flip_x = 1;
}

static bool opflowDetect(opflowDev_t * dev, uint8_t opflowHardwareToUse)
{
    opticalFlowSensor_e opflowHardware = OPFLOW_NONE;
    requestedSensors[SENSOR_INDEX_OPFLOW] = opflowHardwareToUse;

    if (virtualOpflowDetect(dev, &opflowMSPVtable)) {
        opflowHardware = OPFLOW_MSP;
    }

    detectedSensors[SENSOR_INDEX_OPFLOW] = opflowHardware;
    sensorsSet(SENSOR_OPTICALFLOW);
    return true;
}

static void opflowZeroBodyGyroAcc(void)
{
    opflow.gyroBodyRateTimeUs = 0;
    opflow.gyroBodyRateAcc[X] = 0;
    opflow.gyroBodyRateAcc[Y] = 0;
}

bool opflowInit(void)
{
    if (!opflowDetect(&opflow.dev, opticalFlowConfig.opflow_hardware)) {
        return false;
    }

    if (!opflow.dev.initFn(&opflow.dev)) {
        sensorsClear(SENSOR_OPTICALFLOW);
        return false;
    }

    opflowZeroBodyGyroAcc();

    return true;
}

void opflowStartCalibration(void)
{
    opflowCalibrationStartedAt = millis();
    opflowIsCalibrating = true;
    opflowCalibrationBodyAcc = 0;
    opflowCalibrationFlowAcc = 0;
}

/*
 * This is called periodically by the scheduler
 */
void opflowUpdate(timeUs_t currentTimeUs)
{
    if (!opflow.dev.updateFn)
        return;

    if (opflow.dev.updateFn(&opflow.dev)) {
        // Indicate valid update
        opflow.isHwHealty = true;
        opflow.lastValidUpdate = currentTimeUs;
        opflow.rawQuality = opflow.dev.rawData.quality;

        // Handle state switching
        switch (opflow.flowQuality) {
            case OPFLOW_QUALITY_INVALID:
                if (opflow.dev.rawData.quality >= OPFLOW_SQUAL_THRESHOLD_HIGH) {
                    opflow.flowQuality = OPFLOW_QUALITY_VALID;
                }
                break;

            case OPFLOW_QUALITY_VALID:
                if (opflow.dev.rawData.quality <= OPFLOW_SQUAL_THRESHOLD_LOW) {
                    opflow.flowQuality = OPFLOW_QUALITY_INVALID;
                }
                break;
        }

        // Opflow updated. Assume zero valus unless further processing sets otherwise
        opflow.flowRate[X] = 0;
        opflow.flowRate[Y] = 0;
        opflow.bodyRate[X] = 0;
        opflow.bodyRate[Y] = 0;

        // In the following code we operate deg/s and do conversion to rad/s in the last step
        // Calculate body rates
        if (opflow.gyroBodyRateTimeUs > 0) {
            opflow.bodyRate[X] = opflow.gyroBodyRateAcc[X] / opflow.gyroBodyRateTimeUs;
            opflow.bodyRate[Y] = opflow.gyroBodyRateAcc[Y] / opflow.gyroBodyRateTimeUs;
        }

        // If quality of the flow from the sensor is good - process further
        if (opflow.flowQuality == OPFLOW_QUALITY_VALID) {
            const float integralToRateScaler = (opticalFlowConfig.opflow_scale > 0.01f) ? (1.0e6 / opflow.dev.rawData.deltaTime) / (float)opticalFlowConfig.opflow_scale : 0.0f;

            // Apply sensor alignment
//            applySensorAlignment(opflow.dev.rawData.flowRateRaw, opflow.dev.rawData.flowRateRaw, opticalFlowConfig.opflow_align);
            alignSensorViaRotation(&opflow.dev.rawData.flowRateRaw, CW0_DEG_FLIP);
            // Calculate flow rate and accumulated body rate
            opflow.flowRate[X] = opflow.dev.rawData.flowRateRaw.x * integralToRateScaler;
            opflow.flowRate[Y] = opflow.dev.rawData.flowRateRaw.y * integralToRateScaler;

///////////////////////////////////////////////////////////////////////////////////////////
            vector2_t raw;
            raw.x = opflow.dev.rawData.flowRateRaw.x;
            raw.y = opflow.dev.rawData.flowRateRaw.y;
            vector2_t processed;
            uint8_t delayedGyroSampleIndex;

            applySensorRotation(&processed, &raw);

            // Attenuate the optical flow when body rotation is detected
            // There is a delay between a detected gyro motion and this
            // being seen in the optical flow output
            static uint8_t gyroSampleIndex = 0;
            static float rollRate[MAX_GYRO_SAMPLE_DELAY];
            static float pitchRate[MAX_GYRO_SAMPLE_DELAY];

            const uint8_t sampleDelay = (uint8_t)constrain((int)opticalFlowConfig.gyroSampleDelay, 1, MAX_GYRO_SAMPLE_DELAY);
            gyroSampleIndex = (gyroSampleIndex + 1) % sampleDelay;
            rollRate[gyroSampleIndex] = (float)gyroGetFilteredDownsampled(X);
            pitchRate[gyroSampleIndex] = -(float)gyroGetFilteredDownsampled(Y);
            delayedGyroSampleIndex = (gyroSampleIndex + 1) % sampleDelay;
            vector2_t delayedGyroRaw = {{
                rollRate[delayedGyroSampleIndex],
                pitchRate[delayedGyroSampleIndex]
            }};
            vector2_t delayedGyroRotated;

            applySensorRotation(&delayedGyroRotated, &delayedGyroRaw);

            // Subtract the rate of body rotation (converted from dps to rad/s) from the
            // optical flow
            processed.x -= DEGREES_TO_RADIANS(delayedGyroRotated.x);
            processed.y -= DEGREES_TO_RADIANS(delayedGyroRotated.y);

            // For large rates of body rotation the velocity will be unreliable, so zero
            if (fabsf(delayedGyroRotated.x) > ROTATION_GYRO_LIMIT) {
                processed.x = 0;
            }
            if (fabsf(delayedGyroRotated.y) > ROTATION_GYRO_LIMIT) {
                processed.y = 0;
            }

            applyLPF(&processed);

            opflow.rawFlowRates = raw;
            opflow.processedFlowRates = processed;
            opflow.timeStampUs  = currentTimeUs;

///////////////////////////////////////////////////////////////////////////////////////////

            // Only update DEBUG_FLOW_RAW if flow is good
            DEBUG_SET(DEBUG_FLOW_RAW, 0, (opflow.flowRate[X])); // right +
            DEBUG_SET(DEBUG_FLOW_RAW, 1, (opflow.flowRate[Y])); // front -
            DEBUG_SET(DEBUG_FLOW_RAW, 2, (opflow.bodyRate[X])); // right -
            DEBUG_SET(DEBUG_FLOW_RAW, 3, (opflow.bodyRate[Y])); // front -
        }

        // Process calibration
        if (opflowIsCalibrating) {
            // Blink LED
            LED0_TOGGLE;

            if ((millis() - opflowCalibrationStartedAt) > OPFLOW_CALIBRATE_TIME_MS) {
                // Finish calibration if we accumulated more than 3600 deg of rotation over 30 seconds
                if (opflowCalibrationBodyAcc > 3600.0f) {
                  opticalFlowConfig.opflow_scale = opflowCalibrationFlowAcc / opflowCalibrationBodyAcc;
                    //saveConfigAndNotify();
                }

                opflowIsCalibrating = 0;
            }
            else if (opflow.flowQuality == OPFLOW_QUALITY_VALID) {
                // Ongoing calibration - accumulate body and flow rotation magniture if opflow quality is good enough
                const float invDt = 1.0e6 / opflow.dev.rawData.deltaTime;
                opflowCalibrationBodyAcc += calc_length_pythagorean_2D(opflow.bodyRate[X], opflow.bodyRate[Y]);
                opflowCalibrationFlowAcc += calc_length_pythagorean_2D(opflow.dev.rawData.flowRateRaw.x, opflow.dev.rawData.flowRateRaw.y) * invDt;
            }
        }

        // Convert to radians so NAV doesn't have to do the conversion
        opflow.bodyRate[X] = DEGREES_TO_RADIANS(opflow.bodyRate[X]);
        opflow.bodyRate[Y] = DEGREES_TO_RADIANS(opflow.bodyRate[Y]);
        opflow.flowRate[X] = DEGREES_TO_RADIANS(opflow.flowRate[X]);
        opflow.flowRate[Y] = DEGREES_TO_RADIANS(opflow.flowRate[Y]);

        // Zero out gyro accumulators to calculate rotation per flow update
        opflowZeroBodyGyroAcc();
    }
    else {
        // No new data available
        if (opflow.isHwHealty && ((currentTimeUs - opflow.lastValidUpdate) > OPFLOW_UPDATE_TIMEOUT_US)) {
            opflow.isHwHealty = false;

            opflow.flowQuality = OPFLOW_QUALITY_INVALID;
            opflow.rawQuality = 0;

            opflow.flowRate[X] = 0;
            opflow.flowRate[Y] = 0;
            opflow.bodyRate[X] = 0;
            opflow.bodyRate[Y] = 0;

            opflowZeroBodyGyroAcc();
        }
    }
}

/* Run a simple gyro update integrator to estimate average body rate between two optical flow updates */
void opflowGyroUpdateCallback(timeUs_t gyroUpdateDeltaUs)
{
    if (!opflow.isHwHealty)
        return;

    for (int axis = 0; axis < 2; axis++) {
        opflow.gyroBodyRateAcc[axis] += bmi270.gyro.gyroADCf[axis] * gyroUpdateDeltaUs;
    }

    opflow.gyroBodyRateTimeUs += gyroUpdateDeltaUs;
}

static void applySensorRotation(vector2_t * dst, vector2_t * src) {
    dst->x = (opticalFlowConfig.flip_x ? -1.0f : 1.0f) * (src->x * cosRotAngle - src->y * sinRotAngle);
    dst->y = src->x * sinRotAngle + src->y * cosRotAngle;
}

static void applyLPF(vector2_t * flowRates) {
    if (opticalFlowConfig.flow_lpf == 0) {
        return;
    }

    flowRates->x = pt2FilterApply(&xFlowLpf, flowRates->x);
    flowRates->y = pt2FilterApply(&yFlowLpf, flowRates->y);
}

const opflow_t * getOpticalFlowData(void) {
    return &opflow;
}

bool opflowIsHealthy(void)
{
    return opflow.isHwHealty;
}
#endif
