/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

//#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

//#include "config/feature.h"
//#include "config/parameter_group.h"
//#include "config/parameter_group_ids.h"

//#include "drivers/io.h"
//#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
//#include "drivers/rangefinder/rangefinder_srf10.h"
//#include "drivers/rangefinder/rangefinder_vl53l0x.h"
//#include "drivers/rangefinder/rangefinder_vl53l1x.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
//#include "drivers/rangefinder/rangefinder_us42.h"
//#include "drivers/rangefinder/rangefinder_tof10120_i2c.h"

//#include "fc/config.h"
#include "fc/runtime_config.h"
//#include "fc/settings.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"

#include "io/rangefinder.h"

#include "scheduler/scheduler.h"

rangefinder_t rangefinder;

#define RANGEFINDER_HARDWARE_TIMEOUT_MS         500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define RANGEFINDER_DYNAMIC_THRESHOLD           600     //Used to determine max. usable rangefinder disatance
#define RANGEFINDER_DYNAMIC_FACTOR              75

#ifdef USE_RANGEFINDER

rangefinderConfig_t rangefinderConfig;

#define SETTING_RANGEFINDER_HARDWARE_DEFAULT 0
#define SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT 0

void rangefinder_Init(void)
{
  rangefinderConfig.rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT;
  rangefinderConfig.use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT;
}

/*
 * Detect which rangefinder is present
 */
static bool rangefinderDetect(rangefinderDev_t * dev, uint8_t rangefinderHardwareToUse)
{
    rangefinderType_e rangefinderHardware = RANGEFINDER_NONE;
    requestedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardwareToUse;

#if defined(USE_RANGEFINDER_MSP)
            if (virtualRangefinderDetect(dev, &rangefinderMSPVtable)) {
                rangefinderHardware = RANGEFINDER_MSP;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VIRTUAL_TASK_PERIOD_MS));
            }
#endif

    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardware;
    sensorsSet(SENSOR_RANGEFINDER);
    return true;
}

bool rangefinderInit(void)
{
    if (!rangefinderDetect(&rangefinder.dev, rangefinderConfig.rangefinder_hardware)) {
        return false;
    }

    rangefinder.dev.init(&rangefinder.dev);
    rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    rangefinder.maxTiltCos = cos_approx(DECIDEGREES_TO_RADIANS(rangefinder.dev.detectionConeExtendedDeciDegrees / 2.0f));
    rangefinder.lastValidResponseTimeMs = millis();

    return true;
}

static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > RANGEFINDER_OUT_OF_RANGE) {// only accept samples that are in range
        filterSamples[filterSampleIndex] = newReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter5(filterSamples) : newReading;
}

/*
 * This is called periodically by the scheduler
 */
timeDelta_t rangefinderUpdate(void)
{
    if (rangefinder.dev.update) {
        rangefinder.dev.update(&rangefinder.dev);
    }

    return MS2US(rangefinder.dev.delayMs);
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
bool rangefinderProcess(float cosTiltAngle)
{
    if (rangefinder.dev.read) {
        const int32_t distance = rangefinder.dev.read(&rangefinder.dev);

        // If driver reported no new measurement - don't do anything
        if (distance == RANGEFINDER_NO_NEW_DATA) {
            return false;
        }

        if (distance >= 0) {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = distance;

            if (rangefinderConfig.use_median_filtering) {
                rangefinder.rawAltitude = applyMedianFilter(rangefinder.rawAltitude);
            }
        }
        else if (distance == RANGEFINDER_OUT_OF_RANGE) {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
        }
        else {
            // Invalid response / hardware failure
            rangefinder.rawAltitude = RANGEFINDER_HARDWARE_FAILURE;
        }
    }
    else {
        // Bad configuration
        rangefinder.rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    }

    /**
    * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
    * the altitude. Returns the computed altitude in centimeters.
    *
    * When the ground is too far away or the tilt is too large, RANGEFINDER_OUT_OF_RANGE is returned.
    */
    if (cosTiltAngle < rangefinder.maxTiltCos || rangefinder.rawAltitude < 0) {
        rangefinder.calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    } else {
        rangefinder.calculatedAltitude = rangefinder.rawAltitude * cosTiltAngle;
    }

    return true;
}

/**
 * Get the latest altitude that was computed, or RANGEFINDER_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t rangefinderGetLatestAltitude(void)
{
    return rangefinder.calculatedAltitude;
}

int32_t rangefinderGetLatestRawAltitude(void) {
    return rangefinder.rawAltitude;
}

bool rangefinderIsHealthy(void)
{
    return (millis() - rangefinder.lastValidResponseTimeMs) < RANGEFINDER_HARDWARE_TIMEOUT_MS;
}
#endif
