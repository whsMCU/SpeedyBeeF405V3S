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
#include <stdlib.h>
#include <stdint.h>

#include "build/debug.h"

#include "common/utils.h"

#include "config/sdcard.h"
#include "config/feature.h"

#include "fc/dispatch.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/failsafe.h"


#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/gps/gps.h"
#include "drivers/motor.h"

#include "flight/pid.h"

#include "io/ledstrip.h"

#include "scheduler/scheduler.h"
#include "scheduler/tasks.h"

#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/opflow.h"
#include "sensors/rangefinder.h"

#include "rx/rx.h"
#include "fc/rc_controls.h"

#include "msp/fc_msp.h"
#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "osd/osd.h"

#include "scheduler/tasks.h"

#include "msp/fc_msp.h"

#include "telemetry/telemetry.h"
#include "telemetry/crsf.h"

static void ledUpdate(uint32_t currentTimeUs)
{
    static uint32_t pre_time = 0;
    if(currentTimeUs - pre_time >= 1000000)
    {
        pre_time = currentTimeUs;
        if(ARMING_FLAG(ARMED))
        {
        	ledOn(ST1);
        }
        else
        {
          LED0_TOGGLE;
        }
    }
}

uint32_t debug1;
static void debugPrint(uint32_t currentTimeUs)
{

}

static void taskHandleSerial(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    cliMain();

    // Allow MSP processing even if in CLI mode
    mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand);
}

#ifdef USE_RANGEFINDER
void taskUpdateRangefinder(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!sensors(SENSOR_RANGEFINDER))
        return;

    // Update and adjust task to update at required rate
    const uint32_t newDeadline = rangefinderUpdate();
    if (newDeadline != 0) {
        rescheduleTask(TASK_SELF, newDeadline);
    }

    /*
     * Process raw rangefinder readout
     */
    if (rangefinderProcess(getCosTiltAngle())) {
        updatePositionEstimator_SurfaceTopic(currentTimeUs, rangefinderGetLatestAltitude());
    }
}
#endif

#ifdef USE_OPFLOW
void taskUpdateOpticalFlow(timeUs_t currentTimeUs)
{
    if (!sensors(SENSOR_OPFLOW))
        return;

    opflowUpdate(currentTimeUs);
    updatePositionEstimator_OpticalFlowTopic(currentTimeUs);
}
#endif

FAST_CODE void taskFiltering(timeUs_t currentTimeUs)
{
    gyroFiltering(currentTimeUs);

}

#ifdef USE_TELEMETRY

#define GYRO_TEMP_READ_DELAY_US 3e6    // Only read the gyro temp every 3 seconds
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs)
{
    static timeUs_t lastGyroTempTimeUs = 0;

    if (cmpTimeUs(currentTimeUs, lastGyroTempTimeUs) >= GYRO_TEMP_READ_DELAY_US) {
        // Read out gyro temperature if used for telemmetry
        //gyroReadTemperature();
        lastGyroTempTimeUs = currentTimeUs;
    }
}

static void taskTelemetry(timeUs_t currentTimeUs)
{
    //if (!cliMode && featureIsEnabled(FEATURE_TELEMETRY)) {
        subTaskTelemetryPollSensors(currentTimeUs);

        telemetryProcess(currentTimeUs);
    //}
}
#endif

static void taskBatteryAlerts(uint32_t currentTimeUs)
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}

#define DEFINE_TASK(taskNameParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .taskName = taskNameParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
	  .staticPriority = staticPriorityParam \
}

// Task info in .bss (unitialised data)
task_t tasks[TASK_COUNT];

// Task ID data in .data (initialised data)
task_attribute_t task_attributes[TASK_COUNT] = {
    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
//    [TASK_MAIN] = DEFINE_TASK("SYSTEM", "UPDATE", NULL, taskMain, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", taskHandleSerial, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW), // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
    [TASK_BATTERY_ALERTS] = DEFINE_TASK("BATTERY_ALERTS", taskBatteryAlerts, TASK_PERIOD_HZ(5), TASK_PRIORITY_MEDIUM),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", batteryUpdateVoltage, TASK_PERIOD_HZ(SLOW_VOLTAGE_TASK_FREQ_HZ), TASK_PRIORITY_MEDIUM), // Freq may be updated in tasksInit
    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),

    [TASK_GYRO] = DEFINE_TASK("GYRO", taskGyroUpdate, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    [TASK_FILTER] = DEFINE_TASK("FILTER", taskFiltering, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    [TASK_PID] = DEFINE_TASK("PID", taskMainPidLoop, TASK_PERIOD_HZ(1000), TASK_PRIORITY_REALTIME),

#ifdef USE_ACC
    [TASK_ACCEL] = DEFINE_TASK("ACC", taskAccUpdate, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", imuUpdateAttitude, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
#endif

    [TASK_RX] = DEFINE_TASK("RX", taskUpdateRxMain, TASK_PERIOD_HZ(33), TASK_PRIORITY_HIGH), // If event-based scheduling doesn't work, fallback to periodic scheduling
    [TASK_DISPATCH] = DEFINE_TASK("DISPATCH", dispatchProcess, TASK_PERIOD_HZ(1000), TASK_PRIORITY_HIGH),

    [TASK_LED] = DEFINE_TASK("LED", ledUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
    [TASK_DEBUG] = DEFINE_TASK("DEBUG", debugPrint, TASK_PERIOD_HZ(50), TASK_PRIORITY_LOW),

#ifdef USE_BEEPER
    [TASK_BEEPER] = DEFINE_TASK("BEEPER", NULL, NULL, beeperUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_GPS
    [TASK_GPS] = DEFINE_TASK("GPS", gpsUpdate, TASK_PERIOD_HZ(TASK_GPS_RATE), TASK_PRIORITY_MEDIUM), // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = DEFINE_TASK("COMPASS", taskUpdateMag, TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW),
#endif

#ifdef USE_BARO
    [TASK_BARO] = DEFINE_TASK("BARO", taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", calculateEstimatedAltitude, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW),
#endif

#ifdef USE_OSD
    [TASK_OSD] = DEFINE_TASK("OSD", osdUpdate, TASK_PERIOD_HZ(OSD_FRAMERATE_DEFAULT_HZ), TASK_PRIORITY_LOW),
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", taskTelemetry, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW),
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = DEFINE_TASK("LEDSTRIP", ledStripUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_ESC_SENSOR
    [TASK_ESC_SENSOR] = DEFINE_TASK("ESC_SENSOR", escSensorProcess, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_ADC_INTERNAL
    [TASK_ADC_INTERNAL] = DEFINE_TASK("ADCINTERNAL", adcInternalProcess, TASK_PERIOD_HZ(1), TASK_PRIORITY_LOWEST),
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = DEFINE_TASK("RANGEFINDER", taskUpdateRangefinder, TASK_PERIOD_MS(70), TASK_PRIORITY_LOWEST),
#endif

#ifdef USE_OPFLOW
    [TASK_OPFLOW] = DEFINE_TASK("OPFLOW", taskUpdateOpticalFlow, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOWEST),
#endif
};

task_t *getTask(unsigned taskId)
{
    return &tasks[taskId];
}

// Has to be done before tasksInit() in order to initialize any task data which may be uninitialized at boot
void tasksInitData(void)
{
    for (int i = 0; i < TASK_COUNT; i++) {
        tasks[i].attribute = &task_attributes[i];
    }
}

void tasksInit(void)
{
    schedulerInit();

    //setTaskEnabled(TASK_MAIN, true);

    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_LED, true);
    setTaskEnabled(TASK_DEBUG, true);
    rescheduleTask(TASK_SERIAL, TASK_PERIOD_HZ(100));


	const bool useBatteryVoltage = batteryConfig.voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    // If vbat motor output compensation is used, use fast vbat samplingTime
    if (isSagCompensationConfigured()) {
        rescheduleTask(TASK_BATTERY_VOLTAGE, TASK_PERIOD_HZ(FAST_VOLTAGE_TASK_FREQ_HZ));
    }
#endif

    const bool useBatteryCurrent = batteryConfig.currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
    const bool useBatteryAlerts = batteryConfig.useVBatAlerts || batteryConfig.useConsumptionAlerts;
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

    if (sensors(SENSOR_GYRO)) {
      rescheduleTask(TASK_GYRO, bmi270.sampleLooptime);
      rescheduleTask(TASK_FILTER, bmi270.targetLooptime);
      setTaskEnabled(TASK_GYRO, true);
      setTaskEnabled(TASK_FILTER, true);
      setTaskEnabled(TASK_PID, true);
    }

#if defined(USE_ACC)
	if (sensors(SENSOR_ACC) && bmi270.accSampleRateHz) {
    setTaskEnabled(TASK_ACCEL, true);
    rescheduleTask(TASK_ACCEL, TASK_PERIOD_HZ(bmi270.accSampleRateHz));
    setTaskEnabled(TASK_ATTITUDE, true);
	}
#endif

  setTaskEnabled(TASK_RX, true);

  setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());

//#ifdef USE_BEEPER
//    setTaskEnabled(TASK_BEEPER, true);
//#endif

#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, true);
#endif

#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif
//
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, true);
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    setTaskEnabled(TASK_ALTITUDE, true);
#endif

//#ifdef USE_DASHBOARD
//    setTaskEnabled(TASK_DASHBOARD, featureIsEnabled(FEATURE_DASHBOARD));
//#endif

#ifdef USE_TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, true);
    // Reschedule telemetry to 500hz, 2ms for CRSF
    rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(250));
#endif

#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, featureConfigured(FEATURE_LED_STRIP));
#endif

//#ifdef USE_TRANSPONDER
//    setTaskEnabled(TASK_TRANSPONDER, featureIsEnabled(FEATURE_TRANSPONDER));
//#endif

#ifdef USE_OSD
    rescheduleTask(TASK_OSD, TASK_PERIOD_HZ(osdConfig.framerate_hz));
    setTaskEnabled(TASK_OSD, true);
#endif

//#ifdef USE_BST
//    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
//#endif
//
//#ifdef USE_ESC_SENSOR
//    setTaskEnabled(TASK_ESC_SENSOR, featureIsEnabled(FEATURE_ESC_SENSOR));
//#endif

#ifdef USE_ADC_INTERNAL
    setTaskEnabled(TASK_ADC_INTERNAL, true);
#endif

#ifdef USE_RANGEFINDER
    setTaskEnabled(TASK_RANGEFINDER, sensors(SENSOR_RANGEFINDER));
#endif

#ifdef USE_OPFLOW
    setTaskEnabled(TASK_OPFLOW, sensors(SENSOR_OPFLOW));
#endif

//#ifdef USE_PINIOBOX
//    pinioBoxTaskControl();
//#endif
//
//#ifdef USE_CMS
//#ifdef USE_MSP_DISPLAYPORT
//    setTaskEnabled(TASK_CMS, true);
//#else
//    setTaskEnabled(TASK_CMS, featureIsEnabled(FEATURE_OSD) || featureIsEnabled(FEATURE_DASHBOARD));
//#endif
//#endif
//
//#ifdef USE_VTX_CONTROL
//#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
//    setTaskEnabled(TASK_VTXCTRL, true);
//#endif
//#endif
//
//#ifdef USE_CAMERA_CONTROL
//    setTaskEnabled(TASK_CAMCTRL, true);
//#endif
//
//#ifdef USE_RCDEVICE
//    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
//#endif
//
//#ifdef USE_CRSF_V3
//    const bool useCRSF = rxRuntimeState.serialrxProvider == SERIALRX_CRSF;
//    setTaskEnabled(TASK_SPEED_NEGOTIATION, useCRSF);
//#endif
}

