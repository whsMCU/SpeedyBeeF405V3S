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



#include "flight/imu.h"
//#include "flight/position.h"


#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
//#include "drivers/gps/gps.h"

#include "flight/pid.h"

#include "scheduler/scheduler.h"
#include "scheduler/tasks.h"

//#include "sensors/acceleration_init.h"
//#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"


#include "rx/rx.h"

//#include "osd/osd.h"

#include "scheduler/tasks.h"

//#include "msp/msp.h"

//#include "telemetry/telemetry.h"
//#include "telemetry/crsf.h"



// taskUpdateRxMain() has occasional peaks in execution time so normal moving average duration estimation doesn't work
// Decay the estimated max task duration by 1/(1 << RX_TASK_DECAY_SHIFT) on every invocation
#define RX_TASK_DECAY_SHIFT 6
// Add a margin to the task duration estimation
#define RX_TASK_MARGIN 1

static void ledUpdate(uint32_t currentTimeUs)
{
    static uint32_t pre_time = 0;
    if(currentTimeUs - pre_time >= 1000000)
    {
        pre_time = currentTimeUs;
        LED0_TOGGLE;
    }
}

uint8_t telemetry_tx_buf[20];

static void Encode_Msg_AHRS(void)
{
  telemetry_tx_buf[0] = 0x46;
  telemetry_tx_buf[1] = 0x43;

  telemetry_tx_buf[2] = 0x10;

  telemetry_tx_buf[3] = (short)(attitude.values.roll*10);
  telemetry_tx_buf[4] = ((short)(attitude.values.roll*10))>>8;

  telemetry_tx_buf[5] = (short)(attitude.values.pitch*10);
  telemetry_tx_buf[6] = ((short)(attitude.values.pitch*10))>>8;

  telemetry_tx_buf[7] = (unsigned short)(attitude.values.yaw*10);
  telemetry_tx_buf[8] = ((unsigned short)(attitude.values.yaw*10))>>8;

  telemetry_tx_buf[9] = (short)(baro.BaroAlt*10);
  telemetry_tx_buf[10] = ((short)(baro.BaroAlt*10))>>8;

  telemetry_tx_buf[11] = (short)((rcData[ROLL]-1500)*0.1f*100);
  telemetry_tx_buf[12] = ((short)((rcData[ROLL]-1500)*0.1f*100))>>8;

  telemetry_tx_buf[13] = (short)((rcData[PITCH]-1500)*0.1f*100);
  telemetry_tx_buf[14] = ((short)((rcData[PITCH]-1500)*0.1f*100))>>8;

  telemetry_tx_buf[15] = (unsigned short)((rcData[YAW]-1000)*0.36f*100);
  telemetry_tx_buf[16] = ((unsigned short)((rcData[YAW]-1000)*0.36f*100))>>8;

  telemetry_tx_buf[17] = (short)(rcData[THROTTLE]*10);
  telemetry_tx_buf[18] = ((short)(rcData[THROTTLE]*10))>>8;

  telemetry_tx_buf[19] = 0xff;

  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];

  uartWrite(0, &telemetry_tx_buf[0], 20);
}
static void Encode_Msg_PID_Gain(unsigned char id, float p, float i, float d)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;

	  telemetry_tx_buf[2] = id;


	  *(float*)&telemetry_tx_buf[3] = p;
	  *(float*)&telemetry_tx_buf[7] = i;
	  *(float*)&telemetry_tx_buf[11] = d;

	  telemetry_tx_buf[15] = 0x00;
	  telemetry_tx_buf[16] = 0x00;
	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];

	  uartWrite(0, &telemetry_tx_buf[0], 20);
}
static void debugPrint(uint32_t currentTimeUs)
{

	Encode_Msg_AHRS();
//    cliPrintf("BARO : %d cm, Load : %d, count : %d \n\r", baro.BaroAlt, getAverageSystemLoadPercent(), getCycleCounter());
	  //cliPrintf("excute_time : %4.d us, max : %4.d us, callback : %4.d us, uartAvalavle : %4.d \n\r", excute_time, excute_max, rxRuntimeState.callbackTime, rxRuntimeState.uartAvalable);

//    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
//        taskInfo_t taskInfo;
//        getTaskInfo(taskId, &taskInfo);
//        if (taskInfo.isEnabled) {
//            int taskFrequency = taskInfo.averageDeltaTime10thUs == 0 ? 0 : lrintf(1e7f / taskInfo.averageDeltaTime10thUs);
//            const int maxLoad = taskInfo.maxExecutionTimeUs == 0 ? 0 : (taskInfo.maxExecutionTimeUs * taskFrequency) / 1000;
//            const int averageLoad = taskInfo.averageExecutionTime10thUs == 0 ? 0 : (taskInfo.averageExecutionTime10thUs * taskFrequency) / 10000;
//            if(taskId == TASK_FILTER)
//            {
//            	filter_load = averageLoad/10;
//            }
//            if(taskId == TASK_PID)
//            {
//            	pid_load = averageLoad/10;
//            }
//    }
//    }
//    cliPrintf("IMU R: %d, P: %d, Y: %d\n\r",    attitude.values.roll,
//                                                attitude.values.pitch,
//                                                attitude.values.yaw);

//	    cliPrintf("T: %d, P: %d\n\r",    baro.baroTemperature, baro.baroPressure);

//    cliPrintf("Motor 1: %.f, 2: %.f, 3: %.f, 4: %.f\n\r",    motor[0],
//															motor[1],
//															motor[2],
//															motor[3]);
//    cliPrintf("rx: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n\r", rcData[0], rcData[1], rcData[2], rcData[3], rcData[4],
//    		rcData[5], rcData[6], rcData[7], rcData[8], rcData[9]);
//	cliPrintf("rx: %4.f, %4.f, %4.f, %4.f\n\r", rcData[0], rcData[1], rcData[2], rcData[3]);
//    cliPrintf("main: %d\n\r", micros());

//     cliPrintf("ACC R: %.1f, P: %.1f, Y: %.1f\n\r",    acc.accADC[X],
//												 acc.accADC[Y],
//												 acc.accADC[Z]);

//	     cliPrintf("ACC R: %d, P: %d, Y: %d\n\r",    acc.dev.ADCRaw[X],
//													 acc.dev.ADCRaw[Y],
//													 acc.dev.ADCRaw[Z]);
//     cliPrintf("GYRO R: %.f, P: %.f, Y: %.f\n\r",    gyro.gyroADCf[X],
//												 gyro.gyroADCf[Y],
//												 gyro.gyroADCf[Z]);

//     cliPrintf("GYRO R: %d, P: %d, Y: %d\n\r",    gyro.gyroSensor1.gyroDev.gyroADCRaw[X],
//													 gyro.gyroSensor1.gyroDev.gyroADCRaw[Y],
//													 gyro.gyroSensor1.gyroDev.gyroADCRaw[Z]);
}

static void taskHandleSerial(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    cliMain();
    //SerialCom();
    // bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
    // mspSerialProcess(evaluateMspData, mspFcProcessCommand, mspFcProcessReply);
}

static void taskBatteryAlerts(uint32_t currentTimeUs)
{
    if (true) { //!ARMING_FLAG(ARMED)
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}

#define DEFINE_TASK(taskNameParam, taskFuncParam, desiredPeriodParam) {  \
    .taskName = taskNameParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam \
}

// Task info in .bss (unitialised data)
task_t tasks[TASK_COUNT];

// Task ID data in .data (initialised data)
task_attribute_t task_attributes[TASK_COUNT] = {
//    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", "LOAD", NULL, taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
//    [TASK_MAIN] = DEFINE_TASK("SYSTEM", "UPDATE", NULL, taskMain, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", taskHandleSerial, TASK_PERIOD_HZ(100)), // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
    [TASK_BATTERY_ALERTS] = DEFINE_TASK("BATTERY_ALERTS", taskBatteryAlerts, TASK_PERIOD_HZ(5)),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", batteryUpdateVoltage, TASK_PERIOD_HZ(SLOW_VOLTAGE_TASK_FREQ_HZ)), // Freq may be updated in tasksInit
    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50)),

    [TASK_GYRO] = DEFINE_TASK("GYRO", taskGyroUpdate, TASK_GYROPID_DESIRED_PERIOD),
//    [TASK_FILTER] = DEFINE_TASK("FILTER", NULL, NULL, taskFiltering, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME),
    [TASK_PID] = DEFINE_TASK("PID", taskMainPidLoop, TASK_PERIOD_HZ(1000)),

#ifdef USE_ACC
    [TASK_ACCEL] = DEFINE_TASK("ACC", taskAccUpdate, TASK_PERIOD_HZ(1000)),
    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", imuUpdateAttitude, TASK_PERIOD_HZ(100)),
#endif

    [TASK_RX] = DEFINE_TASK("RX", taskUpdateRxMain, TASK_PERIOD_HZ(33)), // If event-based scheduling doesn't work, fallback to periodic scheduling
//    [TASK_DISPATCH] = DEFINE_TASK("DISPATCH", NULL, NULL, dispatchProcess, TASK_PERIOD_HZ(1000), TASK_PRIORITY_HIGH),

    [TASK_LED] = DEFINE_TASK("LED", ledUpdate, TASK_PERIOD_HZ(100)),
    [TASK_DEBUG] = DEFINE_TASK("DEBUG", debugPrint, TASK_PERIOD_HZ(50)),

#ifdef USE_BEEPER
    [TASK_BEEPER] = DEFINE_TASK("BEEPER", NULL, NULL, beeperUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_GPS
    [TASK_GPS] = DEFINE_TASK("GPS", NULL, NULL, gpsUpdate, TASK_PERIOD_HZ(TASK_GPS_RATE), TASK_PRIORITY_MEDIUM), // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = DEFINE_TASK("COMPASS", taskUpdateMag, TASK_PERIOD_HZ(10)),
#endif

#ifdef USE_BARO
    [TASK_BARO] = DEFINE_TASK("BARO", taskUpdateBaro, TASK_PERIOD_HZ(20)),
#endif

#if defined(USE_BARO) || defined(USE_GPS)
//    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", taskCalculateAltitude, TASK_PERIOD_HZ(40)),
#endif

#ifdef USE_OSD
//    [TASK_OSD] = DEFINE_TASK("OSD", osdUpdate, TASK_PERIOD_HZ(OSD_FRAMERATE_DEFAULT_HZ)),
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", NULL, NULL, taskTelemetry, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW),
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = DEFINE_TASK("LEDSTRIP", NULL, NULL, ledStripUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW),
#endif

#ifdef USE_ADC_INTERNAL
    [TASK_ADC_INTERNAL] = DEFINE_TASK("ADCINTERNAL", adcInternalProcess, TASK_PERIOD_HZ(1)),
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = DEFINE_TASK("RANGEFINDER", NULL, NULL, taskUpdateRangefinder, TASK_PERIOD_HZ(10), TASK_PRIORITY_LOWEST),
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
    const bool useBatteryAlerts = false;//batteryConfig.useVBatAlerts || batteryConfig.useConsumptionAlerts || featureIsEnabled(FEATURE_OSD);
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);


	rescheduleTask(TASK_GYRO, bmi270.sampleLooptime);
	//rescheduleTask(TASK_FILTER, gyro.targetLooptime);
	setTaskEnabled(TASK_GYRO, true);
	//setTaskEnabled(TASK_FILTER, true);
	setTaskEnabled(TASK_PID, true);


	setTaskEnabled(TASK_ACCEL, true);
	rescheduleTask(TASK_ACCEL, TASK_PERIOD_HZ(1000));
	setTaskEnabled(TASK_ATTITUDE, true);

//
//#ifdef USE_RANGEFINDER
//    if (sensors(SENSOR_RANGEFINDER)) {
//        setTaskEnabled(TASK_RANGEFINDER, featureIsEnabled(FEATURE_RANGEFINDER));
//    }
//#endif

    setTaskEnabled(TASK_RX, true);

//    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());
//
//#ifdef USE_BEEPER
//    setTaskEnabled(TASK_BEEPER, true);
//#endif
//
//#ifdef USE_GPS
//    setTaskEnabled(TASK_GPS, true);
//#endif
//
#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, true);
#endif
//
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, true);
#endif
//
//#if defined(USE_BARO) || defined(USE_GPS)
//    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS));
//#endif
//
//#ifdef USE_DASHBOARD
//    setTaskEnabled(TASK_DASHBOARD, featureIsEnabled(FEATURE_DASHBOARD));
//#endif
//
//#ifdef USE_TELEMETRY
//    if (featureIsEnabled(FEATURE_TELEMETRY)) {
//        setTaskEnabled(TASK_TELEMETRY, true);
//        if (rxRuntimeState.serialrxProvider == SERIALRX_JETIEXBUS) {
//            // Reschedule telemetry to 500hz for Jeti Exbus
//            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
//        } else if (rxRuntimeState.serialrxProvider == SERIALRX_CRSF) {
//            // Reschedule telemetry to 500hz, 2ms for CRSF
//            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
//        }
//    }
//#endif
//
//#ifdef USE_LED_STRIP
//    setTaskEnabled(TASK_LEDSTRIP, featureIsEnabled(FEATURE_LED_STRIP));
//#endif
//
//#ifdef USE_TRANSPONDER
//    setTaskEnabled(TASK_TRANSPONDER, featureIsEnabled(FEATURE_TRANSPONDER));
//#endif
//
//#ifdef USE_OSD
//    rescheduleTask(TASK_OSD, TASK_PERIOD_HZ(osdConfig.framerate_hz));
//    setTaskEnabled(TASK_OSD, featureIsEnabled(FEATURE_OSD) && osdGetDisplayPort(NULL));
//#endif
//
//#ifdef USE_BST
//    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
//#endif
//
//#ifdef USE_ESC_SENSOR
//    setTaskEnabled(TASK_ESC_SENSOR, featureIsEnabled(FEATURE_ESC_SENSOR));
//#endif
//
//#ifdef USE_ADC_INTERNAL
//    setTaskEnabled(TASK_ADC_INTERNAL, true);
//#endif
//
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

