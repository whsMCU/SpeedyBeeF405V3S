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

#include <gps.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hw.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf_serial.h"

//#include "config/config.h"
#include "config/feature.h"
#include "config/sdcard.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif
//#include "drivers/vtx_common.h"
//#include "drivers/vtx_rtc6705.h"
//#include "drivers/vtx_table.h"

//#include "io/displayport_max7456.h"
#include "io/ledstrip.h"

#include "fc/board_info.h"
#include "fc/dispatch.h"
#include "fc/init.h"
#include "fc/rc.h"
//#include "fc/core.h"
//#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"
#include "fc/controlrate_profile.h"

#include "scheduler/tasks.h"

//#include "flight/failsafe.h"
#include "flight/imu.h"
//#include "flight/mixer_init.h"
//#include "flight/mixer.h"
#include "flight/pid.h"
//#include "flight/pid_init.h"
#include "flight/position.h"
//#include "flight/servos.h"

//#include "msc/emfat_file.h"
#ifdef USE_PERSISTENT_MSC_RTC
#include "msc/usbd_storage.h"
#endif

//#include "msp/msp.h"
//#include "msp/msp_serial.h"
#include "msp/fc_msp.h"

#include "navigation/navigation.h"

#include "osd/osd.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"
#include "scheduler/tasks.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/adcinternal.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
//#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/opflow.h"
#include "sensors/rangefinder.h"
//#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"

#include "flight/pid.h"

//#include "drivers/gps/gps.h"
#include "drivers/motor.h"
//#include "drivers/pwm_output.h"
#include "drivers/osd/osd.h"
#include "drivers/osd/max7456.h"

#include "fc/stats.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

static void Param_Config_Init(void);

void init(void)
{
  debugMode = DEBUG_NONE;//DEBUG_FLOW;//DEBUG_POS_EST;// DEBUG_NONE; //DEBUG_PIDLOOP
  Param_Config_Init();
  bool existing = loadFromSDCard();
  if(existing)
  {
    readSDCard();
  }

  activeAdjustmentRangeReset();

	tasksInitData();
	cliOpen(_DEF_USB, 57600);
	//uartOpen(_DEF_UART1, 115200);

	mspSerialInit();
	//mixerInit(mixerConfig.mixerMode);

	bmi270_Init();
  //Sensor_Init();

#ifdef USE_MAG
	compassInit();
#endif

	position_Init();

#ifdef USE_BARO
	Baro_Init();
#endif

#ifdef USE_ADC_INTERNAL
	adcInternalInit();
#endif

  // Finally initialize the gyro filtering
	//gyroInitFilters();

	//mixerInitProfile();

	/////////////// LED //////////////////
	LED0_ON;
	for (int i = 0; i < 10; i++)
	{
		LED0_TOGGLE;
		#if defined(USE_BEEPER)
		delay(25);
		if (!(beeperConfig.beeper_off_flags & BEEPER_GET_FLAG(BEEPER_SYSTEM_INIT))) {
			BEEP_ON;
		}
			delay(25);
			BEEP_OFF;
		#else
			delay(50);
		#endif
	}
	LED0_OFF;
	////////////////////////////////////////

	imuInit();

  DISABLE_STATE(FIXED_WING_LEGACY);
  DISABLE_STATE(MULTIROTOR);
  DISABLE_STATE(ROVER);
  DISABLE_STATE(BOAT);
  DISABLE_STATE(AIRPLANE);
  DISABLE_STATE(MOVE_FORWARD_ONLY);

  ENABLE_STATE(MULTIROTOR);
  ENABLE_STATE(ALTITUDE_CONTROL);

  // Sensors have now been detected, mspFcInit() can now be called
  // to set the boxes up
  mspFcInit();

//    failsafeInit();
//
    rxInit();

    initRcProcessing();

#ifdef USE_GPS
    gpsInit();
#endif

#ifdef USE_LED_STRIP
    ledStripInit();
    featureSet(FEATURE_LED_STRIP);
    if (featureConfigured(FEATURE_LED_STRIP)) {
        ledStripEnable();
    }
#endif

//#ifdef USE_ACC
//    if (mixerConfig.mixerMode == MIXER_GIMBAL) {
//        accStartCalibration();
//    }
//#endif
    gyroStartCalibration(false);
#ifdef USE_BARO
    baroStartCalibration();
#endif

#ifdef USE_RANGEFINDER
    rangefinderInit();
#endif

#ifdef USE_OPFLOW
    opflowInit();
#endif
    navigationInit();

    positionEstimationConfig_Init();

	batteryInit(); // always needs doing, regardless of features.

#ifdef USE_PERSISTENT_STATS
    statsInit();
#endif


//#ifdef USE_MOTOR
//    motorPostInit();
//    motorEnable();
//#endif

#ifdef USE_OSD
	osdInit();
#endif

#ifdef USE_TELEMETRY
    // Telemetry will initialise displayport and register with CMS by itself.
    telemetryInit();
#endif
    tasksInit();

    MSP_SET_MODE_RANGE(0,  0, 0, 1700, 2100); // SE : ARM
    MSP_SET_MODE_RANGE(1,  6, 1, 1700, 2100); // SA : HEADFREE
    MSP_SET_MODE_RANGE(2,  3, 2, 1300, 2100); // SB : NAV ALTHOLD
    MSP_SET_MODE_RANGE(3, 11, 2, 1700, 2100); // SB : NAV POSHOLD
    MSP_SET_MODE_RANGE(4, 33, 2, 1300, 2100); // SB : SURFACE
    MSP_SET_MODE_RANGE(5, 27, 4, 1700, 2100); // SD : FALISAFE
    MSP_SET_MODE_RANGE(6, 30, 5, 1700, 2100); // SF : HOME RESET
    int i = 0;
    uint32_t src = 0xFFFFFFFF;
    ledConfig_t *ledConfig = &ledStripStatusModeConfig.ledConfigs[i];
    *ledConfig = src;
    reevaluateLedConfig();

    setModeColor(LED_AUX_CHANNEL, 0, 10);


}

void Param_Config_Init(void)
{
//	systemConfig_Init();
//	pilotConfig_Init();
	boardConfig_Init();

	boardAlignment_Init(0, 0, 0);
//	failsafeConfig_Init();
	gyroConfig_init();

  pidInit();
	statsConfig_Init();
	motorConfig_Init();
#ifdef USE_GPS
	//gpsConfig_Init();
#endif
#ifdef USE_OPFLOW
	opflow_Init();
#endif
	barometerConfig_Init();
#ifdef USE_MAG
	compassConfig_Init();
#endif
	adcConfig_Init();
	voltageSensorADCConfig_Init();
	currentSensorADCConfig_Init();

	imuConfig_Init();
	rxConfig_Init();
	rxChannelRangeConfigs_Init();
	rxFailsafeChannelConfigs_Init();
	batteryConfig_Init();
	controlRateProfiles_Init();
//	mixerConfig_Init();
//	throttleCorrectionConfig_Init();
//	featureConfig_Init();
  navConfig_Init();
	positionConfig_Init();
	posControlConfig_Init();
	rcControlsConfig_Init();
	armingConfig_Init();
	flight3DConfig_Init();
	ledStripConfig_Init();
	ledStripStatusModeConfig_Init();
#ifdef USE_OSD
	vcdProfile_Init();
	osdConfig_Init();
	osdElementConfig_Init();
#endif
}
