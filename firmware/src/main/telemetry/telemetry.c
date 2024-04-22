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
#include <stdlib.h>

#include "hw.h"

#ifdef USE_TELEMETRY

#include "common/utils.h"
#include "common/unit.h"

//#include "config/parameter_group.h"
//#include "config/parameter_group_ids.h"

//#include "drivers/serial.h"

//#include "fc/config.h"
#include "fc/rc_controls.h"
//#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
//#include "fc/settings.h"

//#include "io/serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
//#include "telemetry/frsky_d.h"
//#include "telemetry/hott.h"
//#include "telemetry/smartport.h"
//#include "telemetry/ltm.h"
//#include "telemetry/mavlink.h"
//#include "telemetry/jetiexbus.h"
//#include "telemetry/ibus.h"
#include "telemetry/crsf.h"
//#include "telemetry/srxl.h"
//#include "telemetry/sim.h"
//#include "telemetry/ghst.h"


telemetryConfig_t telemetryConfig;

static void telemetryConfig_Init(void)
{

  telemetryConfig.gpsNoFixLatitude = 0;
  telemetryConfig.gpsNoFixLongitude = 0;
  telemetryConfig.telemetry_switch = 0;
  telemetryConfig.telemetry_inverted = false;
  telemetryConfig.halfDuplex = 1;
  telemetryConfig.frsky_coordinate_format = FRSKY_FORMAT_DMS;
  telemetryConfig.frsky_unit = UNIT_METRIC;
  telemetryConfig.frsky_vfas_precision = 0;
  telemetryConfig.hottAlarmSoundInterval = 5;
  telemetryConfig.report_cell_voltage = true;

}

void telemetryInit(void)
{
#if defined(USE_TELEMETRY_FRSKY)
    initFrSkyTelemetry();
#endif

#if defined(USE_TELEMETRY_HOTT)
    initHoTTTelemetry();
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    initSmartPortTelemetry();
#endif

#if defined(USE_TELEMETRY_LTM)
    initLtmTelemetry();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    initMAVLinkTelemetry();
#endif

#if defined(USE_TELEMETRY_JETIEXBUS)
    initJetiExBusTelemetry();
#endif

#if defined(USE_TELEMETRY_IBUS)
    initIbusTelemetry();
#endif

#if defined(USE_TELEMETRY_SIM)
    initSimTelemetry();
#endif

#if defined(USE_TELEMETRY_CRSF)
    initCrsfTelemetry();
#endif

#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif

#ifdef USE_TELEMETRY_GHST
    initGhstTelemetry();
#endif

    telemetryCheckState();
}

//bool telemetryDetermineEnabledState(portSharing_e portSharing)
//{
//    bool enabled = portSharing == PORTSHARING_NOT_SHARED;
//
//    if (portSharing == PORTSHARING_SHARED) {
//        if (telemetryConfig()->telemetry_switch)
//            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
//        else
//            enabled = ARMING_FLAG(ARMED);
//    }
//
//    return enabled;
//}

//bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
//{
//    return portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK;
//}
//
//serialPort_t *telemetrySharedPort = NULL;

void telemetryCheckState(void)
{
#if defined(USE_TELEMETRY_FRSKY)
    checkFrSkyTelemetryState();
#endif

#if defined(USE_TELEMETRY_HOTT)
    checkHoTTTelemetryState();
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    checkSmartPortTelemetryState();
#endif

#if defined(USE_TELEMETRY_LTM)
    checkLtmTelemetryState();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    checkMAVLinkTelemetryState();
#endif

#if defined(USE_TELEMETRY_JETIEXBUS)
    checkJetiExBusTelemetryState();
#endif

#if defined(USE_TELEMETRY_IBUS)
    checkIbusTelemetryState();
#endif

#if defined(USE_TELEMETRY_SIM)
    checkSimTelemetryState();
#endif

#if defined(USE_TELEMETRY_CRSF)
    checkCrsfTelemetryState();
#endif

#ifdef USE_TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef USE_TELEMETRY_GHST
    checkGhstTelemetryState();
#endif
}

void telemetryProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs); // since not used by all the telemetry protocols

#if defined(USE_TELEMETRY_FRSKY)
    handleFrSkyTelemetry();
#endif

#if defined(USE_TELEMETRY_HOTT)
    handleHoTTTelemetry(currentTimeUs);
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    handleSmartPortTelemetry();
#endif

#if defined(USE_TELEMETRY_LTM)
    handleLtmTelemetry();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    handleMAVLinkTelemetry(currentTimeUs);
#endif

#if defined(USE_TELEMETRY_JETIEXBUS)
    handleJetiExBusTelemetry();
#endif

#if defined(USE_SERIALRX_IBUS) && defined(USE_TELEMETRY_IBUS)
    handleIbusTelemetry();
#endif

#if defined(USE_TELEMETRY_SIM)
    handleSimTelemetry();
#endif

#if defined(USE_TELEMETRY_CRSF)
    handleCrsfTelemetry(currentTimeUs);
#endif

#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTimeUs);
#endif
#ifdef USE_TELEMETRY_GHST
    handleGhstTelemetry(currentTimeUs);
#endif
}

#endif
