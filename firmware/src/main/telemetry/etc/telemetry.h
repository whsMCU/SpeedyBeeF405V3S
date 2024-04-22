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

/*
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#pragma once

#include "hw.h"

#include "common/unit.h"

//#include "io/serial.h"

#include "rx/rx.h"

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    SENSOR_VOLTAGE         = 1 << 0,
    SENSOR_CURRENT         = 1 << 1,
    SENSOR_FUEL            = 1 << 2,
    SENSOR_MODE            = 1 << 3,
    SENSOR_ACC_X           = 1 << 4,
    SENSOR_ACC_Y           = 1 << 5,
    SENSOR_ACC_Z           = 1 << 6,
    SENSOR_PITCH           = 1 << 7,
    SENSOR_ROLL            = 1 << 8,
    SENSOR_HEADING         = 1 << 9,
    SENSOR_ALTITUDE        = 1 << 10,
    SENSOR_VARIO           = 1 << 11,
    SENSOR_LAT_LONG        = 1 << 12,
    SENSOR_GROUND_SPEED    = 1 << 13,
    SENSOR_DISTANCE        = 1 << 14,
    ESC_SENSOR_CURRENT     = 1 << 15,
    ESC_SENSOR_VOLTAGE     = 1 << 16,
    ESC_SENSOR_RPM         = 1 << 17,
    ESC_SENSOR_TEMPERATURE = 1 << 18,
    ESC_SENSOR_ALL         = ESC_SENSOR_CURRENT \
                            | ESC_SENSOR_VOLTAGE \
                            | ESC_SENSOR_RPM \
                            | ESC_SENSOR_TEMPERATURE,
    SENSOR_TEMPERATURE     = 1 << 19,
    SENSOR_CAP_USED        = 1 << 20,
    SENSOR_ALL             = (1 << 21) - 1,
} sensor_e;

#define IBUS_SENSOR_COUNT 15

typedef struct telemetryConfig_s {
  float gpsNoFixLatitude;
  float gpsNoFixLongitude;
  uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
  uint8_t telemetry_inverted;             // Flip the default inversion of the protocol - Same as serialrx_inverted in rx.c, but for telemetry.
  frskyGpsCoordFormat_e frsky_coordinate_format;
  frskyUnit_e frsky_unit;
  uint8_t frsky_vfas_precision;
  uint8_t frsky_pitch_roll;
  uint8_t report_cell_voltage;
  uint8_t hottAlarmSoundInterval;
  uint8_t halfDuplex;
  smartportFuelUnit_e smartportFuelUnit;
  uint8_t ibusTelemetryType;
  uint8_t ltmUpdateRate;

#ifdef USE_TELEMETRY_SIM
  int16_t simLowAltitude;
  char simGroundStationNumber[16];
  char simPin[8];
  uint16_t simTransmitInterval;
  uint8_t simTransmitFlags;

  uint16_t accEventThresholdHigh;
  uint16_t accEventThresholdLow;
  uint16_t accEventThresholdNegX;
#endif
  struct {
      uint8_t extended_status_rate;
      uint8_t rc_channels_rate;
      uint8_t position_rate;
      uint8_t extra1_rate;
      uint8_t extra2_rate;
      uint8_t extra3_rate;
      uint8_t version;
  } mavlink;
} telemetryConfig_t;


//extern serialPort_t *telemetrySharedPort;

void telemetryInit(void);
//bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider);

void telemetryCheckState(void);
void telemetryProcess(uint32_t currentTime);

//bool telemetryDetermineEnabledState(portSharing_e portSharing);

bool telemetryIsSensorEnabled(sensor_e sensor);
