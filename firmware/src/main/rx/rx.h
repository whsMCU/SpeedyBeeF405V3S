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

#include "hw.h"


#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + (PWM_RANGE / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step) (PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)
#define MAX_RXFAIL_RANGE_STEP ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25)

#define DEFAULT_SERVO_MIN 1000
#define DEFAULT_SERVO_MIDDLE 1500
#define DEFAULT_SERVO_MAX 2000

#define GET_FRAME_ERR_LPF_FREQUENCY(period) (1 / (period / 10.0f))
#define FRAME_ERR_RESAMPLE_US 100000

#define RX_MAPPABLE_CHANNEL_COUNT 8
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} rxFrameState_e;

#define RX_MAPPABLE_CHANNEL_COUNT 8

typedef struct rxConfig_s {
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;                 // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t serialrx_inverted;                 // invert the serial RX protocol compared to it's default setting
    uint8_t halfDuplex;                        // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
    //ioTag_t spektrum_bind_pin_override_ioTag;
    //ioTag_t spektrum_bind_plug_ioTag;
    uint8_t spektrum_sat_bind;                 // number of bind pulses for Spektrum satellite receivers
    uint8_t spektrum_sat_bind_autoreset;       // whenever we will reset (exit) binding mode after hard reboot
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    uint16_t midrc;                            // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                         // minimum rc end
    uint16_t maxcheck;                         // maximum rc end
    uint8_t fpvCamAngleDegrees;                // Camera angle to be scaled into rc commands
    uint8_t airModeActivateThreshold;          // Throttle setpoint percent where airmode gets activated
    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
    uint8_t rssi_src_frame_errors;             // true to use frame drop flags in the rx protocol
    int8_t rssi_offset;                        // offset applied to the RSSI value before it is returned
    uint8_t rc_smoothing_mode;                 // Whether filter based rc smoothing is on or off
    uint8_t rc_smoothing_setpoint_cutoff;      // Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_feedforward_cutoff;   // Filter cutoff frequency for the feedforward filter (0 = auto)
    uint8_t rc_smoothing_throttle_cutoff;      // Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_debug_axis;           // Axis to log as debug values when debug_mode = RC_SMOOTHING
    uint8_t rc_smoothing_auto_factor_rpy;      // Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rc_smoothing_auto_factor_throttle; // Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rssi_src_frame_lpf_period;         // Period of the cutoff frequency for the source frame RSSI filter (in 0.1 s)
    uint8_t srxl2_unit_id;                     // Spektrum SRXL2 RX unit id
    uint8_t srxl2_baud_fast;                   // Select Spektrum SRXL2 fast baud rate
    uint8_t sbus_baud_fast;                    // Select SBus fast baud rate
    uint8_t crsf_use_rx_snr;                   // Use RX SNR (in dB) instead of RSSI dBm for CRSF
    uint32_t msp_override_channels_mask;       // Channels to override when the MSP override mode is enabled
    uint8_t crsf_use_negotiated_baud;          // Use negotiated baud rate for CRSF V3

    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    bool yaw_control_reversed;            // invert control direction of yaw
} rxConfig_t;

extern rxConfig_t rxConfig;

#define MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT          12
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT  8
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18

#define NON_AUX_CHANNEL_COUNT 4
#define MAX_AUX_CHANNEL_COUNT (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT
#else
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
#endif

extern const char rcChannelLetters[];

extern uint16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

extern float initialThrottleHold;

#define RSSI_SCALE_MIN 1
#define RSSI_SCALE_MAX 255

#define RSSI_SCALE_DEFAULT 100

typedef enum rc_alias {
    ROLL = 0, //rcData[0], 	ROLL
    PITCH,		//rcData[1],	PITCH
    YAW,			//rcData[2],	YAW
    THROTTLE,	//rcData[3],	THROTTLE
		ARM,			//rcData[4],	ARMED
		SA,			//rcData[5],	SA
		SB,			//rcData[6],	SB
		SC,			//rcData[7],	SC
		SD,			//rcData[8],	SD
		SF,			//rcData[9],	SF
		S1,			//rcData[10],	S1
		S2,			//rcData[11],	S2
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

typedef enum {
    RX_FAILSAFE_MODE_AUTO = 0,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_FAILSAFE_MODE_INVALID
} rxFailsafeChannelMode_e;

#define RX_FAILSAFE_MODE_COUNT 3

typedef enum {
    RX_FAILSAFE_TYPE_FLIGHT = 0,
    RX_FAILSAFE_TYPE_AUX
} rxFailsafeChannelType_e;

#define RX_FAILSAFE_TYPE_COUNT 2

typedef struct rxFailsafeChannelConfig_s {
    uint8_t mode; // See rxFailsafeChannelMode_e
    uint8_t step;
} rxFailsafeChannelConfig_t;

typedef struct rxChannelRangeConfig_s {
    uint16_t min;
    uint16_t max;
} rxChannelRangeConfig_t;

struct rxRuntimeState_s;
typedef float (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan); // used by receiver driver to return channel data
typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
typedef uint32_t rcGetFrameTimeUsFn(void);  // used to retrieve the timestamp in microseconds for the last channel data frame

typedef enum {
    RX_PROVIDER_NONE = 0,
    RX_PROVIDER_PARALLEL_PWM,
    RX_PROVIDER_PPM,
    RX_PROVIDER_SERIAL,
    RX_PROVIDER_MSP,
    RX_PROVIDER_SPI,
} rxProvider_t;

typedef struct rxRuntimeState_s {
    rxProvider_t        rxProvider;
    uint8_t             channelCount; // number of RC channels as reported by current input driver
    uint16_t            rxRefreshRate;
    rcReadRawDataFnPtr  rcReadRawFn;
    rcFrameStatusFnPtr  rcFrameStatusFn;
    rcProcessFrameFnPtr rcProcessFrameFn;
    rcGetFrameTimeUsFn *rcFrameTimeUsFn;
    uint16_t            *channelData;
    void                *frameData;
    uint32_t            lastRcFrameTimeUs;
    uint32_t 						FrameTime;
    uint32_t            callbackTime;
    uint32_t            uartAvalable;
    bool            	  RxCallback_Flag;
    uint32_t            RxCount;
    bool                rcCommand_updated;
} rxRuntimeState_t;

typedef enum {
    RSSI_SOURCE_NONE = 0,
    RSSI_SOURCE_ADC,
    RSSI_SOURCE_RX_CHANNEL,
    RSSI_SOURCE_RX_PROTOCOL,
    RSSI_SOURCE_MSP,
    RSSI_SOURCE_FRAME_ERRORS,
    RSSI_SOURCE_RX_PROTOCOL_CRSF,
} rssiSource_e;

extern rssiSource_e rssiSource;

typedef enum {
    LQ_SOURCE_NONE = 0,
    LQ_SOURCE_RX_PROTOCOL_CRSF,
    LQ_SOURCE_RX_PROTOCOL_GHST,
} linkQualitySource_e;

extern linkQualitySource_e linkQualitySource;

extern rxRuntimeState_t rxRuntimeState; //!!TODO remove this extern, only needed once for channelCount

void rxConfig_Init(void);
void rxChannelRangeConfigs_Init(void);
void rxFailsafeChannelConfigs_Init(void);

void rxInit(void);
void rxProcessPending(bool state);
bool taskUpdateRxMainInProgress(void);
bool rxUpdateCheck(uint32_t currentTimeUs, int32_t currentDeltaTimeUs);
void rxFrameCheck(uint32_t currentTimeUs, int32_t currentDeltaTimeUs);
bool rxIsReceivingSignal(void);
bool rxAreFlightChannelsValid(void);
bool calculateRxChannels(uint32_t currentTimeUs);

struct rxConfig_s;

void parseRcChannels(const char *input, struct rxConfig_s *rxConfig);

#define RSSI_MAX_VALUE 1023

void setRssiDirect(uint16_t newRssi, rssiSource_e source);
void setRssi(uint16_t rssiValue, rssiSource_e source);
void setRssiMsp(uint8_t newMspRssi);
void updateRSSI(uint32_t currentTimeUs);
uint16_t getRssi(void);
uint8_t getRssiPercent(void);
bool isRssiConfigured(void);

#define LINK_QUALITY_MAX_VALUE 1023

void setLinkQuality(bool validFrame, uint32_t currentDeltaTimeUs);
uint16_t rxGetLinkQuality(void);
void setLinkQualityDirect(uint16_t linkqualityValue);
uint16_t rxGetLinkQualityPercent(void);

int16_t getRssiDbm(void);
void setRssiDbm(int16_t newRssiDbm, rssiSource_e source);
void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source);

void rxSetRfMode(uint8_t rfModeValue);
uint8_t rxGetRfMode(void);

void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue);
uint16_t rxGetUplinkTxPwrMw(void);

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig);

void suspendRxSignal(void);
void resumeRxSignal(void);

uint16_t rxGetRefreshRate(void);

int32_t rxGetFrameDelta(int32_t *frameAgeUs);

uint32_t rxFrameTimeUs(void);

int8_t calculateThrottlePercent(void);

void taskUpdateRxMain(uint32_t currentTimeUs);
void processRxModes(uint32_t currentTimeUs);
