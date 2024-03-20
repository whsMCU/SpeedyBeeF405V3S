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
#include <stdlib.h>
#include <string.h>

//#include "build/build_config.h"
#include "build/debug.h"


#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "common/time.h"

//#include "config/config.h"

#include "rx/rx.h"
#include "rx/crsf.h"

//#include "telemetry/crsf.h"

#define CRSF_TIME_NEEDED_PER_FRAME_US   1750 // a maximally sized 64byte payload will take ~1550us, round up to 1750.
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)

#define CRSF_LINK_STATUS_UPDATE_TIMEOUT_US  250000 // 250ms, 4 Hz mode 1 telemetry

#define CRSF_FRAME_ERROR_COUNT_THRESHOLD    3

static bool crsfFrameDone = false;
static crsfFrame_t crsfFrame;
static crsfFrame_t crsfChannelDataFrame;
static uint32_t crsfChannelData[CRSF_MAX_CHANNEL];

//static serialPort_t *serialPort;
static uint32_t crsfFrameStartAtUs = 0;
static uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;
static float channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;

#ifdef USE_RX_LINK_UPLINK_POWER
#define CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT 9
// Uplink power levels by uplinkTXPower expressed in mW (250 mW is from ver >=4.00, 50 mW in a future version and for ExpressLRS)
const uint16_t uplinkTXPowerStatesMw[CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
#endif

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

/*
* SUBSET RC FRAME 0x17
*
* The structure of 0x17 frame consists of 8-bit configuration data & variable length packed channel data.
*
* definition of the configuration byte
* bits 0-4: number of first channel packed
* bits 5-6: resolution configuration of the channel data (00 -> 10 bits, 01 -> 11 bits, 10 -> 12 bits, 11 -> 13 bits)
* bit 7:    reserved

* data structure of the channel data
*  - first channel packed with specified resolution
*  - second channel packed with specified resolution
*  - third channel packed with specified resolution
*                       ...
*  - last channel packed with specified resolution
*/


static uint8_t crsfFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

static uint8_t crsfFrameCmdCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_poly_0xba(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1; ++ii) {
        crc = crc8_poly_0xba(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

uint32_t error_count;
// Receive ISR callback, called back from serial port
void crsfDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;

    static uint8_t crsfFramePosition = 0;
#if defined(USE_CRSF_V3)
    static uint8_t crsfFrameErrorCnt = 0;
#endif
    const uint32_t currentTimeUs = micros();

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = currentTimeUs - crsfFrameStartAtUs;
#endif
    if (cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
#if defined(USE_CRSF_V3)
        if (crsfFramePosition > 0) {
            // count an error if full valid frame not received within the allowed time.
            crsfFrameErrorCnt++;
        }
#endif
        crsfFramePosition = 0;
    }

    if (crsfFramePosition == 0) {
        crsfFrameStartAtUs = currentTimeUs;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : MIN(crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

    if (crsfFramePosition < fullFrameLength) {
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        if (crsfFramePosition >= fullFrameLength) {
            crsfFramePosition = 0;
            const uint8_t crc = crsfFrameCRC();
            if (crc == crsfFrame.bytes[fullFrameLength - 1]) {
#if defined(USE_CRSF_V3)
                crsfFrameErrorCnt = 0;
#endif
                switch (crsfFrame.frame.type) {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                    if (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                        rxRuntimeState->lastRcFrameTimeUs = currentTimeUs;
                        rxRuntimeState->FrameTime = rxRuntimeState->lastRcFrameTimeUs - crsfFrameStartAtUs;
                        crsfFrameDone = true;
                        memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
                    }
                    break;

#if defined(USE_TELEMETRY_CRSF) && defined(USE_MSP_OVER_TELEMETRY)
                case CRSF_FRAMETYPE_MSP_REQ:
                case CRSF_FRAMETYPE_MSP_WRITE: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    if (bufferCrsfMspFrame(frameStart, crsfFrame.frame.frameLength - 4)) {
                        crsfScheduleMspResponse(crsfFrame.frame.payload[1]);
                    }
                    break;
                }
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
                case CRSF_FRAMETYPE_DEVICE_PING:
                    crsfScheduleDeviceInfoResponse();
                    break;
                case CRSF_FRAMETYPE_DISPLAYPORT_CMD: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    crsfProcessDisplayPortCmd(frameStart);
                    break;
                }
#endif
#if defined(USE_CRSF_LINK_STATISTICS)

                case CRSF_FRAMETYPE_LINK_STATISTICS: {
                    // if to FC and 10 bytes + CRSF_FRAME_ORIGIN_DEST_SIZE
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)) {
                        const crsfLinkStatistics_t* statsFrame = (const crsfLinkStatistics_t*)&crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_LINK_STATISTICS_RX: {
                    break;
                }
                case CRSF_FRAMETYPE_LINK_STATISTICS_TX: {
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE)) {
                        const crsfLinkStatisticsTx_t* statsFrame = (const crsfLinkStatisticsTx_t*)&crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsTxFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#endif
#endif
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_COMMAND:
                    if ((crsfFrame.bytes[fullFrameLength - 2] == crsfFrameCmdCRC()) &&
                        (crsfFrame.bytes[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
                        //crsfProcessCommand(crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE);
                    }
                    break;
#endif
                default:
                    break;
                }
            } else {
            	error_count++;
#if defined(USE_CRSF_V3)
                if (crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD)
                    crsfFrameErrorCnt++;
#endif
            }
        }
#if defined(USE_CRSF_V3)
        if (crsfBaudNegotiationInProgress() || isEepromWriteInProgress()) {
            // don't count errors when negotiation or eeprom write is in progress
            crsfFrameErrorCnt = 0;
        } else if (crsfFrameErrorCnt >= CRSF_FRAME_ERROR_COUNT_THRESHOLD) {
            // fall back to default speed if speed mismatch detected
            setCrsfDefaultSpeed();
            crsfFrameErrorCnt = 0;
        }
#endif
    }
}

static uint8_t crsfFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

#if defined(USE_CRSF_LINK_STATISTICS)
    crsfCheckRssi(micros());
#endif
    if (crsfFrameDone) {
        crsfFrameDone = false;

        // unpack the RC channels
        if (crsfChannelDataFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // use ordinary RC frame structure (0x16)
            const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfChannelDataFrame.frame.payload;
            channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
            crsfChannelData[0] = rcChannels->chan0;
            crsfChannelData[1] = rcChannels->chan1;
            crsfChannelData[2] = rcChannels->chan2;
            crsfChannelData[3] = rcChannels->chan3;
            crsfChannelData[4] = rcChannels->chan4;
            crsfChannelData[5] = rcChannels->chan5;
            crsfChannelData[6] = rcChannels->chan6;
            crsfChannelData[7] = rcChannels->chan7;
            crsfChannelData[8] = rcChannels->chan8;
            crsfChannelData[9] = rcChannels->chan9;
            crsfChannelData[10] = rcChannels->chan10;
            crsfChannelData[11] = rcChannels->chan11;
            crsfChannelData[12] = rcChannels->chan12;
            crsfChannelData[13] = rcChannels->chan13;
            crsfChannelData[14] = rcChannels->chan14;
            crsfChannelData[15] = rcChannels->chan15;
        } else {
            // use subset RC frame structure (0x17)
            uint8_t readByteIndex = 0;
            const uint8_t *payload = crsfChannelDataFrame.frame.payload;

            // get the configuration byte
            uint8_t configByte = payload[readByteIndex++];

            // get the channel number of start channel
            uint8_t startChannel = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            uint8_t channelBits;
            uint16_t channelMask;
            uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;
            switch (channelRes) {
            case CRSF_SUBSET_RC_RES_CONF_10B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
                break;
            default:
            case CRSF_SUBSET_RC_RES_CONF_11B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_12B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_13B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
                break;
            }

            // do nothing for the reserved configuration bit
            configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            uint8_t numOfChannels = ((crsfChannelDataFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1) * 8) / channelBits;

            // unpack the channel data
            uint8_t bitsMerged = 0;
            uint32_t readValue = 0;
            for (uint8_t n = 0; n < numOfChannels; n++) {
                while (bitsMerged < channelBits) {
                    uint8_t readByte = payload[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                crsfChannelData[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

static float crsfReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    if (channelScale == CRSF_RC_CHANNEL_SCALE_LEGACY) {
        /* conversion from RC value to PWM
        * for 0x16 RC frame
        *       RC     PWM
        * min  172 ->  988us
        * mid  992 -> 1500us
        * max 1811 -> 2012us
        * scale factor = (2012-988) / (1811-172) = 0.62477120195241
        * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
        */
        return (channelScale * (float)crsfChannelData[chan]) + 881;
    } else {
        /* conversion from RC value to PWM
        * for 0x17 Subset RC frame
        */
        return (channelScale * (float)crsfChannelData[chan]) + 988;
    }
}

void crsfRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void crsfRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        //serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool crsfRxIsTelemetryBufEmpty(void)
{
    return telemetryBufLen == 0;
}

bool crsfRxInit(rxRuntimeState_t *rxRuntimeState)
{
    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        crsfChannelData[ii] = (16 * rxConfig.midrc) / 10 - 1408;
    }

    rxRuntimeState->channelCount = CRSF_MAX_CHANNEL;
    rxRuntimeState->rxRefreshRate = CRSF_TIME_BETWEEN_FRAMES_US; //!!TODO this needs checking

    rxRuntimeState->rcReadRawFn = crsfReadRawRC;
    rxRuntimeState->rcFrameStatusFn = crsfFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    uint32_t crsfBaudrate = CRSF_BAUDRATE;
    uartOpen(_DEF_UART2, crsfBaudrate);

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL_CRSF;
    }
    return true;
}

bool crsfRxIsActive(void)
{
    return true;
}
