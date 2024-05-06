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
 Created by Marcin Baliniak
 some functions based on MinimOSD

 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "hw.h"

#ifdef USE_OSD


#include "build/build_config.h"
#include "build/version.h"


#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"
#include "common/unit.h"

//#include "config/feature.h"

//#include "drivers/display.h"
//#include "drivers/dshot.h"
//#include "drivers/flash.h"
#include "drivers/osd/osd_symbols.h"
#include "drivers/osd/max7456.h"
//#include "drivers/sdcard.h"
//#include "drivers/time.h"

//#include "fc/core.h"
//#include "fc/rc_controls.h"
//#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
//#include "fc/stats.h"

//#include "flight/failsafe.h"
#include "flight/imu.h"
//#include "flight/mixer.h"
//#include "flight/position.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
//#include "osd/osd_warnings.h"

#include "rx/crsf.h"
#include "rx/rx.h"

#include "scheduler/scheduler.h"

//#include "sensors/acceleration.h"
#include "sensors/battery.h"
//#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"

typedef enum {
    OSD_LOGO_ARMING_OFF,
    OSD_LOGO_ARMING_ON,
    OSD_LOGO_ARMING_FIRST
} osd_logo_on_arming_e;

const char * const osdTimerSourceNames[] = {
    "ON TIME  ",
    "TOTAL ARM",
    "LAST ARM ",
    "ON/ARM   "
};

// Things in both OSD and CMS

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

timeUs_t osdFlyTime = 0;
#if defined(USE_ACC)
float osdGForce = 0;
#endif

static statistic_t stats;
timeUs_t resumeRefreshAt = 0;
#define REFRESH_1S    1000 * 1000

//STATIC_ASSERT(OSD_POS_MAX == OSD_POS(31,31), OSD_POS_MAX_incorrect);

osdConfig_t osdConfig;

osd_t osd;

osdElementConfig_t osdElementConfig;

// Controls the display order of the OSD post-flight statistics.
// Adjust the ordering here to control how the post-flight stats are presented.
// Every entry in osd_stats_e should be represented. Any that are missing will not
// be shown on the the post-flight statistics page.
// If you reorder the stats it's likely that you'll need to make likewise updates
// to the unit tests.

// If adding new stats, please add to the osdStatsNeedAccelerometer() function
// if the statistic utilizes the accelerometer.
//
const osd_stats_e osdStatsDisplayOrder[OSD_STAT_COUNT] = {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_MAX_FFT,
    OSD_STAT_MIN_RSSI_DBM,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
};

// Group elements in a number of groups to reduce task scheduling overhead
#define OSD_GROUP_COUNT                 OSD_ITEM_COUNT
// Aim to render a group of elements within a target time
#define OSD_ELEMENT_RENDER_TARGET       30
// Allow a margin by which a group render can exceed that of the sum of the elements before declaring insane
// This will most likely be violated by a USB interrupt whilst using the CLI
#if defined(STM32F411xE)
#define OSD_ELEMENT_RENDER_GROUP_MARGIN 7
#else
#define OSD_ELEMENT_RENDER_GROUP_MARGIN 2
#endif
#define OSD_TASK_MARGIN                 1
// Decay the estimated max task duration by 1/(1 << OSD_EXEC_TIME_SHIFT) on every invocation
#define OSD_EXEC_TIME_SHIFT             8

// Format a float to the specified number of decimal places with optional rounding.
// OSD symbols can optionally be placed before and after the formatted number (use SYM_NONE for no symbol).
// The formatString can be used for customized formatting of the integer part. Follow the printf style.
// Pass an empty formatString for default.
int osdPrintFloat(char *buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol)
{
    char mask[7];
    int pos = 0;
    int multiplier = 1;
    for (unsigned i = 0; i < decimalPlaces; i++) {
        multiplier *= 10;
    }

    value *= multiplier;
    const int scaledValueAbs = ABS(round ? lrintf(value) : value);
    const int integerPart = scaledValueAbs / multiplier;
    const int fractionalPart = scaledValueAbs % multiplier;

    if (leadingSymbol != SYM_NONE) {
        buffer[pos++] = leadingSymbol;
    }
    if (value < 0 && (integerPart || fractionalPart)) {
        buffer[pos++] = '-';
    }

    pos += tfp_sprintf(buffer + pos, (strlen(formatString) ? formatString : "%01u"), integerPart);
    if (decimalPlaces) {
        tfp_sprintf((char *)&mask, ".%%0%uu", decimalPlaces); // builds up the format string to be like ".%03u" for decimalPlaces == 3 as an example
        pos += tfp_sprintf(buffer + pos, mask, fractionalPart);
    }

    if (trailingSymbol != SYM_NONE) {
        buffer[pos++] = trailingSymbol;
    }
    buffer[pos] = '\0';

    return pos;
}

const uint16_t osdTimerDefault[OSD_TIMER_COUNT] = {
        OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10),
        OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10)
};

void osdConfig_Init(void)
{
    // Enable the default stats
    osdConfig.enabled_stats = 0; // reset all to off and enable only a few initially
    osdConfig.units = UNIT_METRIC;
    osdConfig.framerate_hz = 250;
    osdConfig.rcChannels[0] = 0;
    osdConfig.rcChannels[1] = 1;
    osdConfig.rcChannels[2] = 2;
    osdConfig.rcChannels[3] = 3;

    osd.spi_tx_flag = true;
}

vcdProfile_t vcdProfile;

void vcdProfile_Init(void)
{
	vcdProfile.video_system = 0;
	vcdProfile.h_offset = 0;
	vcdProfile.v_offset = 0;
}

void osdElementConfig_Init(void)
{
    // Position elements near centre of screen and disabled by default
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        osdElementConfig.item_pos[i] = OSD_POS(10, 7);
    }

    // Always enable warnings elements by default
    uint16_t profileFlags = 0;
    for (unsigned i = 1; i <= OSD_PROFILE_COUNT; i++) {
        profileFlags |= OSD_PROFILE_FLAG(i);
    }
    osdElementConfig.item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | profileFlags;

    // Default to old fixed positions for these elements
    osdElementConfig.item_pos[OSD_CROSSHAIRS]         = OSD_POS(13, 6);
    osdElementConfig.item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(14, 2);
    osdElementConfig.item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(14, 6);
    osdElementConfig.item_pos[OSD_CAMERA_FRAME]       = OSD_POS(3, 1);
    osdElementConfig.item_pos[OSD_UP_DOWN_REFERENCE]  = OSD_POS(13, 6);
    osdElementConfig.item_pos[OSD_PITCH_ANGLE]  = OSD_POS(3, 3)| profileFlags;
    osdElementConfig.item_pos[OSD_ROLL_ANGLE]  = OSD_POS(3, 6)| profileFlags;
}

void osdInit(void)
{
	max7456Init();
}

typedef struct osdStatsRenderingState_s {
    uint8_t row;
    uint8_t index;
    uint8_t rowCount;
} osdStatsRenderingState_t;

typedef enum {
    OSD_STATE_INIT,
    OSD_STATE_IDLE,
    OSD_STATE_CHECK,
    OSD_STATE_PROCESS_STATS1,
    OSD_STATE_REFRESH_STATS,
    OSD_STATE_PROCESS_STATS2,
    OSD_STATE_PROCESS_STATS3,
    OSD_STATE_UPDATE_ALARMS,
    OSD_STATE_UPDATE_CANVAS,
    OSD_STATE_GROUP_ELEMENTS,
    OSD_STATE_UPDATE_ELEMENTS,
    OSD_STATE_UPDATE_HEARTBEAT,
    OSD_STATE_COMMIT,
    OSD_STATE_TRANSFER,
    OSD_STATE_COUNT
} osdState_e;

osdState_e osdState = OSD_STATE_INIT;

#define OSD_UPDATE_INTERVAL_US (1000000 / osdConfig.framerate_hz)
typedef enum {
    OSD_Buffer_Draw = 0,
    OSD_Buffer_Draw1,
    OSD_Buffer_Draw2,
    OSD_Buffer_Draw3,
    OSD_Buffer_Draw4,
    OSD_Buffer_Draw5,
    OSD_Buffer_Draw6,
    OSD_Buffer_Draw7,
    OSD_Buffer_Draw8
} osdUpdateType_e;
// Called when there is OSD update work to be done

void osdUpdate(timeUs_t currentTimeUs)
{
  static uint8_t task = OSD_Buffer_Draw1;
  static uint32_t count = 0;

  switch(task)
  {
    case OSD_Buffer_Draw:
      osdDrawSingleElement(1, 10, OSD_ROLL_ANGLE);
      osdDrawSingleElement(1, 11, OSD_PITCH_ANGLE);
      task = OSD_Buffer_Draw1;
      break;

    case OSD_Buffer_Draw1:
      osdDrawSingleElement(21, 10, OSD_ALTITUDE);
      osdDrawSingleElement(11, 10, OSD_FLYMODE);
      task = OSD_Buffer_Draw2;
      break;

    case OSD_Buffer_Draw2:
      osdDrawSingleElement(21, 8, OSD_AVG_CELL_VOLTAGE);
      osdDrawSingleElement(20, 9, OSD_CURRENT_DRAW);
      task = OSD_Buffer_Draw3;
      break;

    case OSD_Buffer_Draw3:
      osdDrawSingleElement(16, 1, OSD_GPS_LON);
      osdDrawSingleElement(16, 2, OSD_GPS_LAT);
      task = OSD_Buffer_Draw4;
      break;

    case OSD_Buffer_Draw4:
      osdDrawSingleElement(1, 2, OSD_RSSI_VALUE);
      osdDrawSingleElement(12, 4, OSD_NUMERICAL_HEADING);
      task = OSD_Buffer_Draw5;
      break;

    case OSD_Buffer_Draw5:
      osdDrawSingleElement(10, 3, OSD_COMPASS_BAR);

      task = OSD_Buffer_Draw6;
      break;

    case OSD_Buffer_Draw6:
      osdDrawSingleElement(1, 1, OSD_LINK_QUALITY);
      osdDrawSingleElement(1, 3, OSD_RSSI_DBM_VALUE);
      task = OSD_Buffer_Draw7;
      break;

    case OSD_Buffer_Draw7:
      osdDrawSingleElement(1, 6, OSD_ROLL_PIDS);
      osdDrawSingleElement(1, 5, OSD_PITCH_PIDS);
      task = OSD_Buffer_Draw8;
      break;

    case OSD_Buffer_Draw8:
      osdDrawSingleElement(1, 7, OSD_YAW_PIDS);
      osdDrawSingleElement(1, 4, OSD_THROTTLE_POS);
      task = OSD_Buffer_Draw;
      break;

    default:
      task = OSD_Buffer_Draw;
      break;
  }

    max7456DrawScreen();

  if(count > 500)
  {
    max7456ClearLayer(0);
    count = 0;
  }
  count++;
}

statistic_t *osdGetStats(void)
{
    return &stats;
}
#endif // USE_OSD
