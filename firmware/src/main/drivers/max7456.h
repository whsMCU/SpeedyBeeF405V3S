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

#include <stdint.h>

//#include "drivers/display.h"

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

typedef enum {
    // IO defined and MAX7456 was detected
    MAX7456_INIT_OK = 0,
    // IO defined, but MAX7456 could not be detected (maybe not yet
    // powered on)
    MAX7456_INIT_NOT_FOUND = -1,
    // No MAX7456 IO defined, which means either the we don't have it or
    // it's not properly configured
    MAX7456_INIT_NOT_CONFIGURED = -2,
} max7456InitStatus_e;

void max7456Config_Init(void);

// clockConfig values
#define MAX7456_CLOCK_CONFIG_HALF       0  // Force half clock
#define MAX7456_CLOCK_CONFIG_NOMINAL    1  // Nominal clock (default)
#define MAX7456_CLOCK_CONFIG_DOUBLE     2  // Double clock

extern uint16_t maxScreenSize;
struct vcdProfile_s;
void    max7456HardwareReset(void);

max7456InitStatus_e max7456Init(void);
void    max7456Invert(bool invert);
void    max7456Brightness(uint8_t black, uint8_t white);
bool    max7456ReInitIfRequired(bool forceStallCheck);
bool     max7456DrawScreen(void);
bool    max7456WriteNvm(uint8_t char_address, const uint8_t *font_data);
uint8_t max7456GetRowsCount(void);
void    max7456Write(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
bool    max7456DmaInProgress(void);
bool    max7456BuffersSynced(void);

bool max7456_display_string(const char *str, uint8_t x, uint8_t y);
//bool    max7456LayerSupported(displayPortLayer_e layer);
//bool    max7456LayerSelect(displayPortLayer_e layer);
//bool    max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
//bool    max7456IsDeviceDetected(void);
//void    max7456SetBackgroundType(displayPortBackground_e backgroundType);