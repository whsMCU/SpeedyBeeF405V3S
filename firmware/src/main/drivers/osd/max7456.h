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
#include "drivers/osd/max7456registers.h"
//#include "drivers/display.h"

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

typedef enum {
    DISPLAYPORT_LAYER_FOREGROUND,
    DISPLAYPORT_LAYER_BACKGROUND,
    DISPLAYPORT_LAYER_COUNT,
} displayPortLayer_e;

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

typedef struct max7456Register_s {
	bool _isActivatedOsd;

	REG_VM0   _regVm0;
	REG_VM1   _regVm1;
	REG_HOS   _regHos;
	REG_VOS   _regVos;
	REG_DMM   _regDmm;
	REG_DMAH  _regDmah; // not used yet
	REG_DMAL  _regDmal; // not used yet
	REG_DMDI  _regDmdi; // not used yet
	REG_CMM   _regCmm;
	REG_CMAH  _regCmah; // not used yet
	REG_CMAL  _regCmal; // not used yet
	REG_CMDI  _regCmdi; // not used yet
	REG_OSDM  _regOsdm; // not used yet
	REG_RBN   _regRb[16];  // not used yet
	REG_OSDBL _regOsdbl; // not used yet
	REG_STAT  _regStat; // not used yet
	DMDO  _regDmdo; // not used yet
	REG_CMDO  _regCmdo; // not used yet
} max7456Register_t;

void max7456Config_Init(void);

max7456InitStatus_e max7456Init(void);
void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
void max7456Write(uint8_t x, uint8_t y, const char *buff);
bool max7456WriteNvm(uint8_t char_address, const uint8_t *font_data);

bool max7456DmaInProgress(void);

bool    max7456LayerSupported(displayPortLayer_e layer);
bool    max7456LayerSelect(displayPortLayer_e layer);
bool    max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
void max7456ClearLayer(displayPortLayer_e layer);
void DrawOSD(void);
bool max7456DrawScreen(void);
bool max7456DrawScreen_test(void);
