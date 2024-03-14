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

#include "drivers/display.h"

//#include "pg/displayport_profiles.h"

typedef struct displayPortProfile_s {
    int8_t colAdjust;
    int8_t rowAdjust;
    bool invert;
    uint8_t blackBrightness;
    uint8_t whiteBrightness;
    int8_t displayPortSerial;  // serialPortIdentifier_e

    // For attribute-rich OSDs

    uint8_t attrValues[4];     // NORMAL, INFORMATIONAL, WARNING, CRITICAL
    uint8_t useDeviceBlink;    // Use device local blink capability
} displayPortProfile_t;

#if defined(USE_MSP_DISPLAYPORT)
extern displayPortProfile_t displayPortProfileMsp;
extern void displayPortProfileMsp_Init(void);
#endif

#if defined(USE_MAX7456)
extern displayPortProfile_t displayPortProfileMax7456;
extern void displayPortProfileMax7456_Init(void);
#endif

struct vcdProfile_s;
bool max7456DisplayPortInit(const struct vcdProfile_s *vcdProfile, displayPort_t **displayPort);
