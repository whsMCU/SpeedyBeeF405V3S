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

#include "fc/rc_modes.h"

typedef struct box_s {
    const uint8_t boxId;            // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      // permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
} box_t;

#define PERMANENT_ID_NONE 255

#define BOX_PERMANENT_ID_USER1      47
#define BOX_PERMANENT_ID_USER2      48
#define BOX_PERMANENT_ID_USER3      57
#define BOX_PERMANENT_ID_USER4      58
#define BOX_PERMANENT_ID_NONE       255       // A permanent ID for no box mode

const box_t *findBoxByBoxId(boxId_e boxId);
const box_t *findBoxByPermanentId(uint8_t permanentId);

struct boxBitmask_s;
void packBoxModeFlags(struct boxBitmask_s * mspBoxModeFlags);
uint16_t packSensorStatus(void);
struct sbuf_s;
bool serializeBoxNamesReply(struct sbuf_s *dst);
void serializeBoxReply(struct sbuf_s *dst);
void initActiveBoxIds(void);
