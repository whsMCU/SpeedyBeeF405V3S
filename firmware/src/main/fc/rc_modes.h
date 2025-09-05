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

#include <stdbool.h>

#include "common/bitarray.h"

#define BOXID_NONE 255

typedef enum {
  BOXARM           = 0,
  BOXANGLE         = 1,
  BOXHORIZON       = 2,
  BOXNAVALTHOLD    = 3,    // old BOXBARO
  BOXHEADINGHOLD   = 4,    // old MAG
  BOXHEADFREE      = 5,
  BOXHEADADJ       = 6,
  BOXCAMSTAB       = 7,
  BOXNAVRTH        = 8,    // old GPSHOME
  BOXNAVPOSHOLD    = 9,    // old GPSHOLD
  BOXMANUAL        = 10,
  BOXBEEPERON      = 11,
  BOXLEDLOW        = 12,
  BOXLIGHTS        = 13,
  BOXNAVLAUNCH     = 14,
  BOXOSD           = 15,
  BOXTELEMETRY     = 16,
  BOXBLACKBOX      = 17,
  BOXFAILSAFE      = 18,
  BOXNAVWP         = 19,
  BOXAIRMODE       = 20,
  BOXHOMERESET     = 21,
  BOXGCSNAV        = 22,
  BOXKILLSWITCH    = 23,   // old HEADING LOCK
  BOXSURFACE       = 24,
  BOXFLAPERON      = 25,
  BOXTURNASSIST    = 26,
  BOXAUTOTRIM      = 27,
  BOXAUTOTUNE      = 28,
  BOXCAMERA1       = 29,
  BOXCAMERA2       = 30,
  BOXCAMERA3       = 31,
  BOXOSDALT1       = 32,
  BOXOSDALT2       = 33,
  BOXOSDALT3       = 34,
  BOXNAVCOURSEHOLD = 35,
  BOXBRAKING       = 36,
  BOXUSER1         = 37,
  BOXUSER2         = 38,
  BOXFPVANGLEMIX   = 39,
  BOXLOITERDIRCHN  = 40,
  BOXMSPRCOVERRIDE = 41,
  BOXPREARM        = 42,
  BOXTURTLE        = 43,
  BOXNAVCRUISE     = 44,
  BOXAUTOLEVEL     = 45,
  BOXPLANWPMISSION = 46,
  BOXSOARING       = 47,
  BOXUSER3         = 48,
  BOXUSER4         = 49,
  BOXCHANGEMISSION = 50,
  CHECKBOX_ITEM_COUNT
} boxId_e;

typedef enum {
    MODELOGIC_OR = 0,
    MODELOGIC_AND
} modeLogic_e;

// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
typedef struct boxBitmask_s { uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32]; } boxBitmask_t;

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 40

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define CHANNEL_RANGE_STEP_WIDTH 25

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + CHANNEL_RANGE_STEP_WIDTH * step)
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP_WIDTH)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP_WIDTH)

// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
typedef struct channelRange_s {
    uint8_t startStep;
    uint8_t endStep;
} channelRange_t;

typedef struct modeActivationCondition_s {
    boxId_e modeId;
    uint8_t auxChannelIndex;
    channelRange_t range;
    modeLogic_e modeLogic;
    boxId_e linkedTo;
} modeActivationCondition_t;

extern modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];

#if defined(USE_CUSTOM_BOX_NAMES)

#define MAX_BOX_USER_NAME_LENGTH 16

typedef struct modeActivationConfig_s {
    char box_user_1_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_2_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_3_name[MAX_BOX_USER_NAME_LENGTH];
    char box_user_4_name[MAX_BOX_USER_NAME_LENGTH];
} modeActivationConfig_t;

PG_DECLARE(modeActivationConfig_t, modeActivationConfig);
#endif

typedef struct modeActivationProfile_s {
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
} modeActivationProfile_t;

#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)

bool IS_RC_MODE_ACTIVE(boxId_e boxId);
void rcModeUpdate(boxBitmask_t *newState);

bool airmodeIsEnabled(void);

bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range);
void updateActivatedModes(void);
bool isModeActivationConditionPresent(boxId_e modeId);
bool isModeActivationConditionLinked(boxId_e modeId);
void removeModeActivationCondition(boxId_e modeId);
bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac);
void analyzeModeActivationConditions(void);
void MSP_SET_MODE_RANGE(uint32_t i, uint8_t boxId, uint8_t auxChannelIndex, uint32_t start, uint32_t end);
