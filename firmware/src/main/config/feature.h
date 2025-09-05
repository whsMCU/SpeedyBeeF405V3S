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

#pragma once

//#include "config/parameter_group.h"

typedef enum {
    FEATURE_THR_VBAT_COMP = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_TX_PROF_SEL = 1 << 2,       // Profile selection by TX stick command
    FEATURE_BAT_PROFILE_AUTOSWITCH = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_UNUSED_1 = 1 << 5,   // was FEATURE_SERVO_TILT was FEATURE_DYNAMIC_FILTERS
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_UNUSED_3 = 1 << 8,        // was FEATURE_FAILSAFE
    FEATURE_UNUSED_4 = 1 << 9,          // was FEATURE_SONAR
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_CURRENT_METER = 1 << 11,
    FEATURE_REVERSIBLE_MOTORS = 1 << 12,
    FEATURE_UNUSED_5 = 1 << 13,         // RX_PARALLEL_PWM
    FEATURE_UNUSED_6 = 1 << 14,         // RX_MSP
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_UNUSED_7 = 1 << 18,         // Unused in INAV
    FEATURE_BLACKBOX = 1 << 19,
    FEATURE_UNUSED_10 = 1 << 20,        // was FEATURE_CHANNEL_FORWARDING
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
    FEATURE_SUPEREXPO_RATES = 1 << 23,
    FEATURE_VTX = 1 << 24,
    FEATURE_UNUSED_8 = 1 << 25,         // RX_SPI
    FEATURE_UNUSED_9 = 1 << 26,         // SOFTSPI
    FEATURE_UNUSED_11 = 1 << 27,        // FEATURE_PWM_SERVO_DRIVER
    FEATURE_PWM_OUTPUT_ENABLE = 1 << 28,
    FEATURE_OSD = 1 << 29,
    FEATURE_FW_LAUNCH = 1 << 30,
    FEATURE_FW_AUTOTRIM = 1 << 31,
} features_e;

typedef struct featureConfig_s {
    uint32_t enabledFeatures;
} featureConfig_t;


void latchActiveFeatures(void);
bool featureConfigured(uint32_t mask);
bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);
