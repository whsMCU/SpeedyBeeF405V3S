/*
 * failsafe.h
 *
 *  Created on: Nov 1, 2024
 *      Author: WANG
 */

#ifndef SRC_MAIN_FLIGHT_FAILSAFE_H_
#define SRC_MAIN_FLIGHT_FAILSAFE_H_

#include "hw.h"
#include "rx/rx.h"

extern uint8_t failsafeFlags;

typedef enum {
  FAILSAFE_IDLE             = (1 << 0),
  FAILSAFE_RX_LOSS_DETECTED = (1 << 1),
  FAILSAFE_RX_SWITCH        = (1 << 2),
  FAILSAFE_BATTERY_LOW      = (1 << 3),
  FAILSAFE_LOOP_TIME        = (1 << 4)
} failsafe_Flags_t;

#define DISABLE_FAILSAFE(mask) (failsafeFlags &= ~(mask))
#define ENABLE_FAILSAFE(mask) (failsafeFlags |= (mask))
#define FAILSAFE(mask) (failsafeFlags & (mask))

#endif /* SRC_MAIN_FLIGHT_FAILSAFE_H_ */
