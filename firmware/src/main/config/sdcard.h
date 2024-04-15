/*
 * sdcard.h
 *
 *  Created on: Apr 14, 2024
 *      Author: WANG
 */

#ifndef SRC_MAIN_CONFIG_SDCARD_H_
#define SRC_MAIN_CONFIG_SDCARD_H_

#include "hw.h"

enum
{
  PID_Roll_in,
  PID_Roll_out,
  PID_pitch_in,
  PID_pitch_out,
  PID_yaw_heading,
  PID_yaw_rate,
  ACC_offset
};

bool loadFromSDCard(void);
bool writeSDCard(uint8_t type);
bool readSDCard(void);

#endif /* SRC_MAIN_CONFIG_SDCARD_H_ */
