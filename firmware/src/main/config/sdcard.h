/*
 * sdcard.h
 *
 *  Created on: Apr 14, 2024
 *      Author: WANG
 */

#ifndef SRC_MAIN_CONFIG_SDCARD_H_
#define SRC_MAIN_CONFIG_SDCARD_H_

#include "hw.h"

bool loadFromSDCard(void);
bool writeSDCard(void);
bool readSDCard(void);

#endif /* SRC_MAIN_CONFIG_SDCARD_H_ */
