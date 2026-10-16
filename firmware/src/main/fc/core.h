/*
 * core.h
 *
 *  Created on: 2026. 10. 12.
 *      Author: 왕학승
 */

#ifndef SRC_MAIN_FC_CORE_H_
#define SRC_MAIN_FC_CORE_H_

#include "hw.h"

#include "common/time.h"

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs);

FAST_CODE bool gyroFilterReady(void);

FAST_CODE bool pidLoopReady(void);

FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs);

#endif /* SRC_MAIN_FC_CORE_H_ */
