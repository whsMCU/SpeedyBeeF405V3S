/*
 * core.c
 *
 *  Created on: 2026. 10. 12.
 *      Author: 왕학승
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build/debug.h"
#include "fc/core.h"
#include "flight/pid.h"

#include "sensors/gyro.h"

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gyroUpdate(currentTimeUs);
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

FAST_CODE bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE bool pidLoopReady(void)
{
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

// Function for loop trigger
FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)
{
	// DEBUG_PIDLOOP, timings for:
	// 0 - gyroUpdate()
	// 1 - subTaskPidController()
	// 2 - subTaskMotorUpdate()
	// 3 - subTaskPidSubprocesses()
	//DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);

	//subTaskRcCommand(currentTimeUs);
	subTaskPidController(currentTimeUs);
	//subTaskMotorUpdate(currentTimeUs);
	//subTaskPidSubprocesses(currentTimeUs);

	//DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));
	//DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());
}
