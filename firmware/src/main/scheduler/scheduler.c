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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"

#include "common/maths.h"

#include "rx/rx.h"
#include "rx/crsf.h"

#include "sensors/gyro_init.h"
#include "sensors/gyro.h"

#include "scheduler/scheduler.h"
#include "scheduler/tasks.h"

// DEBUG_SCHEDULER, timings for:
// 0 - Average time spent executing check function
// 1 - Time spent priortising
// 2 - time spent in scheduler

// DEBUG_SCHEDULER_DETERMINISM, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - Gyro task start cycle time in 10th of a us
// 1 - ID of late task
// 2 - Amount task is late in 10th of a us
// 3 - Gyro lock skew in 10th of a us

// DEBUG_TIMING_ACCURACY, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - % CPU busy
// 1 - Tasks late in last second
// 2 - Total lateness in last second in 10ths us
// 3 - Total tasks run in last second

static task_t *currentTask = NULL;

uint16_t averageSystemLoadPercent = 0;

static int taskQueuePos = 0;
static int taskQueueSize = 0;

static task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

static uint32_t taskTotalExecutionTime = 0;

void taskSystemLoad(uint32_t currentTimeUs)
{
    static uint32_t lastExecutedAtUs;
    int32_t deltaTime = cmpTimeUs(currentTimeUs, lastExecutedAtUs);

    // Calculate system load
    if (deltaTime) {
        averageSystemLoadPercent = 100 * taskTotalExecutionTime / deltaTime;
        taskTotalExecutionTime = 0;
        lastExecutedAtUs = currentTimeUs;
    } else {
        //schedulerIgnoreTaskExecTime();
    }
}

void rescheduleTask(taskId_e taskId, int32_t newPeriodUs)
{
    task_t *task;

    if (taskId == TASK_SELF) {
        task = currentTask;
    } else if (taskId < TASK_COUNT) {
        task = getTask(taskId);
    } else {
        return;
    }
    task->attribute->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->attribute->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

void schedulerInit(void)
{
    queueClear();

    queueAdd(getTask(TASK_SYSTEM));
}

void scheduler(void)
{
    static uint32_t scheduleCount = 0;
    uint32_t taskExecutionTimeUs = 0;

    uint32_t currentTimeUs;
    task_t *selectedTask = NULL;

	// Update task dynamic priorities
	for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
		currentTimeUs = micros();
		if(currentTimeUs - task->lastExecutedAtUs >= task->attribute->desiredPeriodUs) {
		  selectedTask = task;
			if (selectedTask) {
				currentTask = selectedTask;
				selectedTask->taskPeriodTimeUs = currentTimeUs - selectedTask->lastExecutedAtUs;
        selectedTask->lastExecutedAtUs = currentTimeUs;

				// Execute task
				const uint32_t currentTimeBeforeTaskCallUs = micros();
				selectedTask->attribute->taskFunc(currentTimeBeforeTaskCallUs);
				taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
				taskTotalExecutionTime += taskExecutionTimeUs;

				selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastStatsAtUs);
				selectedTask->lastStatsAtUs = currentTimeUs;

				selectedTask->taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
				selectedTask->taskExcutedEndUs = currentTimeBeforeTaskCallUs;
		    selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
			}
		}
	}
	rxRuntimeState.uartAvalable = uartAvailable(_DEF_UART2);
	while(uartAvailable(_DEF_UART2))
	{
		crsfDataReceive(uartRead(_DEF_UART2), (void*) &rxRuntimeState);
	}

  // Check for incoming RX data. Don't do this in the checker as that is called repeatedly within
  // a given gyro loop, and ELRS takes a long time to process this and so can only be safely processed
  // before the checkers
  rxFrameCheck(currentTimeUs, cmpTimeUs(currentTimeUs, getTask(TASK_RX)->lastExecutedAtUs));

  scheduleCount++;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}
