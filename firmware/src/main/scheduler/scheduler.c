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

#include "flight/failsafe.h"

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

static FAST_DATA_ZERO_INIT task_t *currentTask = NULL;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecRate;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecTime;

int32_t schedLoopStartCycles;
static int32_t schedLoopStartMinCycles;
static int32_t schedLoopStartMaxCycles;
static uint32_t schedLoopStartDeltaDownCycles;
static uint32_t schedLoopStartDeltaUpCycles;

int32_t taskGuardCycles;
static int32_t taskGuardMinCycles;
static int32_t taskGuardMaxCycles;
static uint32_t taskGuardDeltaDownCycles;
static uint32_t taskGuardDeltaUpCycles;

FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT int taskQueuePos = 0;
static FAST_DATA_ZERO_INIT int taskQueueSize = 0;

static int32_t desiredPeriodCycles;
static uint32_t lastTargetCycles;

#if defined(USE_LATE_TASK_STATISTICS)
static int16_t lateTaskCount = 0;
static uint32_t lateTaskTotal = 0;
static int16_t taskCount = 0;
static uint32_t nextTimingCycles;
#endif

static FAST_DATA_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

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
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->attribute->staticPriority < task->attribute->staticPriority) {
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
        schedulerIgnoreTaskExecTime();
    }

    if(rxRuntimeState.RxCount == 0)
    {
      ENABLE_FAILSAFE(FAILSAFE_RX_LOSS_DETECTED);
    }
    rxRuntimeState.RxCount = 0;
}

timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    taskInfo->isEnabled = queueContains(getTask(taskId));
    taskInfo->desiredPeriodUs = getTask(taskId)->attribute->desiredPeriodUs;
    taskInfo->staticPriority = getTask(taskId)->attribute->staticPriority;
    taskInfo->taskName = getTask(taskId)->attribute->taskName;
//    taskInfo->subTaskName = getTask(taskId)->attribute->subTaskName;
    taskInfo->maxExecutionTimeUs = getTask(taskId)->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = getTask(taskId)->totalExecutionTimeUs;
    taskInfo->averageExecutionTime10thUs = getTask(taskId)->movingSumExecutionTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTime10thUs = getTask(taskId)->movingSumDeltaTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTimeUs = getTask(taskId)->taskLatestDeltaTimeUs;
    taskInfo->movingAverageCycleTimeUs = getTask(taskId)->movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->lateCount = getTask(taskId)->lateCount;
    taskInfo->runCount = getTask(taskId)->runCount;
    taskInfo->execTime = getTask(taskId)->execTime;
#endif
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

// Called by tasks executing what are known to be short states
void schedulerIgnoreTaskStateTime()
{
    ignoreCurrentTaskExecRate = true;
    ignoreCurrentTaskExecTime = true;
}

// Called by tasks with state machines to only count one state as determining rate
void schedulerIgnoreTaskExecRate()
{
    ignoreCurrentTaskExecRate = true;
}

// Called by tasks without state machines executing in what is known to be a shorter time than peak
void schedulerIgnoreTaskExecTime()
{
    ignoreCurrentTaskExecTime = true;
}

bool schedulerGetIgnoreTaskExecTime()
{
    return ignoreCurrentTaskExecTime;
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

void schedulerResetTaskStatistics(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->anticipatedExecutionTime = 0;
        currentTask->movingSumDeltaTime10thUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        getTask(taskId)->anticipatedExecutionTime = 0;
        getTask(taskId)->movingSumDeltaTime10thUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{

    if (taskId == TASK_SELF) {
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
#if defined(USE_LATE_TASK_STATISTICS)
        task->lateCount = 0;
        task->runCount = 0;
#endif
    }
}

void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}

void schedulerInit(void)
{
    queueClear();

    queueAdd(getTask(TASK_SYSTEM));

    schedLoopStartMinCycles = clockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    schedLoopStartMaxCycles = clockMicrosToCycles(SCHED_START_LOOP_MAX_US);
    schedLoopStartCycles = schedLoopStartMinCycles;
    schedLoopStartDeltaDownCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
    schedLoopStartDeltaUpCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

    taskGuardMinCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
    taskGuardMaxCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
    taskGuardCycles = taskGuardMinCycles;
    taskGuardDeltaDownCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
    taskGuardDeltaUpCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

    desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->attribute->desiredPeriodUs);

    lastTargetCycles = getCycleCounter();

#if defined(USE_LATE_TASK_STATISTICS)
    nextTimingCycles = lastTargetCycles;
#endif

    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        schedulerResetTaskStatistics(taskId);
    }
}

static timeDelta_t taskNextStateTime;

FAST_CODE void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    taskNextStateTime = nextStateTime;
}

FAST_CODE timeDelta_t schedulerGetNextStateTime()
{
    return currentTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
}

void FAST_CODE scheduler(void)
{
    static uint32_t scheduleCount = 0;
    uint32_t taskExecutionTimeUs = 0;

    uint32_t currentTimeUs;
    task_t *selectedTask = NULL;

    currentTimeUs = micros();

	// Update task dynamic priorities
	for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
		if((currentTimeUs - task->lastExecutedAtUs) >= task->attribute->desiredPeriodUs) {
		  selectedTask = task;
		  if(selectedTask){
		    currentTask = selectedTask;
        ignoreCurrentTaskExecRate = false;
        ignoreCurrentTaskExecTime = false;
        taskNextStateTime = -1;
        float period = currentTimeUs - selectedTask->lastExecutedAtUs;
		    selectedTask->taskPeriodTimeUs = currentTimeUs - selectedTask->lastExecutedAtUs;
		    selectedTask->lastExecutedAtUs = currentTimeUs;

	      // Execute task
	      const uint32_t currentTimeBeforeTaskCallUs = micros();
	      selectedTask->attribute->taskFunc(currentTimeBeforeTaskCallUs);
	      taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
	      taskTotalExecutionTime += taskExecutionTimeUs;
        selectedTask->movingSumExecutionTime10thUs += (taskExecutionTimeUs * 10) - selectedTask->movingSumExecutionTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
        if (!ignoreCurrentTaskExecRate) {
            // Record task execution rate and max execution time
            selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastStatsAtUs);
            selectedTask->movingSumDeltaTime10thUs += (selectedTask->taskLatestDeltaTimeUs * 10) - selectedTask->movingSumDeltaTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
            selectedTask->lastStatsAtUs = currentTimeUs;
        }
	      //DEBUG_SET(DEBUG_NONE, taskQueuePos, (taskExecutionTimeUs));

        // Update estimate of expected task duration
        if (taskNextStateTime != -1) {
            selectedTask->anticipatedExecutionTime = taskNextStateTime << TASK_EXEC_TIME_SHIFT;
        } else if (!ignoreCurrentTaskExecTime) {
            if (taskExecutionTimeUs > (selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT)) {
                selectedTask->anticipatedExecutionTime = taskExecutionTimeUs << TASK_EXEC_TIME_SHIFT;
            } else if (selectedTask->anticipatedExecutionTime > 1) {
                // Slowly decay the max time
                selectedTask->anticipatedExecutionTime--;
            }
        }

	      selectedTask->taskExcutedEndUs = currentTimeBeforeTaskCallUs;

	      if (!ignoreCurrentTaskExecTime) {
	          selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
	      }
	      selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
	      selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
	    #if defined(USE_LATE_TASK_STATISTICS)
	      selectedTask->runCount++;
	    #endif
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
