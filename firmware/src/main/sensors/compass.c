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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hw.h"

#if defined(USE_MAG)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

//#include "config/config.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/imu.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/compass.h"
#include "sensors/sensors.h"
#include "compass.h"
#include "compass/compass_qmc5883l.h"

#ifdef _USE_HW_CLI
static void cliQmc5883l(cli_args_t *args);
#endif

#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

#define LAMBDA_MIN 0.95f // minimal adaptive forgetting factor, range: [0.90, 0.99], currently tuned for 200 Hz
                         // (TASK_COMPASS_RATE_HZ) and update rate of compassBiasEstimatorApply(), not the mag readout
                         // rate.
                         // offline evaluations showed that 10 Hz is ok, 50 Hz is better and above 100 Hz is good.
                         // TBC with online tests.
#define P0 1.0e2f        // value to initialize P(0) = diag([P0, P0, P0, P0]), typically in range: (1, 1000)

#define CALIBRATION_WAIT_US (15 * 1000 * 1000)               // wait for movement to start and trigger the calibration routine in us
#define GYRO_NORM_SQUARED_MIN sq(DEGREES_TO_RADIANS(350.0f)) // minimal value that has to be reached so that the calibration routine starts in (rad/sec)^2,
                                                             // a relatively high value so that the calibration routine is not triggered too early
#define CALIBRATION_TIME_US (30 * 1000 * 1000)               // duration of the calibration phase in us

static timeUs_t magCalEndTime = 0;
static bool didMovementStart = false;
static bool magCalProcessActive = false;

static compassBiasEstimator_t compassBiasEstimator;

magDev_t magDev;
mag_t mag;

compassConfig_t compassConfig;

// If the i2c bus is busy, try again in 500us
#define COMPASS_BUS_BUSY_INTERVAL_US 500
// If we check for new mag data, and there is none, try again in 1000us
#define COMPASS_RECHECK_INTERVAL_US 1000
// default compass read interval, for those with no specified ODR, will be TASK_COMPASS_RATE_HZ
static uint32_t compassReadIntervalUs = TASK_PERIOD_HZ(TASK_COMPASS_RATE_HZ);

void compassConfig_Init(void)
{
  compassConfig.mag_alignment = ALIGN_DEFAULT;
  memset(&compassConfig.mag_customAlignment, 0x00, sizeof(compassConfig.mag_customAlignment));
  compassConfig.mag_hardware = MAG_DEFAULT;
	compassConfig.magZero.values.roll = 326;//-352;
	compassConfig.magZero.values.pitch = 378;//-368;
	compassConfig.magZero.values.yaw = -23;//75;
}

static int16_t magADCRaw[XYZ_AXIS_COUNT];
static uint8_t magInit = 0;

bool compassDetect(magDev_t *magDev, sensor_align_e *alignment)
{
  magSensor_e magHardware = MAG_NONE;

    if (qmc5883lDetect(magDev)) {

      *alignment = CW270_DEG_FLIP;

      magHardware = MAG_QMC5883L;
      sensorsSet(SENSOR_MAG);
    }

    if (magHardware == MAG_NONE) {
        return false;
    }

    return true;
}

bool compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)

    sensor_align_e alignment;

    if (!compassDetect(&magDev, &alignment)) {
        return false;
    }

    //LED1_ON;
    magDev.init(&magDev);
    //LED1_OFF;
    magInit = 1;

    #ifdef _USE_HW_CLI
    cliAdd("qmc5883l", cliQmc5883l);
    #endif

    magDev.magAlignment = alignment;

     if (compassConfig.mag_alignment != ALIGN_DEFAULT) {
         magDev.magAlignment = compassConfig.mag_alignment;
     }

     buildRotationMatrixFromAngles(&magDev.rotationMatrix, &compassConfig.mag_customAlignment);

    ENABLE_FLIGHT_MODE(HEADING_MODE);
    magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

    return true;
}

bool compassIsHealthy(void)
{
    return (mag.magADC.x != 0) && (mag.magADC.y != 0) && (mag.magADC.z != 0);
}

void compassStartCalibration(void)
{
    // starting now, the user has CALIBRATION_WAIT_US to start moving the quad and trigger the actual calibration routine
    //beeper(BEEPER_ACC_CALIBRATION); // Beep to alert user that calibration request was received
    magCalProcessActive = true;
    magCalEndTime = micros() + CALIBRATION_WAIT_US;
    didMovementStart = false;
    // reset / update the compass bias estimator for faster convergence
    compassBiasEstimatorUpdate(&compassBiasEstimator, LAMBDA_MIN, P0);
}

bool compassIsCalibrationComplete(void)
{
    return !magCalProcessActive;
}

uint32_t compassUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t previousTaskTimeUs = 0;
    const timeDelta_t dTaskTimeUs = cmpTimeUs(currentTimeUs, previousTaskTimeUs);
    previousTaskTimeUs = currentTimeUs;
    //DEBUG_SET(DEBUG_MAG_TASK_RATE, 6, dTaskTimeUs);

    bool checkBusBusy = busBusy();
    //DEBUG_SET(DEBUG_MAG_TASK_RATE, 4, checkBusBusy);
    if (checkBusBusy) {
        // No action is taken, as the bus was busy.
        schedulerIgnoreTaskExecRate();
        return COMPASS_BUS_BUSY_INTERVAL_US; // come back in 500us, maybe the bus won't be busy then
    }

    bool checkReadState = !magDev.read(&magDev, magADCRaw);
    //DEBUG_SET(DEBUG_MAG_TASK_RATE, 5, checkReadState);
    if (checkReadState) {
        // The compass reported no data available to be retrieved; it may use a state engine that has more than one read state
        schedulerIgnoreTaskExecRate();
        return COMPASS_RECHECK_INTERVAL_US; // come back in 1ms, maybe data will be available then
    }

    // if we get here, we have new data to parse
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
      mag.magADC.v[axis] = magADCRaw[axis];
  }
    // If debug_mode is DEBUG_GPS_RESCUE_HEADING, we should update the magYaw value, after which isNewMagADCFlag will be set false
    mag.isNewMagADCFlag = true;

  if (magDev.magAlignment == ALIGN_CUSTOM) {
      alignSensorViaMatrix(&mag.magADC, &magDev.rotationMatrix);
  } else {
      alignSensorViaRotation(&mag.magADC, magDev.magAlignment);
  }

    // get stored cal/bias values
  flightDynamicsTrims_t *magZero = &compassConfig.magZero;

  // ** perform calibration, if initiated by switch or Configurator button **
    if (magCalProcessActive) {
        if (cmpTimeUs(magCalEndTime, currentTimeUs) > 0) {
                // compare squared norm of rotation rate to GYRO_NORM_SQUARED_MIN
            float gyroNormSquared = 0.0f;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                gyroNormSquared += sq(DEGREES_TO_RADIANS(gyroGetFilteredDownsampled(axis)));
            }
            if (!didMovementStart && gyroNormSquared > GYRO_NORM_SQUARED_MIN) {
                // movement has started
                //beeper(BEEPER_READY_BEEP); // Beep to alert user to start moving the quad (does this work?)
                // zero the old cal/bias values
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    magZero->raw[axis] = 0;
                }
                didMovementStart = true;
                // the user has CALIBRATION_TIME_US from now to move the quad in all directions
                magCalEndTime = micros() + CALIBRATION_TIME_US;
            }
            // start acquiring mag data and computing new cal factors
            if (didMovementStart) {
                // LED will flash at task rate while calibrating, looks like 'ON' all the time.
                LED0_ON;
                compassBiasEstimatorApply(&compassBiasEstimator, &mag.magADC);
            }
        } else {
            // mag cal process is not complete until the new cal values are saved
            if (magCalProcessActive) {
                // if movement started, accept whatever cal/bias values are available at the end of the movement period
                if (didMovementStart) {
                    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                        magZero->raw[axis] = lrintf(compassBiasEstimator.b[axis]);
                    }
                    //beeper(BEEPER_GYRO_CALIBRATED); // re-purpose gyro cal success beep
                    //saveConfigAndNotify();
                } else {
                    // there was no movement, and no new calibration values were saved
                    //beeper(BEEPER_ACC_CALIBRATION_FAIL); // calibration fail beep
                }
                // didMovementStart remains true until next run
                // signal that the calibration process is finalised, whether successful or not, by setting end time to zero
                magCalProcessActive = false;
            }
        }
    }

    // remove saved cal/bias; this is zero while calibrating
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC.v[axis] -= magZero->raw[axis];
    }

//    if (debugMode == DEBUG_MAG_CALIB) {
//        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//            // DEBUG 0-2: magADC.x, magADC.y, magADC.z
//            DEBUG_SET(DEBUG_MAG_CALIB, axis, lrintf(mag.magADC.v[axis]));
//            // DEBUG 4-6: estimated magnetometer bias, increases above zero when calibration starts
//            DEBUG_SET(DEBUG_MAG_CALIB, axis + 4, lrintf(compassBiasEstimator.b[axis]));
//        }
//        // DEBUG 3: absolute vector length of magADC, should stay constant independent of the orientation of the quad
//        DEBUG_SET(DEBUG_MAG_CALIB, 3, lrintf(vector3Norm(&mag.magADC)));
//        // DEBUG 7: adaptive forgetting factor lambda, only while analysing cal data
//        // after the transient phase it should converge to 2000
//        // set dsiplayed lambda to zero unless calibrating, to indicate start and finish in Sensors tab
//        float displayLambdaGain = 0.0f;
//        if (magCalProcessActive && didMovementStart) {
//            // map adaptive forgetting factor lambda from (lambda_min, 1.0f) -> (0, 2000)
//            const float mapLambdaGain = 1.0f / (1.0f - compassBiasEstimator.lambda_min + 1.0e-6f) * 2.0e3f;
//            displayLambdaGain = (compassBiasEstimator.lambda - compassBiasEstimator.lambda_min) * mapLambdaGain;
//        }
//        DEBUG_SET(DEBUG_MAG_CALIB, 7, lrintf(displayLambdaGain));
//    }

//    if (debugMode == DEBUG_MAG_TASK_RATE) {
//        static timeUs_t previousTimeUs = 0;
//        const timeDelta_t dataIntervalUs = cmpTimeUs(currentTimeUs, previousTimeUs); // time since last data received
//        previousTimeUs = currentTimeUs;
//        const uint16_t actualCompassDataRateHz = 1e6f / dataIntervalUs;
//        timeDelta_t executeTimeUs = micros() - currentTimeUs;
//        DEBUG_SET(DEBUG_MAG_TASK_RATE, 0, TASK_COMPASS_RATE_HZ);
//        DEBUG_SET(DEBUG_MAG_TASK_RATE, 1, actualCompassDataRateHz);
//        DEBUG_SET(DEBUG_MAG_TASK_RATE, 2, dataIntervalUs);
//        DEBUG_SET(DEBUG_MAG_TASK_RATE, 3, executeTimeUs); // time in uS to complete the mag task
//    }

    // don't do the next read check until compassReadIntervalUs has expired
    schedulerIgnoreTaskExecRate();
    return compassReadIntervalUs;
}

void taskUpdateMag(uint32_t currentTimeUs)
{
    UNUSED(currentTimeUs);
		const uint32_t newDeadline = compassUpdate(currentTimeUs);
		if (newDeadline != 0) {
				rescheduleTask(TASK_SELF, newDeadline);
		}

}

#if defined(USE_GPS) || defined(USE_MAG)
void updateMagHold(void)
{
    if (fabsf(rcData[YAW]-1500) < 15 && FLIGHT_MODE(HEADING_MODE))
    {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -GET_DIRECTION(rcControlsConfig.yaw_control_reversed);
//        if (isUpright()) {
//            rcCommand[YAW] -= dif * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
//        }
    } else
    {
      magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
    }
}
// initialize the compass bias estimator
void compassBiasEstimatorInit(compassBiasEstimator_t *cBE, const float lambda_min, const float p0)
{
    memset(cBE, 0, sizeof(*cBE)); // zero contained IEEE754 floats
    // create identity matrix
    for (unsigned i = 0; i < 4; i++) {
        cBE->U[i][i] = 1.0f;
    }

    compassBiasEstimatorUpdate(cBE, lambda_min, p0);

    cBE->lambda = lambda_min;
}

// reset / update the compass bias estimator, this can be used after the compass bias estimator did
// already run to achieve faster convergence for the next run
void compassBiasEstimatorUpdate(compassBiasEstimator_t *cBE, const float lambda_min, const float p0)
{
    cBE->lambda_min = lambda_min;
    // update diagonal entries for faster convergence
    for (unsigned i = 0; i < 4; i++) {
        cBE->D[i] = p0;
    }
}

// apply one estimation step of the compass bias estimator
void compassBiasEstimatorApply(compassBiasEstimator_t *cBE, vector3_t *mag)
{
    // update phi
    float phi[4];
    phi[0] = sq(mag->x) + sq(mag->y) + sq(mag->z);
    for (unsigned i = 0; i < 3; i++) {
        phi[i + 1] = mag->v[i];
    }

    // update e
    float e = 1.0f;
    for (unsigned i = 0; i < 4; i++) {
        e -= phi[i] * cBE->theta[i];
    }

    // U D U^T
    float f[4];
    float v[4];
    for (unsigned i = 0; i < 4; i++) {
        f[i] = 0.0f;
        for (unsigned j = 0; j <= i; j++) {
            f[i] += cBE->U[j][i] * phi[j];
        }
        v[i] = cBE->D[i] * f[i];
    }

    // first iteration
    float alpha[4];
    float k[4] = {0};
    alpha[0] = cBE->lambda + v[0] * f[0];
    cBE->D[0] /= alpha[0];
    k[0] = v[0];

    // rest of the iterations
    for (unsigned i = 1; i < 4; i++) {
        alpha[i] = alpha[i - 1] + v[i] * f[i];
        cBE->D[i] *= alpha[i - 1] / (alpha[i] * cBE->lambda);
        for (unsigned j = 0; j < i; j++) {
            float dU = -(f[i] / alpha[i - 1]) * k[j];
            k[j] += v[i] * cBE->U[j][i];
            cBE->U[j][i] += dU;
        }
        k[i] += v[i];
    }

    // parameter-update
    for (unsigned i = 0; i < 4; i++) {
        cBE->theta[i] += (k[i] / alpha[3]) * e;
    }

    // bias update
    for (unsigned i = 0; i < 3; i++) {
        cBE->b[i] = -0.5f * cBE->theta[i + 1] / cBE->theta[0];
    }

    // compute zn
    float U_v;
    float phiTrans_U_v = 0.0f;
    for (unsigned i = 0; i < 4; i++) {
        U_v = 0.0f;
        for (unsigned j = i; j < 4; j++) {
            U_v += cBE->U[i][j] * v[j];
        }
        phiTrans_U_v += phi[i] * U_v;
    }
    float zn = cBE->lambda / (cBE->lambda + phiTrans_U_v);

    // update lambda
    cBE->lambda = cBE->lambda_min + (1.0f - cBE->lambda_min) * sq(zn);
}

#endif

#ifdef _USE_HW_CLI
void cliQmc5883l(cli_args_t *args)
{
  bool ret = false;

if (args->argc == 1 && args->isStr(0, "compass_show") == true)
{
    uint32_t pre_time;
    pre_time = millis();
    while(cliKeepLoop())
    {
        if (millis()-pre_time >= 1000)
        {
            pre_time = millis();
            // cliPrintf("acc x: %d, y: %d, z: %d\n\r", x, y, z);
        }
    }
    ret = true;
    }

  if (args->argc == 3 && args->isStr(0, "mem_read") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch   = (uint8_t)args->getData(1);
    addr = (uint8_t)args->getData(2);
    addr |= 0x80;

    ret = true;
  }

    if (args->argc == 4 && args->isStr(0, "mem_write") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch     = (uint8_t)args->getData(1);
    addr   = (uint8_t)args->getData(2);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("qmc5883l show \n\r");
    cliPrintf("qmc5883l mem_read ch0:1, addr \n\r");
    cliPrintf("qmc5883l mem_write ch0:1, addr data \n\r");
  }
}
#endif

#endif // USE_MAG
