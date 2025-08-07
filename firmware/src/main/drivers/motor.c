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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hw.h"

#include "common/maths.h"

#include "flight/failsafe.h"
#include "flight/pid.h"

#include "fc/runtime_config.h"

#include "drivers/motor.h"

#include "rx/rx.h"

motorConfig_t motorConfig;
motor_t motor;

unsigned short LF, LR, RR, RF;

//TIM4->CCR1 // RR
//TIM4->CCR2 // RF
//TIM4->CCR3 // LR
//TIM4->CCR4 // LF

//ROLL
//angle : +, gyro : +, rx : +

//PITCH
//angle : +, gyro : +, rx : +

//YAW
//angle : +, gyro : -, rx : +  //gyro mul negative sign

void motorConfig_Init(void)
{
  motorConfig.minthrottle = 1070;
  motorConfig.maxthrottle = 2000;
  motorConfig.mincommand = 1000;
  motorConfig.digitalIdleOffsetValue = 550;
  motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles
}

void motorShutdown(void)
{

}

FAST_CODE void motorWriteAll(void)
{
  if(ARMING_FLAG(ARMED))
  {
    if(failsafeFlags == 0)
    {
      if(rcData[THROTTLE] > 1030 || _PID_Test.pid_test_flag == 1)
      {
        motor.motor[R_R] = RR > 21000 ? 21000 : RR < 11000 ? 11000 : RR;
        motor.motor[R_F] = RF > 21000 ? 21000 : RF < 11000 ? 11000 : RF;
        motor.motor[L_R] = LR > 21000 ? 21000 : LR < 11000 ? 11000 : LR;
        motor.motor[L_F] = LF > 21000 ? 21000 : LF < 11000 ? 11000 : LF;
      }
      else
      {
        motor.motor[R_R] = 11000;
        motor.motor[R_F] = 11000;
        motor.motor[L_R] = 11000;
        motor.motor[L_F] = 11000;
      }
    }
    else
    {
      motor.motor[R_R] = 10500;
      motor.motor[R_F] = 10500;
      motor.motor[L_R] = 10500;
      motor.motor[L_F] = 10500;
    }
  }
  else
  {
    motor.motor[R_R] = 10500;
    motor.motor[R_F] = 10500;
    motor.motor[L_R] = 10500;
    motor.motor[L_F] = 10500;
  }
  TIM4->CCR1 = motor.motor[R_R];
  TIM4->CCR2 = motor.motor[R_F];
  TIM4->CCR3 = motor.motor[L_R];
  TIM4->CCR4 = motor.motor[L_F];
}

//TIM4->CCR1 // RR
//TIM4->CCR2 // RF
//TIM4->CCR3 // LR
//TIM4->CCR4 // LF

void motorDisable(void)
{
  TIM4->CCR1 = 10500;
  TIM4->CCR2 = 10500;
  TIM4->CCR3 = 10500;
  TIM4->CCR4 = 10500;
}

void motorEnable(void)
{

}

bool motorIsEnabled(void)
{
    return false;
}
