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

#include "drivers/motor.h"

#include "rx/rx.h"

motorConfig_t motorConfig;

unsigned short ccr1, ccr2, ccr3, ccr4;

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

void motorWriteAll(void)
{
  if(rxRuntimeState.arming_flag == 1)
  {
 if(rxRuntimeState.failsafe_flag == 0)
	  {
		  if(rcData[THROTTLE] > 1030)
		  {
			  TIM4->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1;
			  TIM4->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
			  TIM4->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
			  TIM4->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
		  }
		  else
		  {
			  TIM4->CCR1 = 11000;
			  TIM4->CCR2 = 11000;
			  TIM4->CCR3 = 11000;
			  TIM4->CCR4 = 11000;
		  }
	  }
	  else
	  {
		  TIM5->CCR1 = 10500;
		  TIM5->CCR2 = 10500;
		  TIM5->CCR3 = 10500;
		  TIM5->CCR4 = 10500;
	  }
  }
  else
  {
	  TIM4->CCR1 = 10500;
	  TIM4->CCR2 = 10500;
	  TIM4->CCR3 = 10500;
	  TIM4->CCR4 = 10500;
  }
}

void motorDisable(void)
{

}

void motorEnable(void)
{

}

bool motorIsEnabled(void)
{
    return false;
}
