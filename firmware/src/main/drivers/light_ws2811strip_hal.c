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

#include "hw.h"

#ifdef USE_LED_STRIP

#include "common/color.h"

//#include "drivers/dma.h"
//#include "drivers/dma_reqmap.h"
//#include "drivers/io.h"
//#include "drivers/nvic.h"
//#include "drivers/rcc.h"
//#include "drivers/system.h"
//#include "drivers/timer.h"

#include "light_ws2811strip.h"

void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(void)
{

    /* Compute the prescaler value */
    uint16_t prescaler = 3;//timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = 52;//timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;



    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        /* Initialization Error */
        return false;
    }


    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return false;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (HAL_TIMEx_PWMN_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    } else {
        if (HAL_TIM_PWM_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    }

    return true;
}

void ws2811LedStripDMAEnable(void)
{
    if (DMA_SetCurrDataCounter(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = false;
        return;
    }
    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(&TimHandle,0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TimHandle,timerChannel,ENABLE);
}
#endif
