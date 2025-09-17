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

void WS2811_DMA_IRQHandler(void)
{
    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(void)
{
    BIT_COMPARE_1 = 118;//period / 3 * 2; //118
    BIT_COMPARE_0 = 59;//period / 3; //59

    return true;
}

void ws2811LedStripDMAEnable(void)
{
    __HAL_TIM_SET_COUNTER(&htim8,0);
    if (HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_4, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = false;
        return;
    }
}
#endif
