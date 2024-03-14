/*
 * bsp.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#include "hw.h"

#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
static uint32_t cpuClockFrequency = 0;

void cycleCounterInit(void)
{
  cpuClockFrequency = HAL_RCC_GetSysClockFreq();

  usTicks = cpuClockFrequency / 1000000;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#define DWT_LAR
  __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
  *(DWTLAR) = DWT_LAR_UNLOCK_VALUE;

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static volatile int sysTickPending = 0;

void HAL_SYSTICK_Callback(void)
{
    sysTickUptime++;
    //sysTickValStamp = SysTick->VAL;
    //sysTickPending = 0;
    //(void)(SysTick->CTRL);
}

uint32_t getCycleCounter(void)
{
    return DWT->CYCCNT;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / usTicks;
}

// Note that this conversion is signed as this is used for periods rather than absolute timestamps
int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)usTicks;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

uint32_t millis(void)
{
  return HAL_GetTick();
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t micros(void)
{
//	register uint32_t ms, cycle_cnt;
//
//	do {
//		ms = sysTickUptime;
//		cycle_cnt = SysTick->VAL;
//	} while (ms != sysTickUptime);
//	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks; //168
	return htim5.Instance->CNT;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

int __io_putchar(int ch)
{
  uartWrite(_DEF_UART1, (uint8_t *)&ch, 1);
  return 1;
}
