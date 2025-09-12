/*
 * def.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_COMMON_DEF_H_
#define SRC_COMMON_DEF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define _DEF_LED1           0
#define _DEF_LED2           1
#define _DEF_LED3           2
#define _DEF_LED4           3

#define _DEF_USB            0
#define _DEF_UART1          1  // GPS
#define _DEF_UART2          2  // Radio
#define _DEF_UART3          3  // Rangefinder, opticalflow
#define _DEF_UART4          4  // bluetooth
#define _DEF_UART5          5  // ESC telemetry
#define _DEF_UART6          6  // GCS

#define _DEF_SPI1             0
#define _DEF_SPI2             1
#define _DEF_SPI3             2
#define _DEF_SPI4             3

#define _DEF_LOW              0
#define _DEF_HIGH             1

#define _DEF_INPUT            0
#define _DEF_INPUT_PULLUP     1
#define _DEF_INPUT_PULLDOWN   2
#define _DEF_INPUT_IT_RISING  3
#define _DEF_OUTPUT           4
#define _DEF_OUTPUT_PULLUP    5
#define _DEF_OUTPUT_PULLDOWN  6
#define _DEF_INPUT_AF_PP      7


#define MAX_SUPPORTED_MOTORS 8


#define USE_HAL_DRIVER

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif

/********************************************************************/
/****           altitude hold                                    ****/
/********************************************************************/

  /* uncomment to disable the altitude hold feature.
   * This is useful if all of the following apply
   * + you have a baro
   * + want altitude readout
   * + do not use altitude hold feature
   * + want to save memory space
   */
  //#define SUPPRESS_BARO_ALTHOLD

/* Natural alt change for rapid pilots. It's temporary switch OFF the althold when throttle stick is out of deadband defined with ALT_HOLD_THROTTLE_NEUTRAL_ZONE
 * but if it's commented: Smooth alt change routine is activated, for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
 * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
 */
//#define ALTHOLD_FAST_THROTTLE_CHANGE

#define USE_ACC
#define USE_BARO
#define USE_VARIO
#define USE_ADC
//#define USE_OSD
//#define USE_MAX7456
#define USE_MOTOR

#define USE_MAG
#define USE_OPFLOW
#define USE_OPFLOW_MSP
#define USE_RANGEFINDER
#define USE_RANGEFINDER_MSP
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_CRSF_LINK_STATISTICS
#define USE_RX_RSSI_DBM
#define USE_RX_LINK_QUALITY_INFO
#define USE_GYRO_SLEW_LIMITER
#define USE_LATE_TASK_STATISTICS

#define USE_FAST_DATA
#define USE_DYN_NOTCH_FILTER

#define USE_LED_STRIP
#define USE_LED_STRIP_STATUS_MODE
//#define USE_TELEMETRY
//#define USE_TELEMETRY_CRSF
//#define USE_MSP_OVER_TELEMETRY

#define USE_ADC_INTERNAL
//#define USE_RC_SMOOTHING_FILTER
#define USE_PERSISTENT_STATS

#define USE_BOARD_INFO

#define NOINLINE __attribute__((noinline))

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT         __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA                   __attribute__ ((section(".fastram_data"), aligned(4)))
#define FAST_CODE
#else
#define FAST_DATA_ZERO_INIT
#define FAST_DATA
#define FAST_CODE
#endif // USE_FAST_DATA



#endif /* SRC_COMMON_DEF_H_ */
