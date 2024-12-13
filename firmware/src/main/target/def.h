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
#define _DEF_UART1          1
#define _DEF_UART2          2
#define _DEF_UART3          3
#define _DEF_UART4          4
#define _DEF_UART5          5
#define _DEF_UART6          6

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

#define USE_ACC
#define USE_BARO
#define USE_ADC
#define USE_OSD
#define USE_MAX7456
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

//#define USE_TELEMETRY
//#define USE_TELEMETRY_CRSF
//#define USE_MSP_OVER_TELEMETRY

#define USE_ADC_INTERNAL
//#define USE_RC_SMOOTHING_FILTER
#define USE_PERSISTENT_STATS

#define USE_BOARD_INFO



#endif /* SRC_COMMON_DEF_H_ */
