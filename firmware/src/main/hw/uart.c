/*
 * uart.c
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */


#include "uart.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"
#include "rx/rx.h"
#include "rx/crsf.h"

#include "drivers/gps/gps.h"

#include "msp/msp_serial.h"

//#include "fc/core.h"


static bool is_open[UART_MAX_CH];
#define MAX_SIZE 128

static qbuffer_t ring_buffer[UART_MAX_CH];
static volatile uint8_t rx_buf[UART_MAX_CH-1][MAX_SIZE];
static volatile uint8_t rx_ringbuf[UART_MAX_CH-1][MAX_SIZE];

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart6_rx;

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_USB:
      is_open[ch] = true;
      ret = true;
      break;

    case _DEF_UART1:
      huart1.Instance = USART1;
      huart1.Init.BaudRate = baud;
      huart1.Init.WordLength = UART_WORDLENGTH_8B;
      huart1.Init.StopBits = UART_STOPBITS_1;
      huart1.Init.Parity = UART_PARITY_NONE;
      huart1.Init.Mode = UART_MODE_TX_RX;
      huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart1.Init.OverSampling = UART_OVERSAMPLING_16;

      qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART1][0], MAX_SIZE);

      if (HAL_UART_Init(&huart1) != HAL_OK)
      {
        Error_Handler();
      }
      else
      {
        ret = true;
        is_open[ch] = true;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buf[_DEF_UART1][0], 1);
      }
      break;

    case _DEF_UART2:
    	huart2.Instance = USART2;
    	huart2.Init.BaudRate = baud;
    	huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
    	huart2.Init.Parity = UART_PARITY_NONE;
    	huart2.Init.Mode = UART_MODE_TX_RX;
    	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART2][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buf[_DEF_UART2][0], 1);
    	}
      break;

    case _DEF_UART3:
    	huart3.Instance = USART3;
    	huart3.Init.BaudRate = baud;
    	huart3.Init.WordLength = UART_WORDLENGTH_8B;
      huart3.Init.StopBits = UART_STOPBITS_1;
    	huart3.Init.Parity = UART_PARITY_NONE;
    	huart3.Init.Mode = UART_MODE_TX_RX;
    	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART3][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buf[_DEF_UART3][0], 1) != HAL_OK)
        {
          ret = false;
        }
    	}
      break;

    case _DEF_UART4:
    	huart4.Instance = UART4;
    	huart4.Init.BaudRate = baud;
    	huart4.Init.WordLength = UART_WORDLENGTH_8B;
      huart4.Init.StopBits = UART_STOPBITS_1;
    	huart4.Init.Parity = UART_PARITY_NONE;
    	huart4.Init.Mode = UART_MODE_TX_RX;
    	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart4.Init.OverSampling = UART_OVERSAMPLING_16;

      qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART4][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart4) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_buf[_DEF_UART4][0], 1) != HAL_OK)
        {
          ret = false;
        }

    	}
      break;

    case _DEF_UART5:
    	huart5.Instance = UART5;
    	huart5.Init.BaudRate = baud;
    	huart5.Init.WordLength = UART_WORDLENGTH_8B;
      huart5.Init.StopBits = UART_STOPBITS_1;
    	huart5.Init.Parity = UART_PARITY_NONE;
    	huart5.Init.Mode = UART_MODE_TX_RX;
    	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART5][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart5) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_buf[_DEF_UART5][0], 1) != HAL_OK)
        {
          ret = false;
        }
    	}
      break;

    case _DEF_UART6:
    	huart6.Instance = USART6;
    	huart6.Init.BaudRate = baud;
    	huart6.Init.WordLength = UART_WORDLENGTH_8B;
      huart6.Init.StopBits = UART_STOPBITS_1;
    	huart6.Init.Parity = UART_PARITY_NONE;
    	huart6.Init.Mode = UART_MODE_TX_RX;
    	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_ringbuf[_DEF_UART6][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart6) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
    		is_open[ch] = true;
        if(HAL_UART_Receive_IT(&huart6, (uint8_t *)&rx_buf[_DEF_UART6][0], 1) != HAL_OK)
    		{
    			ret = false;
    		}
    	}
      break;
  }

  return ret;
}

bool uartIsConnected(uint8_t ch)
{
  bool ret = false;

  return is_open[ch];
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcAvailable();
      break;

    case _DEF_UART1:
      //ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart2_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART2:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart2_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
    case _DEF_UART3:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart3_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART4:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_uart4_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART5:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_uart5_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART6:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart6_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
  }
  return ret;
}

bool uartTxBufEmpty(uint8_t ch)
{
  bool ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      //ret = cdcAvailable();
      break;
    case _DEF_UART1:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;
    case _DEF_UART2:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;
    case _DEF_UART3:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;

    case _DEF_UART4:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;

    case _DEF_UART5:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;

    case _DEF_UART6:
      ret = qbufferTxEmpty(&ring_buffer[ch]);
      break;
  }
  return ret;
}

uint32_t uartTotalTxBytesFree(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      //ret = cdcAvailable();
      break;
    case _DEF_UART1:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;
    case _DEF_UART2:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;
    case _DEF_UART3:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;

    case _DEF_UART4:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;

    case _DEF_UART5:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;

    case _DEF_UART6:
      ret = qbufferTxBytesFree(&ring_buffer[ch]);
      break;
  }
  return ret;
}

void waitForSerialPortToFinishTransmitting(uint8_t ch)
{
    while (!uartTxBufEmpty(ch)) {
        delay(10);
    };
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_USB:
      ret = cdcRead();
      break;
    case _DEF_UART1:
      qbufferRead(&ring_buffer[ch], &ret, 1);
      break;
    case _DEF_UART2:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART3:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART4:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART5:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART6:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcWrite(p_data, length);
      break; 
    case _DEF_UART1:
      status = HAL_UART_Transmit(&huart1, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
    case _DEF_UART2:
      status = HAL_UART_Transmit(&huart2, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit(&huart3, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART4:
      status = HAL_UART_Transmit(&huart4, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART5:
      status = HAL_UART_Transmit(&huart5, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART6:
      status = HAL_UART_Transmit(&huart6, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_USB:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART1:
      status = HAL_UART_Transmit_IT(&huart1, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART2:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit_IT(&huart3, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART4:
      status = HAL_UART_Transmit_IT(&huart4, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART5:
      status = HAL_UART_Transmit_IT(&huart5, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART6:
      status = HAL_UART_Transmit_IT(&huart6, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteDMA(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_USB:
      status = HAL_UART_Transmit_DMA(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART1:
      status = HAL_UART_Transmit_DMA(&huart1, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART2:
      status = HAL_UART_Transmit_DMA(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit_DMA(&huart3, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART4:
      status = HAL_UART_Transmit_DMA(&huart4, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART5:
      status = HAL_UART_Transmit_DMA(&huart5, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART6:
      status = HAL_UART_Transmit_DMA(&huart6, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

void serialPrint(uint8_t channel, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
      uartWrite(channel, &ch, 1);
    }
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWriteIT(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_USB:
      ret = cdcGetBaud();
      break;

    case _DEF_UART1:
      ret = huart1.Init.BaudRate;
      break;

    case _DEF_UART2:
      ret = huart2.Init.BaudRate;
      break;

    case _DEF_UART3:
      ret = huart3.Init.BaudRate;
      break;

    case _DEF_UART4:
      ret = huart4.Init.BaudRate;
      break;

    case _DEF_UART5:
      ret = huart5.Init.BaudRate;
      break;

    case _DEF_UART6:
      ret = huart6.Init.BaudRate;
      break;
  }

  return ret;
}

bool uartSetBaud(uint8_t ch, uint32_t baud)
{
	bool ret = false;

	switch(ch)
	{
    case _DEF_USB:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART1:
      huart1.Init.BaudRate = baud;
      if (HAL_UART_Init(&huart1) != HAL_OK)
      {
        Error_Handler();
      }else
      {
        ret = true;
      }
      break;

		case _DEF_UART2:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART3:
			huart3.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART4:
			huart4.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart4) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART5:
			huart5.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart5) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART6:
			huart6.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart6) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;
	}

	return ret;
}

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    uint8_t index;

    for (index = 0; index < BAUD_RATE_COUNT; index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}

uint32_t overren_cnt = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
  	if(huart->ErrorCode == 8)
  	{
  		overren_cnt++;
  	    /* UART Over-Run interrupt occurred -----------------------------------------*/
  	    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)) {
  	    	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
  	    	__HAL_UART_CLEAR_OREFLAG(huart);
  	    }
        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
  	}
  }
}

typedef enum {
    GCS_IDLE,
    GCS_HEADER_SYNC,
    GCS_HEADER_ID,

    GCS_PAYLOAD,

    GCS_CHECKSUM,

    GCS_DATA_RECEIVED
} gcsState_e;

typedef enum {
    GCS_Tlemetry,
    GCS_PID_recive,
    GCS_PID_send,
    GCS_PID_save,
    GCS_ACC_calibration,
    GCS_MAG_calibration,
    GCS_PID_Test
} gcsData_e;

uint8_t uart1_rx_data = 0;
uint8_t telemetry_rx_buf[80];
uint8_t telemetry_rx_cplt_flag;


#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

#define UBX_MAX_PAYLOAD 256

typedef enum {
    UBX_SYNC1,
    UBX_SYNC2,
    UBX_CLASS,
    UBX_ID,
    UBX_LENGTH1,
    UBX_LENGTH2,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B
} UbxParserState_t;

typedef struct {
    UbxParserState_t state;
    uint8_t class;
    uint8_t id;
    uint16_t length;
    uint16_t payload_counter;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;
    uint32_t Ubx_Error;
} UbxParser_t;

UbxParser_t ubxParser = {
    .state = UBX_SYNC1
};

static void GPS_Passer(uint8_t c)
{
  switch (ubxParser.state) {
      case UBX_SYNC1:
          if (c == UBX_SYNC_CHAR1) ubxParser.state = UBX_SYNC2;
          break;
      case UBX_SYNC2:
          if (c == UBX_SYNC_CHAR2) ubxParser.state = UBX_CLASS;
          else ubxParser.state = UBX_SYNC1;
          break;
      case UBX_CLASS:
          ubxParser.class = c;
          ubxParser.ck_a = c;
          ubxParser.ck_b = ubxParser.ck_a;
          ubxParser.state = UBX_ID;
          break;
      case UBX_ID:
          ubxParser.id = c;
          ubxParser.ck_a += c;
          ubxParser.ck_b += ubxParser.ck_a;
          ubxParser.state = UBX_LENGTH1;
          break;
      case UBX_LENGTH1:
          ubxParser.length = c;
          ubxParser.ck_a += c;
          ubxParser.ck_b += ubxParser.ck_a;
          ubxParser.state = UBX_LENGTH2;
          break;
      case UBX_LENGTH2:
          ubxParser.length |= ((uint16_t)c << 8);
          ubxParser.ck_a += c;
          ubxParser.ck_b += ubxParser.ck_a;
          if (ubxParser.length > UBX_MAX_PAYLOAD) {
              // 길이 에러 시 초기화
              ubxParser.state = UBX_SYNC1;
          } else if (ubxParser.length == 0) {
              ubxParser.state = UBX_CK_A;
          } else {
              ubxParser.payload_counter = 0;
              ubxParser.state = UBX_PAYLOAD;
          }
          break;
      case UBX_PAYLOAD:
          ubxParser.payload[ubxParser.payload_counter++] = c;
          ubxParser.ck_a += c;
          ubxParser.ck_b += ubxParser.ck_a;
          if (ubxParser.payload_counter >= ubxParser.length) {
              ubxParser.state = UBX_CK_A;
          }
          break;
      case UBX_CK_A:
          if (c == ubxParser.ck_a) {
              ubxParser.state = UBX_CK_B;
          } else {
            ubxParser.Ubx_Error++;
            ubxParser.state = UBX_SYNC1; // 체크섬 에러
          }
          break;
      case UBX_CK_B:
          if (c == ubxParser.ck_b) {
              // 메시지 완성
              Ubx_HandleMessage(ubxParser.class, ubxParser.id, ubxParser.payload, ubxParser.length);
          } else {
            ubxParser.Ubx_Error++;
          }
          ubxParser.state = UBX_SYNC1;
          break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    msp_tx_end_time = micros()-msp_tx_start_time;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint32_t pre_time = 0;

  if(huart->Instance == USART1)
  {
    qbufferWrite(&ring_buffer[_DEF_UART1], (uint8_t *)&rx_buf[_DEF_UART1][0], 1);
    //GCS_Passer(rx_buf[_DEF_UART1][0]);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buf[_DEF_UART1][0], 1);
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&rx_buf[_DEF_UART1][0], MAX_SIZE);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }

  if(huart->Instance == USART2)
  {
      rxRuntimeState.callbackTime = micros() - pre_time;
      pre_time = micros();
      rxRuntimeState.RxCallback_Flag = true;
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buf[_DEF_UART2][0], 1);
      qbufferWrite(&ring_buffer[_DEF_UART2], (uint8_t *)&rx_buf[_DEF_UART2][0], 1);
//      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf[_DEF_UART2][0], MAX_SIZE);
//      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
      rxRuntimeState.RxCallback_Flag = false;
  }

  if(huart->Instance == USART3)
  {
    qbufferWrite(&ring_buffer[_DEF_UART3], (uint8_t *)&rx_buf[_DEF_UART3][0], 1);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buf[_DEF_UART3][0], 1);
  }

  if(huart->Instance == UART4)
  {
    qbufferWrite(&ring_buffer[_DEF_UART4], (uint8_t *)&rx_buf[_DEF_UART4][0], 1);
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_buf[_DEF_UART4][0], 1);
  }

  if(huart->Instance == UART5)
  {
    qbufferWrite(&ring_buffer[_DEF_UART5], (uint8_t *)&rx_buf[_DEF_UART5][0], 1);
    HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_buf[_DEF_UART5][0], 1);
  }

	if(huart->Instance == USART6)
	{
	  GPS_Passer(rx_buf[_DEF_UART6][0]);
    //qbufferWrite(&ring_buffer[_DEF_UART6], (uint8_t *)&rx_buf[_DEF_UART6][0], 1);
		HAL_UART_Receive_IT(&huart6, (uint8_t *)&rx_buf[_DEF_UART6][0], 1);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//  if(huart->Instance == USART1)
//  {
//    HAL_UART_DMAStop(&huart1);
//
//    qbufferWrite(&ring_buffer[_DEF_UART1], (uint8_t *)&rx_buf[_DEF_UART1][0], Size);
//
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&rx_buf[_DEF_UART1][0], MAX_SIZE);
//    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//  }

//  if(huart->Instance == USART2)
//  {
//      HAL_UART_DMAStop(&huart2);
//      rxRuntimeState.callbackTime = micros() - pre_time;
//      pre_time = micros();
//      rxRuntimeState.RxCallback_Flag = true;
//
//      qbufferWrite(&ring_buffer[_DEF_UART2], (uint8_t *)&rx_buf[_DEF_UART2][0], Size);
//
//      // DMA 재시작
//      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf[_DEF_UART2][0], MAX_SIZE);
//      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//      rxRuntimeState.RxCallback_Flag = false;
//  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }else if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}
