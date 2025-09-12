/*
 * timer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "timer.h"

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

bool timerInit()
{
	bool ret = true;

//  TIM_HandleTypeDef* handle = motors->channel.tim;
//  if (handle == NULL) return false;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 41999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


	////////////////////////////////////////////////////////////////
	TIM_ClockConfigTypeDef sClockSourceConfig1 = {0};
	TIM_MasterConfigTypeDef sMasterConfig1 = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 83;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
	Error_Handler();
	}
	sClockSourceConfig1.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_TIM_Base_Init(&htim5);
	HAL_TIM_Base_Start(&htim5);

	///////////////////////////////////////////////////////////////////////////
	 TIM_ClockConfigTypeDef sClockSourceConfig2 = {0};
  TIM_MasterConfigTypeDef sMasterConfig2 = {0};
  TIM_OC_InitTypeDef sConfigOC2 = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig2 = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 3;//(0)
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 52; // 800kHz, (104) 1.25us period
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig2.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
//  sMasterConfig2.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig2) != HAL_OK)
//  {
//    Error_Handler();
//  }
  sConfigOC2.OCMode = TIM_OCMODE_PWM1;
  sConfigOC2.Pulse = 0;
  sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
  //sConfigOC2.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC2.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC2.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC2, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
//  sBreakDeadTimeConfig2.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig2.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig2.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig2.DeadTime = 0;
//  sBreakDeadTimeConfig2.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig2.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig2.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig2) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) {
      /* Starting PWM generation Error */
    Error_Handler();
  }

	return ret;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

	if(tim_baseHandle->Instance==TIM4)
	{
		/* USER CODE BEGIN TIM4_MspInit 0 */

		/* USER CODE END TIM4_MspInit 0 */
		/* TIM4 clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();

		/* TIM4 interrupt Init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspInit 1 */

		/* USER CODE END TIM4_MspInit 1 */
	}
	else if(tim_baseHandle->Instance==TIM5)
	{
		/* USER CODE BEGIN TIM5_MspInit 0 */

		/* USER CODE END TIM5_MspInit 0 */
		/* TIM5 clock enable */
		__HAL_RCC_TIM5_CLK_ENABLE();

		/* TIM5 interrupt Init */
		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);
		/* USER CODE BEGIN TIM5_MspInit 1 */

		/* USER CODE END TIM5_MspInit 1 */
	}
	else if(tim_baseHandle->Instance==TIM8)
	  {
	  /* USER CODE BEGIN TIM8_MspInit 0 */

	  /* USER CODE END TIM8_MspInit 0 */
      /* TIM8 clock enable */
      __HAL_RCC_TIM8_CLK_ENABLE();

      /* TIM8 DMA Init */
      /* TIM8_CH4_TRIG_COM Init */
      hdma_tim8_ch4_trig_com.Instance = DMA2_Stream7;
      hdma_tim8_ch4_trig_com.Init.Channel = DMA_CHANNEL_7;
      hdma_tim8_ch4_trig_com.Init.Direction = DMA_MEMORY_TO_PERIPH;
      hdma_tim8_ch4_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_tim8_ch4_trig_com.Init.MemInc = DMA_MINC_ENABLE;
      hdma_tim8_ch4_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
      hdma_tim8_ch4_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
      hdma_tim8_ch4_trig_com.Init.Mode = DMA_NORMAL;
      hdma_tim8_ch4_trig_com.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_tim8_ch4_trig_com.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_tim8_ch4_trig_com) != HAL_OK)
      {
        Error_Handler();
      }

      /* Several peripheral DMA handle pointers point to the same DMA handle.
       Be aware that there is only one stream to perform all the requested DMAs. */
      __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim8_ch4_trig_com);
      //__HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_TRIGGER],hdma_tim8_ch4_trig_com);
      //__HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim8_ch4_trig_com);

	    /* TIM8 interrupt Init */
	    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
	  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM8) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    PB9     ------> TIM4_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM8)
    {
    /* USER CODE BEGIN TIM8_MspPostInit 0 */

    /* USER CODE END TIM8_MspPostInit 0 */

      __HAL_RCC_GPIOC_CLK_ENABLE();
      /**TIM8 GPIO Configuration
      PC9     ------> TIM8_CH4
      */
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM8_MspPostInit 1 */

    /* USER CODE END TIM8_MspPostInit 1 */
    }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

	if(tim_baseHandle->Instance==TIM4)
	{
		/* USER CODE BEGIN TIM4_MspDeInit 0 */

		/* USER CODE END TIM4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();

		/* TIM4 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspDeInit 1 */

		/* USER CODE END TIM4_MspDeInit 1 */
	}
	else if(tim_baseHandle->Instance==TIM5)
	{
		/* USER CODE BEGIN TIM5_MspDeInit 0 */

		/* USER CODE END TIM5_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM5_CLK_DISABLE();

		/* TIM5 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM5_IRQn);
		/* USER CODE BEGIN TIM5_MspDeInit 1 */

		/* USER CODE END TIM5_MspDeInit 1 */
	}
  else if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /* TIM8 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_TRIGGER]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_COMMUTATION]);

    /* TIM8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
}
