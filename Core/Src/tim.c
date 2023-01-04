/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "HCSR04.h"
#include "User_Debug.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (tim_icHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *tim_icHandle)
{

  if (tim_icHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
uint8_t TIM2CH2_CAPTURE_STA;  // 输入捕获状态
uint16_t TIM2CH2_CAPTURE_VAL; // 输入捕获值
// 溢出回调函数和捕获回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
  {
    if (TIM2CH2_CAPTURE_STA & 0x40) // 已捕获到一个上升沿
    {
      if ((TIM2CH2_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
      {
        TIM2CH2_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
        TIM2CH2_CAPTURE_VAL = 0XFFFF;
      }
      else
        TIM2CH2_CAPTURE_STA++; // 否则标记溢出数加1
    }
  }
}

// 捕获中断发生时执行 上升沿复位开始计时，下降沿获取捕获值计算
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0X80) == 0) // 还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM2CH2_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM2CH2_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM2CH2_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM2CH2_CAPTURE_STA = 0;
      TIM2CH2_CAPTURE_VAL = 0;
      TIM2CH2_CAPTURE_STA |= 0X40;                                            // 将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim2);                                              // 关闭定时器
      __HAL_TIM_SET_COUNTER(&htim2, 0);                                       // 将定时器计数值清零
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);                       // 清除输入捕获标志位
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING); // 将输入捕获上升沿改为捕获下降沿
      __HAL_TIM_ENABLE(&htim2);                                               // 使能定时器，开启定时器
    }
  }
}
/* USER CODE END 1 */
