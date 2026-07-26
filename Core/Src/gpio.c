/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_B_Pin|LED2_G_Pin|LED2_R_Pin|STATUS_LED1_Pin
                          |STATUS_LED2_Pin|LED1_R_Pin|LED1_G_Pin|LED1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_B_Pin LED2_G_Pin LED2_R_Pin STATUS_LED1_Pin
                           STATUS_LED2_Pin LED1_R_Pin LED1_G_Pin LED1_B_Pin */
  GPIO_InitStruct.Pin = LED2_B_Pin|LED2_G_Pin|LED2_R_Pin|STATUS_LED1_Pin
                          |STATUS_LED2_Pin|LED1_R_Pin|LED1_G_Pin|LED1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : nRST_Pin */
  GPIO_InitStruct.Pin = nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FDCAN1_FAULT_Pin FDCAN1_S_Pin */
  GPIO_InitStruct.Pin = FDCAN1_FAULT_Pin|FDCAN1_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_DIR_Pin */
  GPIO_InitStruct.Pin = RIGHT_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RIGHT_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_DIR_Pin */
  GPIO_InitStruct.Pin = LEFT_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEFT_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_SW_Pin TOP_PB_Pin MID_PB_Pin BOT_PB_Pin
                           IMU_INTERRUPT_Pin FDCAN2_FAULT_Pin FDCAN2_S_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW_Pin|TOP_PB_Pin|MID_PB_Pin|BOT_PB_Pin
                          |IMU_INTERRUPT_Pin|FDCAN2_FAULT_Pin|FDCAN2_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
