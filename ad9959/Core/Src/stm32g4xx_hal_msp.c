
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dac3_ch1;

extern DMA_HandleTypeDef hdma_dac4_ch1;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief CORDIC MSP Initialization
* This function configures the hardware resources used in this example
* @param hcordic: CORDIC handle pointer
* @retval None
*/
void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */

  }

}

/**
* @brief CORDIC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcordic: CORDIC handle pointer
* @retval None
*/
void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }

}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspInit 0 */

  /* USER CODE END DAC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC3_CLK_ENABLE();

    /* DAC3 DMA Init */
    /* DAC3_CH1 Init */
    hdma_dac3_ch1.Instance = DMA1_Channel1;
    hdma_dac3_ch1.Init.Request = DMA_REQUEST_DAC3_CHANNEL1;
    hdma_dac3_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac3_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac3_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac3_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac3_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac3_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac3_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_dac3_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hdac,DMA_Handle1,hdma_dac3_ch1);

  /* USER CODE BEGIN DAC3_MspInit 1 */

  /* USER CODE END DAC3_MspInit 1 */
  }
  else if(hdac->Instance==DAC4)
  {
  /* USER CODE BEGIN DAC4_MspInit 0 */

  /* USER CODE END DAC4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC4_CLK_ENABLE();

    /* DAC4 DMA Init */
    /* DAC4_CH1 Init */
    hdma_dac4_ch1.Instance = DMA1_Channel2;
    hdma_dac4_ch1.Init.Request = DMA_REQUEST_DAC4_CHANNEL1;
    hdma_dac4_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac4_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac4_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac4_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac4_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac4_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac4_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_dac4_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hdac,DMA_Handle1,hdma_dac4_ch1);

  /* USER CODE BEGIN DAC4_MspInit 1 */

  /* USER CODE END DAC4_MspInit 1 */
  }

}

/**
* @brief DAC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC3)
  {
  /* USER CODE BEGIN DAC3_MspDeInit 0 */

  /* USER CODE END DAC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC3_CLK_DISABLE();

    /* DAC3 DMA DeInit */
    HAL_DMA_DeInit(hdac->DMA_Handle1);
  /* USER CODE BEGIN DAC3_MspDeInit 1 */

  /* USER CODE END DAC3_MspDeInit 1 */
  }
  else if(hdac->Instance==DAC4)
  {
  /* USER CODE BEGIN DAC4_MspDeInit 0 */

  /* USER CODE END DAC4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC4_CLK_DISABLE();

    /* DAC4 DMA DeInit */
    HAL_DMA_DeInit(hdac->DMA_Handle1);
  /* USER CODE BEGIN DAC4_MspDeInit 1 */

  /* USER CODE END DAC4_MspDeInit 1 */
  }

}

/**
* @brief OPAMP MSP Initialization
* This function configures the hardware resources used in this example
* @param hopamp: OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hopamp->Instance==OPAMP4)
  {
  /* USER CODE BEGIN OPAMP4_MspInit 0 */

  /* USER CODE END OPAMP4_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OPAMP4 GPIO Configuration
    PB12     ------> OPAMP4_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP4_MspInit 1 */

  /* USER CODE END OPAMP4_MspInit 1 */
  }
  else if(hopamp->Instance==OPAMP6)
  {
  /* USER CODE BEGIN OPAMP6_MspInit 0 */

  /* USER CODE END OPAMP6_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OPAMP6 GPIO Configuration
    PB11     ------> OPAMP6_VOUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP6_MspInit 1 */

  /* USER CODE END OPAMP6_MspInit 1 */
  }

}

/**
* @brief OPAMP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hopamp: OPAMP handle pointer
* @retval None
*/
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp)
{
  if(hopamp->Instance==OPAMP4)
  {
  /* USER CODE BEGIN OPAMP4_MspDeInit 0 */

  /* USER CODE END OPAMP4_MspDeInit 0 */

    /**OPAMP4 GPIO Configuration
    PB12     ------> OPAMP4_VOUT
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

  /* USER CODE BEGIN OPAMP4_MspDeInit 1 */

  /* USER CODE END OPAMP4_MspDeInit 1 */
  }
  else if(hopamp->Instance==OPAMP6)
  {
  /* USER CODE BEGIN OPAMP6_MspDeInit 0 */

  /* USER CODE END OPAMP6_MspDeInit 0 */

    /**OPAMP6 GPIO Configuration
    PB11     ------> OPAMP6_VOUT
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN OPAMP6_MspDeInit 1 */

  /* USER CODE END OPAMP6_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */

  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspDeInit 0 */

  /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();
  /* USER CODE BEGIN TIM15_MspDeInit 1 */

  /* USER CODE END TIM15_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
