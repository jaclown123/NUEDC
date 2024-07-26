/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UI_SW3_Pin GPIO_PIN_13
#define UI_SW3_GPIO_Port GPIOC
#define con_Pin GPIO_PIN_0
#define con_GPIO_Port GPIOC
#define AD9833_EN_Pin GPIO_PIN_9
#define AD9833_EN_GPIO_Port GPIOA
#define AD9834_EN_Pin GPIO_PIN_10
#define AD9834_EN_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_11
#define LCD_DC_GPIO_Port GPIOC
#define UI_SW0_Pin GPIO_PIN_2
#define UI_SW0_GPIO_Port GPIOD
#define SIPO_CS_Pin GPIO_PIN_8
#define SIPO_CS_GPIO_Port GPIOB
#define XDAC_CS_Pin GPIO_PIN_9
#define XDAC_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
