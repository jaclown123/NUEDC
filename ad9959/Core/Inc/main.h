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
#define SCLK_Pin GPIO_PIN_0
#define SCLK_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_1
#define CS_GPIO_Port GPIOC
#define SDIO0_Pin GPIO_PIN_2
#define SDIO0_GPIO_Port GPIOC
#define UPDATE_Pin GPIO_PIN_3
#define UPDATE_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_1
#define RESET_GPIO_Port GPIOA
#define PWR_Pin GPIO_PIN_4
#define PWR_GPIO_Port GPIOC
#define PS0_Pin GPIO_PIN_5
#define PS0_GPIO_Port GPIOC
#define PS1_Pin GPIO_PIN_6
#define PS1_GPIO_Port GPIOC
#define PS2_Pin GPIO_PIN_7
#define PS2_GPIO_Port GPIOC
#define PS3_Pin GPIO_PIN_8
#define PS3_GPIO_Port GPIOC
#define SDIO1_Pin GPIO_PIN_9
#define SDIO1_GPIO_Port GPIOC
#define SDIO2_Pin GPIO_PIN_8
#define SDIO2_GPIO_Port GPIOA
#define SDIO3_Pin GPIO_PIN_9
#define SDIO3_GPIO_Port GPIOA
#define SDIO4_Pin GPIO_PIN_10
#define SDIO4_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
