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
#define SW3_Pin GPIO_PIN_13
#define SW3_GPIO_Port GPIOC
#define DIG2_Pin GPIO_PIN_14
#define DIG2_GPIO_Port GPIOC
#define DIG3_Pin GPIO_PIN_15
#define DIG3_GPIO_Port GPIOC
#define SEG6_Pin GPIO_PIN_5
#define SEG6_GPIO_Port GPIOA
#define SEG5_Pin GPIO_PIN_6
#define SEG5_GPIO_Port GPIOA
#define SEG4_Pin GPIO_PIN_7
#define SEG4_GPIO_Port GPIOA
#define KR3_Pin GPIO_PIN_0
#define KR3_GPIO_Port GPIOB
#define KR2_Pin GPIO_PIN_1
#define KR2_GPIO_Port GPIOB
#define KR1_Pin GPIO_PIN_10
#define KR1_GPIO_Port GPIOB
#define KR0_Pin GPIO_PIN_11
#define KR0_GPIO_Port GPIOB
#define SEG7_Pin GPIO_PIN_9
#define SEG7_GPIO_Port GPIOA
#define KC0_Pin GPIO_PIN_2
#define KC0_GPIO_Port GPIOD
#define KC1_Pin GPIO_PIN_5
#define KC1_GPIO_Port GPIOB
#define KC2_Pin GPIO_PIN_6
#define KC2_GPIO_Port GPIOB
#define DIG0_Pin GPIO_PIN_8
#define DIG0_GPIO_Port GPIOB
#define DIG1_Pin GPIO_PIN_9
#define DIG1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
