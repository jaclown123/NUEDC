/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern void LED_Display_Color(LED_Color * Color_Input);
int counter = 0;//转完�??圈的时间
int now_counter = 0;//现在的时�??
double theta;
double rad;
#define theta_1right 3
#define theta_2right 6
#define theta_3right 9
#define theta_4right 12
#define theta_5right 15
#define theta_6right 18
#define theta_7right 21
#define theta_8right 24
#define theta_9right 27
#define theta_10right 30
#define theta_1left 0
#define theta_2left 3
#define theta_3left 6
#define theta_4left 9
#define theta_5left 12
#define theta_6left 15
#define theta_7left 18
#define theta_8left 21
#define theta_9left 24
#define theta_10left 27
extern UART_HandleTypeDef huart1;
extern LED_Color color[16] = {RED,RED};
LED_Color digit_0[10][16] = {
      {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0}
  };

  // 数字1的图�?
  LED_Color digit_1[10][16] = {
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 1},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
  };

  // 数字2的图�?
  LED_Color digit_2[10][16] = {
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 1, 0 ,0, 0, 0, 0, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 1 ,0, 0, 0, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,1, 0, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 1, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 1, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 1, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
  };

  // 数字3的图�?
  LED_Color digit_3[10][16] = {
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 1, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 1, 0, 0, 0 ,0, 1, 1, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 1, 0, 0, 0 ,0, 1, 1, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,1, 0, 1, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,0, 0, 0, 1, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
  };

  // 数字4的图�?
  LED_Color digit_4[10][16] = {
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 1},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 0},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 0},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
    };

  // 数字5的图�?
  LED_Color digit_5[10][16] = {
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 1, 1 ,1, 1, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 0},
  };

  // 数字6的图�?
  LED_Color digit_6[10][16] = {
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1 ,1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 0},
  };

  // 数字7的图�?
  LED_Color digit_7[10][16] = {
      {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 1, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 1 ,1, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 1, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 1, 1, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 1, 1, 0}
    };

  // 数字8的图�?
  LED_Color digit_8[10][16] = {
      {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0}
    };

  // 数字9的图�?
  LED_Color digit_9[10][16] = {
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 1, 1, 1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0}
  };
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
void caculate_rad(int counter)
{
	rad = 360 / (counter);
}
void caculate_theta(void)
{
	theta = now_counter * rad;
}
void show_num(LED_Color ** num)
{
	if (theta < theta_1right && theta > theta_1left)
	  {
		  LED_Display_Color(num[0]);
	  }
	  if (theta < theta_2right && theta > theta_2left)
	    {
	  	  LED_Display_Color(num[1]);
	    }
	  if (theta < theta_3right && theta > theta_3left)
	    {
	  	  LED_Display_Color(num[2]);
	    }
	  if (theta < theta_4right && theta > theta_4left)
	    {
	  	  LED_Display_Color(num[3]);
	    }
	  if (theta < theta_5right && theta > theta_5left)
	    {
	  	  LED_Display_Color(num[4]);
	    }
	  if (theta < theta_6right && theta > theta_6left)
	    {
	  	  LED_Display_Color(num[5]);
	    }
	  if (theta < theta_7right && theta > theta_7left)
	    {
	  	  LED_Display_Color(num[6]);
	    }
	  if (theta < theta_8right && theta > theta_8left)
	    {
	  	  LED_Display_Color(num[7]);
	    }
	  if (theta < theta_9right && theta > theta_9left)
	    {
	  	  LED_Display_Color(num[8]);
	    }
	  if (theta < theta_10right && theta > theta_10left)
	    {
	  	  LED_Display_Color(num[9]);
	    }
}
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != detect_Pin)
	{
		return;
	}
	if (now_counter > 1)
	{

		counter = now_counter;
		caculate_rad(counter);
		now_counter = 0;
	}
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
//  now_counter++;
//
//  caculate_theta();
//  show_num(digit_0);
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(detect_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
  LED_Display_Color(color);
  /* USER CODE END EXTI0_1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
