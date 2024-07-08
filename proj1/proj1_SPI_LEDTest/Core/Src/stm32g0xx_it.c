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
int theta;
int rad;
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
extern LED_Color close[16] = {0};
LED_Color digit[10][10][16] =
{
    {
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
    },
    {
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
    },
    {
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 1, 0 ,0, 0, 0, 0, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,0, 0, 0, 0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 1 ,1, 0, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,1, 1, 0, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 1, 1, 0, 0, 0, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 1, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 1, 1, 0, 0, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0}
    },
    {
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
    },
    {
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
    },
    {
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 1, 1 ,1, 1, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 1, 1 ,1, 1, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 0},
    },
    {
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1 ,1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 1, 1 ,1, 1, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 0, 0, 0, 0, 0},
    },
    {
      {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 1, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 1, 1, 0 ,0, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 1 ,1, 0, 0, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 1, 1, 0, 0, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 1, 1, 0, 0, 1},
  	  {0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 1, 1, 0}
    },
    {
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
    },
    {
	  {0, 0, 0, 0, 0, 0, 1, 0 ,0, 0, 1, 1, 1, 1, 1, 0},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 1, 0, 0, 0, 0, 1},
	  {0, 0, 0, 0, 0, 0, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 0}
    }
};
 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t theta_threshold[10][10] = 
{
	{
		0, 3, 6, 9, 12, 15, 18, 21, 24, 27
	},
	{
		3, 6, 9, 12, 15, 18, 21, 24, 27, 30
	},
	{
		30, 33, 36, 39, 42, 45, 48, 51, 54, 57
	},
	{
		33, 36, 39, 42, 45, 48, 51, 54, 57, 60
	},
	{
		60, 63, 66, 69, 72, 75, 78, 81, 84, 87
	},
	{
		63, 66, 69, 72, 75, 78, 81, 84, 87, 90
	},
	{
		90, 93, 96, 99, 102, 105, 108, 111, 114, 117
	},
	{
		93, 96, 99, 102, 105, 108, 111, 114, 117, 120
	},
	{
		120, 123, 126, 129, 132, 135, 138, 141, 144, 147
	},
	{
		123, 126, 129, 132, 135, 138, 141, 144, 147, 150
	
	}
};
void caculate_rad(int counter)
{
	rad = 360 / (counter);
}
void caculate_theta(void)
{
	theta = now_counter * rad;
}
void show_num(int i, int num)//i为希望在第几位显示，num为要显示的数 i = 0, 1, 2, 3, 4
{
	if (theta <= theta_threshold[2 * i + 1][0] && theta > theta_threshold[2 * i][0])
	  {
		  LED_Display_Color(digit[num][0]);
	  }
	  else if (theta <= theta_threshold[2 * i + 1][1] && theta > theta_threshold[2 * i][1])
	    {
	  	  LED_Display_Color(digit[num][1]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][2] && theta > theta_threshold[2 * i][2])
	    {
	  	  LED_Display_Color(digit[num][2]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][3] && theta > theta_threshold[2 * i][3])
	    {
	  	  LED_Display_Color(digit[num][3]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][4] && theta > theta_threshold[2 * i][4])
	    {
	  	  LED_Display_Color(digit[num][4]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][5] && theta > theta_threshold[2 * i][5])
	    {
	  	  LED_Display_Color(digit[num][5]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][6] && theta > theta_threshold[2 * i][6])
	    {
	  	  LED_Display_Color(digit[num][6]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][7] && theta > theta_threshold[2 * i][1])
	    {
	  	  LED_Display_Color(digit[num][7]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][8] && theta > theta_threshold[2 * i][8])
	    {
	  	  LED_Display_Color(digit[num][8]);
	    }
	  else if (theta <= theta_threshold[2 * i + 1][9] && theta > theta_threshold[2 * i][9])
	    {
	  	  LED_Display_Color(digit[num][9]);
	    }
	  else
	  {
		LED_Display_Color(close);
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
 now_counter++;

 caculate_theta();
 show_num(0, 1);
 show_num(1, 2);
 show_num(4, 5);
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
  //LED_Display_Color(color);
  /* USER CODE END EXTI0_1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
