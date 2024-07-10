/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t flag[6] = {0};
int row[6] = {0};
int counter ;//转完1圈的时间
int atime ;
int now_counter;//现在的时间
int theta = 0;
int rad = 0;
LED_Color color[16] = {RED,RED};
LED_Color close[16] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
void LED_Init()
{
  HAL_GPIO_WritePin(LED_IN_RST_GPIO_Port,LED_IN_RST_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_OUT_EN_GPIO_Port,LED_OUT_EN_Pin,GPIO_PIN_RESET);
}

void Input_Fix(GPIO_PinState * Input_Data)
{
  for (int i=7;i<=47;i=i+8) {
    for (int j=0;j<=2;j++) {
      GPIO_PinState temp=Input_Data[j+i-7];
      Input_Data[j+i-7]=Input_Data[i-1-j];
      Input_Data[i-1-j]=temp;
    }
  }
}

void Generate_Bytes(uint8_t * Input_Bits,uint8_t* Bytes)
{
  for (int byte_index=0;byte_index<6;byte_index++) {
    for (int i = 0;i < 8;i++) {
      switch(Input_Bits[byte_index*8+i]) {
      case 0:{

        break;
      }
      case 1:{
        Bytes[byte_index]|=(1<<(7-i));
        break;
      }
      }
    }
  }
}

void LED_Display_6Byte_SPI(GPIO_PinState * Input_Data) {
  Input_Fix(Input_Data);
  uint8_t Input_Byte[6]={0};
  Generate_Bytes(Input_Data,Input_Byte);

  HAL_GPIO_WritePin(LED_OUT_CLK_GPIO_Port,LED_OUT_CLK_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1,Input_Byte, 6, 1);
  HAL_GPIO_WritePin(LED_OUT_CLK_GPIO_Port,LED_OUT_CLK_Pin,GPIO_PIN_SET);
}

//void LED_Display_6Byte_GPIO(GPIO_PinState * Input_Data) {
//  Input_Fix(Input_Data);
//  HAL_GPIO_WritePin(LED_OUT_CLK_GPIO_Port,LED_OUT_CLK_Pin,GPIO_PIN_RESET);
//  for (int i=0;i < 48;i++) {
//    HAL_GPIO_WritePin(LED_DATA_IN_GPIO_Port,LED_DATA_IN_Pin,Input_Data[i]);
//    HAL_GPIO_WritePin(LED_IN_CLK_GPIO_Port,LED_IN_CLK_Pin,GPIO_PIN_SET);
//    HAL_Delay(0);
//    HAL_GPIO_WritePin(LED_IN_CLK_GPIO_Port,LED_IN_CLK_Pin,GPIO_PIN_RESET);
//    HAL_Delay(0);
//  }
//  HAL_GPIO_WritePin(LED_OUT_CLK_GPIO_Port,LED_OUT_CLK_Pin,GPIO_PIN_SET);
//}

void LED_Display_Color(LED_Color * Color_Input) {
  uint8_t Color_Bytes[48]={1};
  int color_index=0,byte_index=47;
  while(color_index<16) {
    switch(Color_Input[color_index]){
    case BLACK:{
      Color_Bytes[byte_index]=GPIO_PIN_SET;
      Color_Bytes[byte_index-1]=GPIO_PIN_SET;
      Color_Bytes[byte_index-2]=GPIO_PIN_SET;
      break;
    }
    case RED:{
      Color_Bytes[byte_index]=GPIO_PIN_SET;
      Color_Bytes[byte_index-1]=GPIO_PIN_SET;
      Color_Bytes[byte_index-2]=GPIO_PIN_RESET;
      break;
    }
    case GREEN:{
      Color_Bytes[byte_index]=GPIO_PIN_SET;
      Color_Bytes[byte_index-1]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-2]=GPIO_PIN_SET;
      break;
    }
    case BLUE:{
      Color_Bytes[byte_index]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-1]=GPIO_PIN_SET;
      Color_Bytes[byte_index-2]=GPIO_PIN_SET;
      break;
    }
    case YELLOW:{
      Color_Bytes[byte_index]=GPIO_PIN_SET;
      Color_Bytes[byte_index-1]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-2]=GPIO_PIN_RESET;
      break;
    }
    case PINK:{
      Color_Bytes[byte_index]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-1]=GPIO_PIN_SET;
      Color_Bytes[byte_index-2]=GPIO_PIN_RESET;
      break;
    }
    case CYAN:{
      Color_Bytes[byte_index]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-1]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-2]=GPIO_PIN_SET;
      break;
    }
    case WHITE:{
      Color_Bytes[byte_index]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-1]=GPIO_PIN_RESET;
      Color_Bytes[byte_index-2]=GPIO_PIN_RESET;
      break;
    }
    }
    color_index++;
    byte_index-=3;
  }
  LED_Display_6Byte_SPI(Color_Bytes);
}
void caculate_rad(int counter)
{
	rad = 360 / (counter);
	//atime = counter;
}
void caculate_theta(void)
{
	theta = now_counter * rad;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LED_Init();
  HAL_TIM_Base_Start_IT(&htim1);

  LED_Color color[16]={PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK,PINK};
  LED_Color red[16] = {1,1,1,1,1,1};
  RTC_TimeTypeDef * time;
//LED_Display_Color(red);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static int prev = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN);

	int curr = (HAL_GPIO_ReadPin(detect_GPIO_Port, detect_Pin));
	if (prev == 1 && curr == 0)
	{
		if (now_counter > 1)
		{
			counter = now_counter;
			caculate_rad(counter);
			now_counter = 0;
			for(int i = 0;i < 6;i++)
			{
				flag[i] = 0;
				row[i] = 0;
			}
		}
	}
	prev = curr;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x17;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x8;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 498;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_OUT_EN_Pin|LED_OUT_CLK_Pin|LED_IN_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_OUT_EN_Pin LED_OUT_CLK_Pin LED_IN_RST_Pin */
  GPIO_InitStruct.Pin = LED_OUT_EN_Pin|LED_OUT_CLK_Pin|LED_IN_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : detect_Pin */
  GPIO_InitStruct.Pin = detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(detect_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
