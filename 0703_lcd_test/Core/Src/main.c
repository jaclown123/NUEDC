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
#include "tft18.h"
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  setup();
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  GPIO_PinState prev = GPIO_PIN_SET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int Direction =  __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8);   //读取电机转动方向
	  int CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
	  int counter = CaptureNumber / 4 % 7 ;
	  int mode = 0;
	  int mode_t = 0;
	  int ampl = 0;
	  int ampl_t = 0;
	  int modual = 0;
	  int modual_t = 0;
	  int delay = 0;
	  int delay_t = 0;
	  int atten = 0;
	  int atten_t = 0;
	  int freq = 0;
	  int freq_t = 0;
	  int phase = 0;
	  int phase_t = 0;

	  switch(counter)
	  {
	  case 0:
		  lcd_show_picture(101, 5 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 1:
		  lcd_show_picture(101, 25 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 2:
		  lcd_show_picture(101, 45 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 3:
		  lcd_show_picture(101, 65 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 4:
		  lcd_show_picture(101, 85 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 5:
		  lcd_show_picture(101, 105 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  lcd_show_picture(101, 125 , 12, 20, gImage_black);
		  break;
	  case 6:
		  lcd_show_picture(101, 125 , 12, 20, MenuCursor16x16);
		  lcd_show_picture(101, 25 , 12, 20, gImage_black);
		  lcd_show_picture(101, 45 , 12, 20, gImage_black);
		  lcd_show_picture(101, 65 , 12, 20, gImage_black);
		  lcd_show_picture(101, 85 , 12, 20, gImage_black);
		  lcd_show_picture(101, 105 , 12, 20, gImage_black);
		  lcd_show_picture(101, 5 , 12, 20, gImage_black);
		  break;
	  }
	  GPIO_PinState curr = HAL_GPIO_ReadPin(GPIOC, 1<<13);
	  if (prev && !curr)
	  {
		  switch(counter)
		  {
		  case 0:
			  lcd_show_str(100, 145,"MODE:\n");
			  while(1)
			  {
				  mode = abs((CaptureNumber) / 4 % 2 + 1);
				  if(mode != mode_t)
				  {
					  mode_t = mode;
					  if (mode < 2) lcd_show_str(101,165,"CW");
					  else lcd_show_str(101,165,"AM");
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  // api
			  break;
		  case 1:
			  lcd_show_str(100, 145,"AMPL:\n");
			  while (1)
			  {
				  ampl = abs((CaptureNumber) / 4 % 10 * 100 + 100);
				  if(ampl != ampl_t)
				  {
					  ampl_t = ampl;
					  lcd_show_num(ampl);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  case 2:
			  lcd_show_str(100, 145,"MODULATION:\n");
			  while (1)
			  {
				  modual = abs((((CaptureNumber) / 4 )% 7) * 10 + 30);
				  if(modual != modual_t)
				  {
					  modual_t = modual;
					  lcd_show_num(modual);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  case 3:
			  lcd_show_str(100, 145,"DELAY:\n");
			  while (1)
			  {
				  delay = abs((CaptureNumber) / 4 % 6 * 30 + 50);
				  if(delay != delay_t)
				  {
					  delay_t = delay;
					  lcd_show_num(delay);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  case 4:
			  lcd_show_str(100, 145,"ATTENUATION:\n");
			  while (1)
			  {
				  atten = abs((CaptureNumber) / 4 % 11 * 2);
				  if(atten != atten_t)
				  {
					  atten_t = atten;
					  lcd_show_num(atten);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  case 5:
			  lcd_show_str(100, 145,"FREQUENCY:\n");
			  while (1)
			  {
				  freq = abs((CaptureNumber) / 4 % 11 + 30);
				  if(freq != freq_t)
				  {
					  freq_t = freq;
					  lcd_show_num(freq);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  case 6:
			  lcd_show_str(100, 145,"INIT PHASE\n");
			  while (1)
			  {
				  phase = abs((CaptureNumber) / 4 % 7 * 30);
				  if(phase != phase_t)
				  {
					  phase_t = phase;
					  lcd_show_num(phase);
				  }
				  CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
				  if (HAL_GPIO_ReadPin(GPIOD, 1<<2) == GPIO_PIN_RESET)
				  {
					  lcd_show_str(101,185,"CONFIRM?\n");
					  break;
				  }
			  }
			  //need api
			  break;
		  }
		  while(HAL_GPIO_ReadPin(GPIOC, 1<<13) != 0)
			  ;
		  //lcd_show_picture(101, 105, 240, 100, gImage_black_big);
		  lcd_show_black(100, 145, 239, 100);
	  }
	  prev = curr;


    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO_PG10, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
