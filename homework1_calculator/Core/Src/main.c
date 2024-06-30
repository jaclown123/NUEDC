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
  * write by Jac1own.
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
#include <math.h>
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

/* USER CODE BEGIN PV */
#define del -1
#define SEG0_GPIO_Port KR0_GPIO_Port
#define SEG1_GPIO_Port KR1_GPIO_Port
#define SEG2_GPIO_Port KR2_GPIO_Port
#define SEG3_GPIO_Port KR3_GPIO_Port
#define SEG3_Pin KR3_Pin
#define SEG2_Pin KR2_Pin
#define SEG1_Pin KR1_Pin
#define SEG0_Pin KR0_Pin
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint8_t get_char(void);
void close(void);
void keep_char(uint8_t c);
void show_4char(void);
void calculate(int* suf);
GPIO_PinState jac_read_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void push(double c);
void pop(void);
int isempty(void);
double calresult(int * suf);
void getFirstFourDigits(double num);
void show_result(void);
int precedence(uint8_t op);
uint8_t get(void);
double getd(void);


GPIO_TypeDef * colsport[] = {KC0_GPIO_Port,KC1_GPIO_Port,KC2_GPIO_Port};//三列
uint16_t cols[] = {KC0_Pin,KC1_Pin,KC2_Pin};
GPIO_TypeDef * rowsport[] = {KR0_GPIO_Port, KR1_GPIO_Port, KR2_GPIO_Port, KR3_GPIO_Port, SEG4_GPIO_Port};//5�????
uint16_t rows[] = {KR0_Pin,KR1_Pin,KR2_Pin,KR3_Pin,SEG4_Pin};
GPIO_TypeDef * digport[] = {DIG0_GPIO_Port,DIG1_GPIO_Port,DIG2_GPIO_Port,DIG3_GPIO_Port};
uint16_t digpin[] = {DIG0_Pin,DIG1_Pin,DIG2_Pin,DIG3_Pin};
GPIO_TypeDef * segport[] = {SEG0_GPIO_Port, SEG1_GPIO_Port, SEG2_GPIO_Port, SEG3_GPIO_Port, SEG4_GPIO_Port, SEG5_GPIO_Port, SEG6_GPIO_Port, SEG7_GPIO_Port};
uint16_t segpin[] = {SEG0_Pin, SEG1_Pin, SEG2_Pin, SEG3_Pin, SEG4_Pin, SEG5_Pin, SEG6_Pin, SEG7_Pin};
uint8_t codes[] = {
		0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
		0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,
		0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,
		0x3e,0x1c,0x7e,0x64,0x6e,0x59,0x40  };
uint8_t keyboard[6][3] = {{'1', '2', '3'},{'4', '5', '6'},{'7', '8', '9'},{'X', '0', 'E'},{'A', 'M', 'D'},{'\b', ' ', ' '}};
uint8_t expression[100] = {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'};
uint8_t end = 3;
GPIO_PinState pre_state = 0;
uint8_t flag = 0;
uint8_t digits[4] = {0};
double stack[100] = {0};
int stack_head = -1;
int flag2 = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double getd(void){
	if (stack_head!=-1)
	{
		return stack[stack_head];

	}
	else
		return -1;
}
int isempty(void)
{
	if (stack_head == -1)
	{
		return 1;
	}
	else
		return 0;
}
void setxdigit(double num, int x)
{
	while(num < pow(10, x - 1))
	{
		num *= 10;
	}

	    // 将num转换为整数
	    int temp = (int)num;
		int count = 0;
		while (temp % 10 == 0 )
		{
			temp /= 10;
			count++;
		}
		int j = count;
		while ( j > 0)
		{
			digits[x - j] = 0; // 0填充
			j --;

		}
	    // 从最低位开始提取每一位数字
	    for (int i = x - 1 - count; i >= 0; --i) {
	        digits[j] = temp % 10;
	        temp /= 10;
			j++;
	    }
}
void getFirstFourDigits(double num) {
    // 如果数值为负，则取其绝对值

    if (num != fabs(num))
    {

    	num = fabs(num);
    	setxdigit(num, 3);
    	digits[3] = 36;
    }
    else
    {
    	setxdigit(num,4);
    }
}
uint8_t get_char(void)
{
	int i, j;
	for (i = 0;i < 5; ++i)
	{
        HAL_GPIO_WritePin(rowsport[i], rows[i], GPIO_PIN_SET);
        HAL_GPIO_WritePin(rowsport[(i+1)%5], rows[(i+1)%5], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(rowsport[(i+2)%5], rows[(i+2)%5], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(rowsport[(i+3)%5], rows[(i+3)%5], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(rowsport[(i+4)%5], rows[(i+4)%5], GPIO_PIN_RESET);
        for (j = 0; j < 3; ++j)
        {
        	if (HAL_GPIO_ReadPin(colsport[j], cols[j]) == GPIO_PIN_SET)
        		{HAL_GPIO_WritePin(rowsport[i], rows[i], GPIO_PIN_RESET);

        		if (flag == 1)
        			return '&';
        		flag = 1 ;
        		return keyboard[i][j];}
        }
	}
	if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)
	{
		if (flag == 1)
			return '&';
		flag = 1;
		return keyboard[5][0];
	}
	close();
	flag = 0;
	return '&';

}
void close(void)
{
	int i = 0;
	HAL_GPIO_WritePin(digport[i], digpin[i], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(digport[(i + 1) % 4], digpin[(i + 1) % 4], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(digport[(i + 2) % 4], digpin[(i + 2) % 4], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(digport[(i + 3) % 4], digpin[(i + 3) % 4], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rowsport[i], rows[i], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rowsport[(i+1)%5], rows[(i+1)%5], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rowsport[(i+2)%5], rows[(i+2)%5], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rowsport[(i+3)%5], rows[(i+3)%5], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(rowsport[(i+4)%5], rows[(i+4)%5], GPIO_PIN_RESET);

}
void keep_char(uint8_t c)
{

	if (c == '&')
		return;
	if (c != '\b' && end < 33 )
	 {
		 end += 1;
		 expression[end] = c;
	 }
	 else if (c == '\b' && end > 3)
	 {
		 expression[end] = '\b';
		 end --;
	 }
	if (c == 'E')
	{

		expression[end] = '\0';

	}
	//HAL_Delay(0);

}
GPIO_PinState jac_read_pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
     // 使用static以保留上�?状�??
    GPIO_PinState now_state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

    // �?测到边沿：之前是RESET，现在是SET
    if (pre_state == GPIO_PIN_RESET && now_state == GPIO_PIN_SET) {
        pre_state = now_state; // 更新状�??
        //HAL_Delay(4	);
        return GPIO_PIN_SET; // 报告变化
    }
    else {
        pre_state = now_state; // 总是更新状�?�以反映�?新�??
        return GPIO_PIN_RESET; // 默认返回RESET，表示无变化或等待下�?变化
    }
}
void show_4char()
{
	int i, j;

	for (i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(digport[i], digpin[i], GPIO_PIN_SET);
		HAL_GPIO_WritePin(digport[(i + 1) % 4], digpin[(i + 1) % 4], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(digport[(i + 2) % 4], digpin[(i + 2) % 4], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(digport[(i + 3) % 4], digpin[(i + 3) % 4], GPIO_PIN_RESET);

		for (j = 0; j < 7; ++j)
			HAL_GPIO_WritePin(segport[j], segpin[j],codes[expression[end - i] >= '0' && expression[end - i]<='9' ? expression[end - i] - '0' : expression[end - i] - 'A'+10] & 1<<j ? GPIO_PIN_SET : GPIO_PIN_RESET);

		HAL_Delay(4);
	}
	close();
}
void push (double c)
{

	stack_head++;
	stack[stack_head] = c;


	return;
}
void pop(void)
{
	if (stack_head>=0)
	{
		stack[stack_head] = '\0';
		stack_head--;}


	return;
}
uint8_t get(void)
{
	return stack[stack_head];
}
void calculate(int* suf)
{

    int length = end - 3;
    int i = 4, count = 0, temp = 0, num = 0;
    int node = 0;
    stack_head = -1;

    while (expression[i]!=0)
    {
        temp = i;
        count = 0;
        num = 0;
        while (expression[i] <= '9' && expression[i] >= '0') {
            count += 1;
            i++;
        }

        while (i - temp > 0) {
            count--;
            num += (expression[temp] - '0') * pow(10, count);
            temp++;
        }
        suf[node] = num;
        node++;

        if ((expression[i] <'0' ||expression[i] > '9')&& expression[i]!=0)
        {
      			while (isempty() != 1 &&precedence(expression[i]) <= precedence(get())  )
        			{

        				switch(get())
        				{case 'A':suf[node] = -1;break;
        				case 'M':suf[node] = -2;break;
        				case 'X':suf[node] = -3;break;
        				case 'D':suf[node] = -4;break;}
        				pop();
        				node ++;
        			}
        			push(expression[i]);
        			i++;

       	}


     }
        	while (isempty() != 1)
        	{
        		switch(get())
        				{case 'A':suf[node] = -1;break;
        				case 'M':suf[node] = -2;break;
        				case 'X':suf[node] = -3;break;
        				case 'D':suf[node] = -4;break;}
        		pop();
        		node++;
        	}
     suf[node] = -5;
}
int precedence(uint8_t op) {
    switch (op) {
        case 'A':
        case 'M':
            return 1;
        case 'X':
        case 'D':
            return 2;
        default:
            return 0;
    }
}
double calresult(int * suf)
{
	double result;
	double a, b;
	stack_head = -1;
	for (int i = 0; i < 100; i++) {
	    stack[i] = 0;
	}
	for(int i = 0;suf[i] != -5 ;++i)
	{
		if (suf[i] >=0)
			push(suf[i]);
		else
			{b=get();pop();a = get();pop();
			if (suf[i] ==-1){result = a+b;}
		if (suf[i] == -2) {result =a -b;}
		if(suf[i] == -3) {result = a*b;}
		if (suf[i] == -4) {result = a / b; }
			push(result);}


	}
	return getd();
}
void show_result(void)
{
	int i, j;

		for (i = 0; i < 4; ++i)
		{
			HAL_GPIO_WritePin(digport[i], digpin[i], GPIO_PIN_SET);
			HAL_GPIO_WritePin(digport[(i + 1) % 4], digpin[(i + 1) % 4], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(digport[(i + 2) % 4], digpin[(i + 2) % 4], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(digport[(i + 3) % 4], digpin[(i + 3) % 4], GPIO_PIN_RESET);

			for (j = 0; j < 7; ++j)
				HAL_GPIO_WritePin(segport[j], segpin[j],codes[digits[i]]&1<<j ? GPIO_PIN_SET:GPIO_PIN_RESET);

			HAL_Delay(0);
		}
		close();
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //show_4char();
	  uint8_t i,j;
	  //i = HAL_GetTick();
	  //keep_char(get_char());
	 double num;
	 int suf[100] = {0};

	  if (expression[end] == '\0'&&flag2 == 0)
	  {
		  calculate(suf);
		  num = calresult(suf);
		  getFirstFourDigits(num);
		  flag2 =1;


	  }
	  else if (flag2 == 1)
	  {
		  show_result();
	  }
	  else
	  {
		  show_4char();
		  i = get_char();
		  keep_char(i);
	  }
	  //if (i % 2000 == 1000)
	  	  //end += 1 ;






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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIG2_Pin|DIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG6_Pin|SEG5_Pin|SEG4_Pin|SEG7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KR3_Pin|KR2_Pin|KR1_Pin|KR0_Pin
                          |DIG0_Pin|DIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW3_Pin */
  GPIO_InitStruct.Pin = SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG2_Pin DIG3_Pin */
  GPIO_InitStruct.Pin = DIG2_Pin|DIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG5_Pin SEG4_Pin SEG7_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG5_Pin|SEG4_Pin|SEG7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin KR1_Pin KR0_Pin
                           DIG0_Pin DIG1_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin|KR1_Pin|KR0_Pin
                          |DIG0_Pin|DIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KC0_Pin */
  GPIO_InitStruct.Pin = KC0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KC0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
