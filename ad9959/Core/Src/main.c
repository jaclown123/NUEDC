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
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
#define SCLK_LOW HAL_GPIO_WritePin(GPIOC, 1<<0, GPIO_PIN_RESET)
#define SCLK_HIGH HAL_GPIO_WritePin(GPIOC, 1<<0, GPIO_PIN_SET)
#define CS_LOW HAL_GPIO_WritePin(GPIOC, 1<<1, GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(GPIOC, 1<<1, GPIO_PIN_SET)
#define SDIO0_LOW HAL_GPIO_WritePin(GPIOC, 1<<2, GPIO_PIN_RESET)
#define SDIO0_HIGH HAL_GPIO_WritePin(GPIOC, 1<<2, GPIO_PIN_SET)
#define UPDATE_LOW HAL_GPIO_WritePin(GPIOC, 1<<3, GPIO_PIN_RESET)
#define UPDATE_HIGH HAL_GPIO_WritePin(GPIOC, 1<<3, GPIO_PIN_SET)
//typedef struct {
//    // 这里假设csport是一个指向GPIO_TypeDef的指�?????????
//    GPIO_TypeDef* cs_port;
//    uint16_t cs_pin;
//    SPI_TypeDef * SPI_Hanlder;
//
//    // 在这里添加其他你�?????????要的变量
//    // 例如�?????????
//    // uint16_t cspin;
//} AD9959_Handler;
//
//void AD9959_Write(AD9959_Handler *device, AD9959_REG_ADDR reg_addr,  uint32_t data) //incomplete
//{
///* Determining which register is accessed */
//uint8_t instruction = WRITE_INST | (reg_addr & 0x1f);
//
//HAL_GPIO_WritePin(device->cs_port,device->cs_pin, GPIO_PIN_RESET);
//HAL_SPI_Transmit(device->SPI_Hanlder, &instruction, sizeof(instruction),10);
//
///* writing the value */
///* Finding the length of the current register to update */
//
//
//uint8_t i,transfer_data[4];
//uint8_t len = registers[reg_addr].size;
////MSB first
//i = 0;
//while(len-- >0){
//*(transfer_data + i) = (data >> len*8) & 0xFF;
//i++;
//}
//HAL_SPI_Transmit(device->SPI_Hanlder,transfer_data,i,10);
//HAL_GPIO_WritePin(device->cs_port,device->cs_pin, GPIO_PIN_SET);
//// Update device
//
////AD9959_Update(device);
//}

//uint32_t AD9959_Read(AD9959_Handler *device, AD9959_REG_ADDR reg_addr) //***ed
//{
//uint8_t read_instruction = READ_INST | (reg_addr & 0x1F);
//
//uint8_t receivdata;
//HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
//
//uint8_t len = registers[reg_addr].size;
//
//AD9959_SPI_Transfer(device, &read_instruction, sizeof(read_instruction));
//HAL_Delay(1);
//HAL_SPI_Receive(device->SPI_Handler, &receivdata, len, HAL_MAX_DELAY);
//
//
//HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
//
//return receivdata;
//}
//void ad9959_send(uint8_t *data,size_t size)
//{
//	HAL_GPIO_WritePin(GPIOA, 1<<10, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi3, data, size, 10);
//	HAL_GPIO_WritePin(GPIOA, 1<<10, GPIO_PIN_SET);
//
//}
//void ad9959_init(void)
//{
//	uint8_t FR1_DATA[4] = {0x01,0xD0,0x00,0x00};
//	uint8_t FR2_DATA[3] = {0x02,0x00,0x00};
//	ad9959_send(FR1_DATA, 4);
//	ad9959_send(FR2_DATA, 3);
//
//}
//void ad9959_freq(uint32_t freq , uint8_t channel)
//{
//	uint8_t data[2] = {0};
//	data[1] = channel;
//	ad9959_send(data, 2);
//	uint8_t freq_t[5] = {0};
//	freq_t[0] = 0x04;
//	uint32_t temp = freq * 4294967296 / 500000000;
//	freq_t[4] = (uint8_t)temp;
//	freq_t[3] = (uint8_t)(temp>>8);
//	freq_t[2] = (uint8_t)(temp>>16);
//	freq_t[1] = (uint8_t) (temp >>24);
//	ad9959_send(freq_t, 5);
//
//}
//void ad9959_update(void)
//{
//	HAL_GPIO_WritePin(GPIOA, 1<<9, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOA, 1<<9, GPIO_PIN_SET);
//	HAL_Delay(2);
//	HAL_GPIO_WritePin(GPIOA, 1<<9, GPIO_PIN_RESET);
//}
void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData)
{
	uint8_t	ControlValue = 0;
	uint8_t	ValueToWrite = 0;
	uint8_t	RegisterIndex = 0;
	uint8_t	i = 0;

	ControlValue = RegisterAddress;
//д����?
	SCLK_LOW;
	CS_LOW;
	for(i=0; i<8; i++)
	{
		SCLK_LOW;
		if(0x80 == (ControlValue & 0x80))
		SDIO0_HIGH;
		else
			SDIO0_LOW;
		SCLK_HIGH;
		ControlValue <<= 1;
	}
	SCLK_LOW;
//д������
	for (RegisterIndex=0; RegisterIndex<NumberofRegisters; RegisterIndex++)
	{
		ValueToWrite = RegisterData[RegisterIndex];
		for (i=0; i<8; i++)
		{
			SCLK_LOW;
			if(0x80 == (ValueToWrite & 0x80))
			SDIO0_HIGH;
			else
			SDIO0_LOW;
			SCLK_HIGH;
			ValueToWrite <<= 1;
		}
		SCLK_LOW;
	}
  CS_HIGH;
}
void Write_CFTW0(uint32_t fre)
{
		uint8_t CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};	//�м����??
	  uint32_t Temp;
	  Temp=(uint32_t)fre * 4294967296 / 500000000;
	  CFTW0_DATA[3]=(uint8_t)Temp;
	  CFTW0_DATA[2]=(uint8_t)(Temp>>8);
	  CFTW0_DATA[1]=(uint8_t)(Temp>>16);
	  CFTW0_DATA[0]=(uint8_t)(Temp>>24);
		AD9959_WriteData(0x04,4,CFTW0_DATA);//CTW0 address 0x04
}
void AD9959_Set_Fre(uint8_t Channel,uint32_t Freq)
{
		uint8_t CHANNEL[1] = {0x00};

		CHANNEL[0]=Channel;
		AD9959_WriteData(0x00,1,CHANNEL);//���ƼĴ���д��CHnͨ����ѡ��CHn
    Write_CFTW0(Freq);//���CHn�趨Ƶ��
}
void IntReset()
{
	HAL_GPIO_WritePin(GPIOA, 1<<1, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, 1<<1, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, 1<<1, 0);
}
void Intserve(void)
{
		//AD9959_PWR=0;
	HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, 0);
    CS_HIGH;
    SCLK_LOW;
    UPDATE_LOW;
//    PS0 = 0;
//    PS1 = 0;
//    PS2 = 0;
//    PS3 = 0;
    SDIO0_HIGH;
//    SDIO1 = 0;
//    SDIO2 = 0;
//    SDIO3 = 0;
}
void Write_ACR(uint16_t Ampli)
{
	uint32_t A_temp=0;
	uint8_t ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout
  A_temp=Ampli|0x1000;

	ACR_DATA[1] = (uint8_t)(A_temp>>8); //高位数据
	ACR_DATA[2] = (uint8_t)A_temp;  //低位数据
  AD9959_WriteData(0x06,3,ACR_DATA); //ACR address 0x06.CHn设定幅度
}

void AD9959_Set_Amp(uint8_t Channel, uint16_t Ampli)
{
	uint8_t CHANNEL[1] = {0x00};

	CHANNEL[0]=Channel;
	AD9959_WriteData(0x00,1,CHANNEL); //控制寄存器写入CHn通道，选择CHn
	Write_ACR(Ampli);							//	CHn设定幅度
}
void Write_CPOW0(uint16_t Phase)
{
	uint8_t CPOW0_data[2] = {0x00,0x00};
	CPOW0_data[1]=(uint8_t)Phase;
	CPOW0_data[0]=(uint8_t)(Phase>>8);

	AD9959_WriteData(0x05,2,CPOW0_data);//CPOW0 address 0x05.CHn设定相位
}

void AD9959_Set_Phase(uint8_t Channel,uint16_t Phase)
{
	uint8_t CHANNEL[1] = {0x00};
	CHANNEL[0]=Channel;

	AD9959_WriteData(0x00,1,CHANNEL); //控制寄存器写入CHn通道，选择CHn
	Write_CPOW0(Phase);//CHn设定相位
}

void AD9959_Init(void)
{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 //PA,PB,PC端口时钟使能
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//初始化管脚PA2.3.4.5.6.7.8.9.10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口�?�度�?50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10;//初始化管脚PB0.1.10
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口�?�度�?2MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	//初始化管脚PC0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO口�?�度�?2MHz
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC
//
	Intserve();  //IO口电平状态初始化
  IntReset();  //AD9959复位

	//初始化功能寄存器
  uint8_t FR1_DATA[3] = {0xD0,0x00,0x00};//VCO gain control[23]=1系统时钟高于255Mhz; PLL[22:18]=10100,20倍频,20*25M=500MHZ; Charge pump control = 75uA


  uint8_t FR2_DATA[2] = {0x00,0x00};	// 双方向扫描，即从起始值扫到结束�?�后，又从结束�?�扫到起始�??
  AD9959_WriteData(0x01,3,FR1_DATA);//写功能寄存器1
  AD9959_WriteData(0x02,2,FR2_DATA);//
}
void IO_Update(void)
{
	UPDATE_LOW;
	HAL_Delay(1);
	UPDATE_HIGH;
	HAL_Delay(3);
	UPDATE_LOW;
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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
//  ad9959_init();
//  ad9959_freq(30000000, 0x10);
//  ad9959_update();
  AD9959_Init();
  AD9959_Set_Fre(0x10, 30000000);
  AD9959_Set_Amp(0x10, 1023);
  //AD9959_Set_Phase(0x10, 0);
  IO_Update();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 60;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCLK_Pin|CS_Pin|SDIO0_Pin|UPDATE_Pin
                          |PWR_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pins : SCLK_Pin CS_Pin SDIO0_Pin UPDATE_Pin
                           PWR_Pin PC5 */
  GPIO_InitStruct.Pin = SCLK_Pin|CS_Pin|SDIO0_Pin|UPDATE_Pin
                          |PWR_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin PA9 PA10 */
  GPIO_InitStruct.Pin = RESET_Pin|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
