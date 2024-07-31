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
DAC_HandleTypeDef hdac3;
DMA_HandleTypeDef hdma_dac3_ch1;
DMA_HandleTypeDef hdma_dac3_ch2;

OPAMP_HandleTypeDef hopamp3;
OPAMP_HandleTypeDef hopamp6;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_OPAMP6_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SCLK_LOW HAL_GPIO_WritePin(GPIOC, 1<<0, GPIO_PIN_RESET)
#define SCLK_HIGH HAL_GPIO_WritePin(GPIOC, 1<<0, GPIO_PIN_SET)
#define CS_LOW HAL_GPIO_WritePin(GPIOC, 1<<1, GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(GPIOC, 1<<1, GPIO_PIN_SET)
#define SDIO0_LOW HAL_GPIO_WritePin(GPIOC, 1<<2, GPIO_PIN_RESET)
#define SDIO0_HIGH HAL_GPIO_WritePin(GPIOC, 1<<2, GPIO_PIN_SET)
#define UPDATE_LOW HAL_GPIO_WritePin(GPIOC, 1<<3, GPIO_PIN_RESET)
#define UPDATE_HIGH HAL_GPIO_WritePin(GPIOC, 1<<3, GPIO_PIN_SET)


int mode = 0;
int mode_t = 0;
int ampl = 580;
int ampl_t = 580;
int modual = 100;
int modual_t = 100;
int delay = 0;
int delay_t = 0;
int atten = 0;
int atten_t = 0;
int freq = 30000000;
int freq_t = 30000000;
int phase = 0;
int phase_t = 0;
int dac_phase_delay = 0;

float DB[11] = {
1000,
891,
630,
500,
398,
316,
251,
199,
158,
126,
100 };

void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData)
{
	uint8_t	ControlValue = 0;
	uint8_t	ValueToWrite = 0;
	uint8_t	RegisterIndex = 0;
	uint8_t	i = 0;
	ControlValue = RegisterAddress;
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
	uint8_t CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};
	uint32_t Temp;
	Temp=(uint32_t)fre * 4294967296 / 500000000;
	CFTW0_DATA[3]=(uint8_t)Temp;
	CFTW0_DATA[2]=(uint8_t)(Temp>>8);
	CFTW0_DATA[1]=(uint8_t)(Temp>>16);
	CFTW0_DATA[0]=(uint8_t)(Temp>>24);
	AD9959_WriteData(0x04,4,CFTW0_DATA);//CTW0 address 0x04
}
void AD9959_Set_Freq(uint8_t Channel,uint32_t Freq)
{
	uint8_t CHANNEL[1] = {0x00};
	CHANNEL[0]=Channel;
	AD9959_WriteData(0x00,1,CHANNEL);
    Write_CFTW0(Freq);
}

void Write_ACR(uint16_t Ampl)
{
	uint32_t A_temp=0;
	uint8_t ACR_DATA[3] = {0x00,0x00,0x00};//default Value = 0x--0000 Rest = 18.91/Iout
    A_temp=Ampl|0x1000;
	ACR_DATA[1] = (uint8_t)(A_temp>>8); //高位数据
	ACR_DATA[2] = (uint8_t)A_temp;  //低位数据
    AD9959_WriteData(0x06, 3, ACR_DATA); //ACR address 0x06.CHn设定幅度
}

void AD9959_Set_Ampl(uint8_t Channel, uint16_t Ampl)
{
	uint8_t CHANNEL[1] = {0x00};
	CHANNEL[0]=Channel;
	AD9959_WriteData(0x00,1,CHANNEL); //控制寄存器写入CHn通道�????
	Write_ACR(Ampl);							//	CHn设定幅度
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
	AD9959_WriteData(0x00,1,CHANNEL); //控制寄存器写入CHn通道�??
	Write_CPOW0(Phase);//CHn设定相位
}

void AD9959_Init(void)
{
  Intserve();  //IO口电平状态初始化
  IntReset();  //AD9959复位
	//初始化功能寄存器
  uint8_t FR1_DATA[3] = {0xD0,0x00,0x00};//VCO gain control[23]=1系统时钟高于255Mhz; PLL[22:18]=10100,20倍频,20*25M=500MHZ; Charge pump control = 75uA
  uint8_t FR2_DATA[2] = {0x00,0x00};	// 双方向扫描，即从起始值扫到结束�?�后，又从结束�?�扫到起始�??
  AD9959_WriteData(0x01,3,FR1_DATA);//写功能寄存器1
  AD9959_WriteData(0x02,2,FR2_DATA);
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
	HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, 0);
    CS_HIGH;
    SCLK_LOW;
    UPDATE_LOW;
    HAL_GPIO_WritePin(PS0_GPIO_Port, PS0_Pin, 0);
    HAL_GPIO_WritePin(PS1_GPIO_Port, PS1_Pin, 0);
    HAL_GPIO_WritePin(PS2_GPIO_Port, PS2_Pin, 0);
    //HAL_GPIO_WritePin(PS3_GPIO_Port, PS3_Pin, 0);
    SDIO0_HIGH;
    HAL_GPIO_WritePin(SDIO1_GPIO_Port, SDIO1_Pin, 0);
    HAL_GPIO_WritePin(SDIO2_GPIO_Port, SDIO2_Pin, 0);
    HAL_GPIO_WritePin(SDIO3_GPIO_Port, SDIO3_Pin, 0);
}
void IO_Update(void)
{
	UPDATE_LOW;
	HAL_Delay(1);
	UPDATE_HIGH;
	HAL_Delay(3);
	UPDATE_LOW;
}

#define offset 621
#define dac_length 4 //对应500mV，校准时可能�????要改
uint16_t scaled_sine_wave_table_Sd[dac_length];
uint16_t scaled_sine_wave_table_Sm[dac_length];
void set_dac(uint16_t modulation,int16_t phase)
{
	HAL_TIM_Base_Stop(&htim15);
  HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_1);
  HAL_DAC_Stop_DMA(&hdac3, DAC_CHANNEL_2);

  uint16_t max_val;
  uint16_t min_val;
  phase -= 7;
  if(phase < 0) phase += 360;
  if(phase > 360) phase -= 360;
  max_val=(offset*modulation/100)+offset;
  min_val=offset-(offset*modulation/100);

  float sine_wave_table[dac_length];
  float sine_wave_table_phase[dac_length];

  float step = 2 * M_PI / dac_length;

  for (int i = 0; i < dac_length; i++) {
    sine_wave_table[i] = sinf(i * step);
    sine_wave_table_phase[i] = sinf(i * step+ ((float)phase)* M_PI/180.0 );
  }
  for (int i = 0; i < dac_length; i++) {
    scaled_sine_wave_table_Sd[i] = (uint16_t)((sine_wave_table[i] + 1) * (max_val - min_val) / 2 + min_val);
    scaled_sine_wave_table_Sm[i] = (uint16_t)((sine_wave_table_phase[i] + 1) * (max_val - min_val) / 2 + min_val);
  }
  HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_1,scaled_sine_wave_table_Sd, dac_length / 2, DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac3, DAC_CHANNEL_2,scaled_sine_wave_table_Sm, dac_length / 2, DAC_ALIGN_12B_R);

  (&htim15)->Instance->ARR = (uint32_t)(19);
  HAL_OPAMP_Start(&hopamp6);
  HAL_OPAMP_Start(&hopamp3);
  HAL_TIM_Base_Start(&htim15);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_DAC3_Init();
  MX_OPAMP3_Init();
  MX_OPAMP6_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  setup();
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  GPIO_PinState prev = GPIO_PIN_SET;
  AD9959_Init();
  AD9959_Set_Ampl(0xF0, 1000);
  AD9959_Set_Phase(0xF0, 0);
  AD9959_Set_Freq(0xF0, freq);
  IO_Update();
  set_dac(modual, dac_phase_delay);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8);   //读取电机转动方向
	  int CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim8);
	  int counter = CaptureNumber / 4 % 7 ;

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
              if(mode < 2)
			  {
            	  modual = 0;
            	  set_dac(modual,dac_phase_delay );
			  }
              else
              {
            	  modual = 100;
            	  set_dac(modual,dac_phase_delay );
              }
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
			  AD9959_Set_Ampl(0xF0, ampl * 580 / 1000);
			  IO_Update();
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
			  set_dac(modual, dac_phase_delay);
			  break;
		  case 3:
			  lcd_show_str(100, 145,"DELAY:\n");
			  while (1)
			  {
				  delay = abs((CaptureNumber) / 4 % 7 * 30 + 50);
				  if(delay == 230) delay = 0;
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
			  float final_phase = (float)delay * (float)1e-9 * (float)freq ;
			  int integer = final_phase;
			  float set_phase = 1 - final_phase + (float)integer + (float)phase/360;
			  int set_phase_int = set_phase;
			  set_phase = (set_phase - (float)set_phase_int) * 16383;
			  AD9959_Set_Phase(0x80, set_phase);
			  IO_Update();
			  dac_phase_delay = delay * 360 / 500 ;
			  set_dac(modual, dac_phase_delay );
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
			  AD9959_Set_Ampl(0x80, DB[atten / 2] * ampl / 1000);
			  IO_Update();
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
			  AD9959_Set_Freq(0xF0, freq * 1000000);
			  IO_Update();
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
			  AD9959_Set_Phase(0x80, phase * 16383 / 360);
			  IO_Update();
			  break;
		  }
		  while(HAL_GPIO_ReadPin(GPIOC, 1<<13) != 0)		  ;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = ENABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T15_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief OPAMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP6_Init(void)
{

  /* USER CODE BEGIN OPAMP6_Init 0 */

  /* USER CODE END OPAMP6_Init 0 */

  /* USER CODE BEGIN OPAMP6_Init 1 */

  /* USER CODE END OPAMP6_Init 1 */
  hopamp6.Instance = OPAMP6;
  hopamp6.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp6.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp6.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp6.Init.InternalOutput = DISABLE;
  hopamp6.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp6.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP6_Init 2 */

  /* USER CODE END OPAMP6_Init 2 */

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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCLK_Pin|CS_Pin|SDIO0_Pin|UPDATE_Pin
                          |PWR_Pin|PS0_Pin|SDIO1_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|SDIO2_Pin|SDIO3_Pin|PS1_Pin
                          |PS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SCLK_Pin CS_Pin SDIO0_Pin UPDATE_Pin
                           PWR_Pin PS0_Pin SDIO1_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|CS_Pin|SDIO0_Pin|UPDATE_Pin
                          |PWR_Pin|PS0_Pin|SDIO1_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin SDIO2_Pin SDIO3_Pin PS1_Pin
                           PS2_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|SDIO2_Pin|SDIO3_Pin|PS1_Pin
                          |PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
