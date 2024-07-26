/*
 * measure.c
 *
 *  Created on: July 17, 2024
 *      Author: Jac1own
 */
#include <main.h>
#include "measure.h"
#include <stdio.h>
#include <stdint.h>
#include <stm32g4xx_ll_tim.h>
#include "fitting.h"
#include <arm_math.h>
/*
 * need to define the following def in main.c
 * void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc != &hadc1)
		return;
	adc_ongoing = 0;
	HAL_ADC_Stop_DMA(hadc);
	HAL_TIM_Base_Stop(&htim6);
}
 */
volatile int adc_ongoing = 0;
volatile int ic_ongoing = 0;
volatile uint32_t yichu_counter = 0;
uint32_t sys_clock = 150e6;
#define N 1024

uint16_t *make_8to16(uint8_t *buffer, size_t size , uint16_t *new_buffer)
{
	int i = 0;
	if (size % 2 != 0)
		return 0;
	//uint16_t buffer_16[size / 2];
	while (i < size / 2)
	{
		new_buffer[i] = buffer[2*i] | buffer[2 * i + 1] << 8;
		++i;
	}
	return new_buffer;
}

uint16_t get_max_min(uint16_t * buffer, size_t size)
{
	int i = 1 ;
	uint16_t max = 0 ;
	uint16_t min = 4097;
	while (i < size)
	{
		if (max < buffer[i])
		{
			max = buffer[i];
		}
		if (min > buffer[i])
		{
			min = buffer[i];
		}
		i++;
	}
	return (max + min) / 2 ;
}

uint16_t get_ampl(uint16_t * buffer, size_t size)
{
	int i = 1 ;
	uint16_t max = 0 ;
	uint16_t min = 4097;
	while (i < size)
	{
		if (max < buffer[i])
		{
			max = buffer[i];

		}
		if (min > buffer[i])
		{
			min = buffer[i];
		}
		i++;
	}
	uint16_t ampl = max - min;
	return ampl;
}

void samp(uint8_t *buffer, size_t size, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	HAL_TIM_Base_Start(htim);
	HAL_ADC_Start_DMA(hadc, (uint32_t *)buffer, size);
    adc_ongoing = 1;
    while (1)
    {
    	if (adc_ongoing == 0)
    		break;
    }
    return;
}

void set_sm_freq(uint32_t freq, TIM_HandleTypeDef *htim)
{
	uint32_t set = sys_clock / freq - 1;
	if (set <38)
		set = 38;
	htim->Instance->ARR = set;
}

float get_freq(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length, TIM_HandleTypeDef *htim_sec )
{
	HAL_DAC_SetValue(hdac, dac_channel, Alignment , threshold);
	HAL_DAC_Start(hdac, dac_channel);
	HAL_COMP_Start(hcomp);
	HAL_TIM_IC_Start_DMA(htim, tim_channel, pData, length);
	ic_ongoing = 1;
	while (ic_ongoing)
	  ;
	uint32_t counter = 0;
	uint32_t t1 = pData[1];
	uint32_t t2 = pData[length-1];
	if (t2 < t1)
		t2 = (1<<32) - t1 + t2;
	float freq = (float)(150 * 1e6)/ (t2 - t1) * (length-2);
	uint32_t T = 150e6 / freq;
	for (int i = 2 ; i < length ; ++i)
	{
		if( (((pData[i] - pData[i - 1]) * 10 )< (7 * T))
		  && (((pData[i - 1] - pData[i - 2])* 10) < (7*T)))
			counter ++;
	}
	freq = (float)(150 * 1e6)/ (t2 - t1) * (length-2 - counter);
	return freq ;
}

float get_freq_low(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length, TIM_HandleTypeDef *htim_sec )
{
	HAL_DAC_SetValue(hdac, dac_channel, Alignment , threshold);
	HAL_DAC_Start(hdac, dac_channel);
	HAL_COMP_Start(hcomp);
    HAL_TIM_Base_Start_IT(htim);
	HAL_TIM_IC_Start_DMA(htim, tim_channel, pData, length);
	ic_ongoing = 1;
	while (ic_ongoing)
	  ;
	uint32_t t1 = pData[0];
	uint32_t t2 = pData[length - 1] + yichu_counter * 65536;
	float freq = 150.f / (t2 - t1) * (length - 1);
	yichu_counter = 0;
	return freq;
}

uint32_t get_fit_sm_hz(float freq)
{
	uint32_t a = freq ;
	return a*1e3/4;
}

void int_to_float(uint16_t* buffer, float* fft_in)
{
	for(int i = 0;i != N;i++){
		fft_in[i] = buffer[i];
	}
}
void fft_transfer(float* fft_in, float* fft_out, float* fft_mag)
{
	//float fft_in[N];
	//float fft_out[N];
	//float fft_mag[N];
	arm_rfft_fast_instance_f32 S;
	arm_rfft_1024_fast_init_f32(&S);
	arm_rfft_fast_f32(&S, fft_in, fft_out, 0);
	arm_cmplx_mag_f32(fft_out, fft_mag, N);
}

float LIA_get_mag(float* signal, float* temp, float* coef)
{
	float cos_basis[N];
	float sin_basis[N];
	float dc[N];
	float cos_sqr = 0;
	float sin_sqr = 0;
	for (int i = 0; i < N; i++) {
		dc[i] = 1;
	}
	float mag =lia_fitting(signal,sin_basis,cos_basis,dc,N);
	return mag;
}

float get_LIA_freq(float* signal, int freq_in , float* dingzhen)
{
	float dir = -1;
	float grad = 0.1;
	float test_freq = freq_in ;
	float cos_basis[N];
	float sin_basis[N];
	float dc[N];
	float mag[200] = {0};
	float freq_buffer[200] = {0};
	int head = 0;
	do
	{
		for (int i = 0; i < N; i++) {
			cos_basis[i] = arm_cos_f32(i * (2 * PI / 1000) * test_freq);
			sin_basis[i] = arm_sin_f32(i * (2 * PI / 1000) * test_freq);
			dc[i] = 1;
		}
		mag[head] = lia_fitting(signal, sin_basis, cos_basis, dc, N);
		freq_buffer[head] = test_freq;
		if (head != 0 && mag[head] < mag[head - 1])
		{
			test_freq = freq_buffer[head - 1];
			dir *= -1;
		}
		if (head >= 2 && mag[head - 1] > mag[head - 2]&& mag[head - 1] > mag[head] )
		{
			*dingzhen = freq_buffer[head - 1];
			return;
		}
		head ++;
		test_freq += grad * dir;
		if(test_freq == freq_in + 1||test_freq == freq_in - 1) return;
	} while(1);
}

void LIA_transfer(float* fft_in, int num, float* coef)
{
	float cos_basis[N];
	float sin_basis[N];
	float dc[N];
    /*int length = 10;
    uint32_t temp1[length];
    uint32_t temp2[length];
    uint16_t hsdac_buffer[length];
    for (int i = 0; i < length; ++i)
    {
	  temp1[i] = (1llu<<32) / length * i;
    }
    HAL_CORDIC_CalculateZO(&hcordic, temp1, temp2, length,10);
    for (int i= 0 ; i < length; ++i)
    {
	  hsdac_buffer[i] = (temp2[i] + (1<< 31))>>21;
	  hsdac_buffer[i] += 512;
    }*/
	float cos_sqr = 0;
	float sin_sqr = 0;
	for (int i = 0; i < N; i++) {
		uint32_t temp = i * (2 * PI / N) * num ;
		cos_basis[i] = arm_cos_f32(i * (2 * PI / N) * num);
		sin_basis[i] = arm_sin_f32(i * (2 * PI / N) * num);
		dc[i] = 1;
	}
	for (int i = 0; i < N; i++){
		cos_sqr += cos_basis[i] * cos_basis[i];
		sin_sqr += sin_basis[i] * sin_basis[i];
	}
	//float coef[3] = {0};
	for (int i = 0; i < N; i++) {
		coef[0] += cos_basis[i] * fft_in[i] / cos_sqr;
		coef[1] += sin_basis[i] * fft_in[i] / sin_sqr;
		coef[2] += fft_in[i]/N;
	}
}

void fft_to_1024(float * in_data ,float * out_data, float * mo , uint32_t index ,float max_value)
{
	arm_rfft_fast_instance_f32 rfft_instance;
	arm_rfft_1024_fast_init_f32(&rfft_instance);
	arm_rfft_fast_f32(&rfft_instance, in_data, out_data, 1);
	//计算复数序列模值平方；
	arm_cmplx_mag_squared_f32(out_data, mo, 1024);
	//计算复数序列模值
	arm_cmplx_mag_f32(out_data, mo,	 1024);
	arm_max_f32(mo, 1024, &max_value, &index);
}

void set_phase()
{

}
/**
  * @brief  set sigma_delta modulation
 */
/*void sigma_delta(uint16_t value_16bit, uint16_t* seq_12bit) {
  uint16_t base = value_16bit >> 4;
  uint8_t n = value_16bit & 0xf;
  uint8_t e=0;
  for (int i=0;i<16;i++) {
    e+=n;
    if (e>=8) {
      e-=16;
      seq_12bit[i]=base+1;
    }
    else{
      seq_12bit[i]=base;
    }
  }
}*/

/**
  * @brief  set AWG offset
  * @param  offset_level uint16_t from 0 to 65520, 16bit DAC ,from -5V to 5V
  */

/*
void AWG_Offset(uint16_t offset_level){
  if (offset_level <0 || offset_level > 65520) {
      return ;
    }
  else {
    sigma_delta(offset_level,awg_offset_level_buffer);
  }
}

void AWG_Offset_Init(){
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)awg_offset_level_buffer, 16, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim7);
}

void AWG_Offset_Stop(){
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim7);
}


void AWG_Gain(uint16_t gain_level){
  if (gain_level<=0 || gain_level>4) {
    return;
  }
  else {
    uint8_t Gain_Levels[4]={
          AWG_GAIN_1, AWG_GAIN_2,
          AWG_GAIN_3, AWG_GAIN_4
      };
    gain_state |= Gain_Levels[gain_level-1];
    HAL_GPIO_WritePin(SIPO_CS_GPIO_Port, SIPO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &gain_state, 1, 1000);
    HAL_GPIO_WritePin(SIPO_CS_GPIO_Port, SIPO_CS_Pin, GPIO_PIN_SET);
  }
}*/




