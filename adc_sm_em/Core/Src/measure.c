/*
 * measure.c
 *
 *  Created on: Jul 17, 2024
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

uint16_t max = 0 ;
uint16_t min = 4097;



uint16_t get_max_min(uint16_t * buffer, size_t size)
{
	int i = 1 ;

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

void samp(uint8_t * buffer,size_t size ,TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	HAL_TIM_Base_Start(htim);

	HAL_ADC_Start_DMA(hadc, (uint32_t *) buffer, size);

    adc_ongoing = 1;
    while (1)
    {
    	if (adc_ongoing == 0)
    		break;
    }
    return;
    //uart_transmit(adc_buffer,2050);
}
uint32_t sys_clock = 150e6;
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
//	uint32_t base = 0;
//	for (int i = 1; i != length; ++i)
//	{
//	  if (pData[i] < (pData[i-1] & ((1<<16)-1)))
//		  base += 1u << 32;
//
//      pData[i] += base;
//	}
	//HAL_TIM_Base_Start(htim_sec);
	//LL_TIM_SetCounter(htim_sec->Instance, 0);
	//float cyc = ic_fitting(pData + 1, length - 1);
	//uint32_t time = LL_TIM_GetCounter(htim_sec->Instance);
	//HAL_TIM_Base_Stop(htim_sec);
    //const float f_cpu = 150.f;
    //float freq = f_cpu / cyc;
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
	//htim->Instance->DIER =1;
//	htim->Instance->DIER |=1<<6;
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

float get_freq_dynamic(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length, TIM_HandleTypeDef *htim_sec )
{
	HAL_DAC_SetValue(hdac, dac_channel, Alignment , threshold);
	HAL_DAC_Start(hdac, dac_channel);
	HAL_COMP_Start(hcomp);

    float gap = 0;
	HAL_TIM_IC_Start_DMA(htim, tim_channel, pData, 10);
	ic_ongoing = 1;
	while (ic_ongoing)
	  ;
	uint32_t base = 0;
	for (int i = 1; i != 10; ++i)
	{
	  if (pData[i] < (pData[i-1] & ((1<<16)-1)))
		  base += 1u << 16;
        pData[i] += base;
	}
	for (int i = 1; i <= 10; ++i)
	{
        gap += (pData[i] - pData[i-1]) / 9;
	}
	if(gap > 15000)
	{
		float low = gap / 15000 ;
		length =  100;
	}
	HAL_TIM_IC_Start_DMA(htim, tim_channel, pData, length);
	ic_ongoing = 1;
	while (ic_ongoing)
	  ;
	base = 0;
	for (int i = 1; i != length; ++i)
	{
	  if (pData[i] < (pData[i-1] & ((1<<16)-1)))
		  base += 1u << 16;
      pData[i] += base;
	}
//	HAL_TIM_Base_Start(htim_sec);
//	LL_TIM_SetCounter(htim_sec->Instance, 0);
//	float cyc = ic_fitting(pData + 1, length - 1);
//	uint32_t time = LL_TIM_GetCounter(htim_sec->Instance);
//	HAL_TIM_Base_Stop(htim_sec);
//    const float f_cpu = 150.f;
//    float freq = f_cpu / cyc;
	uint32_t t1 = pData[1];
	uint32_t t2 = pData[length-1];
	float freq = 150.f/ (t2 - t1) * (length-2);
	return freq ;
}






