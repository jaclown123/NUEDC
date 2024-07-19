/*
 * measure.h
 *
 *  Created on: Jul 17, 2024
 *      Author: Jac1own
 */

#ifndef INC_MEASURE_H_
#define INC_MEASURE_H_

#include <stdint.h>

extern volatile int adc_ongoing;
extern volatile int ic_ongoing;
extern volatile uint32_t yichu_counter;
uint16_t *make_8to16(uint8_t * buffer, size_t size, uint16_t * new_buffer);
uint16_t get_max_min(uint16_t * buffer, size_t size);
void samp(uint8_t * buffer,size_t size ,TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc);
void set_sm_freq(uint32_t freq , TIM_HandleTypeDef *htim);
float get_freq(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length , TIM_HandleTypeDef * htim_sec );
uint32_t get_fit_sm_hz(float freq);
void fft_to_1024(float * in_data ,float * out_data, float * mo , uint32_t index ,float max_value);
float get_freq_dynamic(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length, TIM_HandleTypeDef *htim_sec );
float get_freq_low(COMP_HandleTypeDef * hcomp ,DAC_HandleTypeDef * hdac,
		uint32_t dac_channel, uint32_t Alignment,uint32_t threshold , TIM_HandleTypeDef *htim,
		uint32_t tim_channel, uint32_t *pData, uint16_t length, TIM_HandleTypeDef *htim_sec );

#endif /* INC_MEASURE_H_ */
