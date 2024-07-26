/*
 * dds.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: LZF12
 */

//#include <stdint.h>
//#include <stdio.h>
//#include <stm32g4xx_hal_cordic.h>
#include <main.h>
#include "dds.h"


void set_n_freq(void (*send_9834)(uint16_t),void (*send_9833)(uint16_t),
		long long int freq, int n, int rad)
{
	long long int temp_1 = (freq * 268435456 / 25000560);// - 6 - 3 * (freq / 1e4 - 2);
	uint32_t fr = temp_1;
	uint16_t lsb = 0x4000;
	uint16_t msb = 0x4000;
	int rad_t = rad - 8.2 * freq * n * 0.36 / 1000;
    if(rad_t < 0) rad_t += 360;
    if(rad_t > 360) rad_t -= 360;
    uint16_t phase = rad_t * 512 / 45;
	uint16_t ph_reg = 0xc001;

	for (int i = 0; i < 12; ++i)
	{
	    if (phase & (1 << i)) {
	        ph_reg |= (1 << i);
	    }
	}
	for (int i = 0; i < 14; ++i)
	{
	    if (fr & (1 << i)) {
	        lsb |= (1 << i);
	    }
	    if (fr & (1 << (i + 14))) {
	        msb |= (1 << i);
	    }
	}

	long long int temp_2 = n * temp_1;
	uint32_t fr_2 = temp_2;
	uint16_t lsb2 = 0x4000;
	uint16_t msb2 = 0x4000;
	for (int i = 0; i < 14; ++i) {
		if (fr_2 & (1 << i)) {
			lsb2 |= (1 << i);
		}
		if (fr_2 & (1 << (i + 14))) {
			msb2 |= (1 << i);
		}
	}

	send_9834(0x2100);//send_data(0x2300);
	send_9834(lsb);//freq0
	send_9834(msb);
	send_9834(0xc001);


	send_9833(0x2100);//send_data(0x2300);
	send_9833(lsb2);//freq0
	send_9833(msb2);
	send_9833(ph_reg);

	send_9833(0x2000);//send_data(0x0200);
    send_9834(0x0000);//sine wave;

}

//0: sine    1:triangle    2:square /out:sqr
void set_freq(void (*send_data)(uint16_t), long long int freq, int c)
{
	//9834
	//20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100
	//-6,-6,-9,

	//9833
	//
	long long int temp = (freq * 268435456 / 25000000) - 6 - 3 * (freq / 1e4 - 2);
	uint32_t fr = temp;
	uint16_t lsb = 0x4000;
	uint16_t msb = 0x4000;
	for (int i = 0; i < 14; ++i) {
	    // 对于lsb，直接取fr对应�?????
	    if (fr & (1 << i)) {
	        lsb |= (1 << i);
	    }
	    if (fr & (1 << (i + 14))) {
	        msb |= (1 << i);
	    }
	}
	send_data(0x2100);//send_data(0x2300);
	send_data(lsb);//freq0
	send_data(msb);
	send_data(0xc001);
//	send_data(0x2100);
//	send_data(0x8000);//freq1
//	send_data(0x8432);
//  send_data(0x0028);
	switch(c)
	{
		case 0:send_data(0x0000);break;//send_data(0x0200);
		case 1:send_data(0x0002);break;
		case 2:send_data(0x0028);break;
	}
	//send_data(0x2200);
}


//#define length 10
//void adc_gen_sin(int freq, CORDIC_HandleTypeDef * hcordic, DAC_HandleTypeDef * hdac4 , uint32_t DAC_CHANNEL_1)
//  {
//	uint32_t temp1[length];
//
//  uint32_t temp2[length];
//  uint16_t hsdac_buffer[length];
//  for (int i = 0; i < length; ++i)
//  {
//	  temp1[i] = (1llu<<32) / length * i;
//  }
//
//  HAL_CORDIC_CalculateZO(&hcordic, temp1, temp2, length,10);
//  for (int i= 0 ; i < length; ++i)
//  {
//	  hsdac_buffer[i] = (temp2[i] + (1<< 31))>>21;
//	  hsdac_buffer[i] += 512;
//  }
//  HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_1, hsdac_buffer, length / 2, DAC_ALIGN_12B_R);
//  HAL_TIM_Base_Start(&htim7);
//  (&htim7)->Instance->ARR = (uint32_t)(150 / length - 1);
//  HAL_OPAMP_Start(&hopamp4);
//
//  }
//










