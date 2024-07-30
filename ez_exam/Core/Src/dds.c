/*
 * dds.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: LZF12
 */

#include <stdint.h>
#include <stdio.h>
//1为正弦波�????? 2为三角波 ,3为正弦波+方波 方波在sqr引脚输出
void set_freq(void (*send_data)(uint16_t), long long int freq,int c)
{

	long long int temp = (freq * 268435456 / 100000000) ;
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
	send_data(0xc000);
	send_data(0x2100);
	send_data(0x8000);//freq1
	send_data(0x8432);

	//send_data(0x0028);
	switch(c)
		{
			case 1:send_data(0x0000);break;//send_data(0x0200);
			case 2:send_data(0x0002);break;
			case 3:send_data(0x0028); break;
		}
	//send_data(0x2200);
}


