/*
 * fitting.h
 *
 *  Created on: Jul 18, 2024
 *      Author: Jac1own
 */

#ifndef INC_FITTING_H_
#define INC_FITTING_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

float ic_fitting(uint32_t times[], uint16_t size);
float lia_fitting(float signal[], float sin_basis[], float cos_basis[], float dc_basis[], uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* INC_FITTING_H_ */
