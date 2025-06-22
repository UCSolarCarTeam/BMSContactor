/*
 * ADC.h
 *
 *  Created on: Jun 21, 2025
 *      Author: violet
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "boardDefines.h"

void AdcTask(void);
uint16_t* AdcReturnAdcBuffer(void);
void AdcUserInit(void);

/** ADC peripheral state enum */
typedef enum e_ADC_State
{
  ADC_FREE = 0U,
  ADC_CONVERTING,
  ADC_ERROR
} te_ADC_State;


#endif /* INC_ADC_H_ */
