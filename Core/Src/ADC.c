/*
 * ADC.c
 *
 *  Created on: Jun 21, 2025
 *      Author: violet
 */

#include "ADC.h"
#include "TIMER.h"
#include "DMA.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

static uint16_t adcBuffer1[2] = {0};
static uint16_t adcBuffer2[2] = {0};
static uint16_t* adcFillingBuffer = adcBuffer1;
static uint16_t* adcProcessingBuffer = adcBuffer2;
static te_ADC_State adcState = ADC_FREE;

static void AdcStartConversion(void);
static bool AdcIsDoneConversion(void);

/*==============================================================================================*/
/**
 * \fn uint16_t* AdcStartConversion(void)
 * Returns processing buffer which most up to date valid ADC readings
 */
uint16_t* AdcReturnAdcBuffer(void)
{
  return adcProcessingBuffer;
}

/*==============================================================================================*/
/**
 * \fn static void AdcStartConversion(void)
 * Setup DMA and start ADC conversions
 */
static void AdcStartConversion(void)
{
  /* Clear conversion-related flags */
  /* Do no readback register value as it cleared by writting 1 */
  hadc1.Instance->ISR = (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
  
  /* Enable DMA requests */
  hadc1.Instance->CFGR |= ADC_CFGR_DMAEN;

  /* arm DMA to store ADC1 results first, then ADC2 */
  DmaAdcArm(adcFillingBuffer, 2);

  /* Start conversion without setting rs (read_set) bits */
  hadc1.Instance->CR = (hadc1.Instance->CR & ~ADC_CR_BITS_PROPERTY_RS) | ADC_CR_ADSTART;
}

/*==============================================================================================*/
/**
 * \fn static bool AdcIsFree(void)
 * Returns whether ADC is busy with a conversion
 */
static bool AdcIsDoneConversion(void)
{
  /* ADSTART bit in ADC_CR register is cleared when conversions are done */
  bool isConversionDone = ((hadc1.Instance->CR & ADC_ISR_EOS) == 0);
  return isConversionDone;
}

/*==============================================================================================*/
/**
 * \fn void AdcTask(void)
 * Update ADC at SYSTEM_UPDATE_RATE
 */
void AdcTask(void)
{
  static uint32_t lastAdcUpdate = 0;
  uint16_t* temp = NULL;

  /* If ADC is busy with conversion, exit */
  if(adcState != ADC_FREE)
  {
    if(AdcIsDoneConversion())
    {
      /* Clear conversion-related flags */
      /* Do no readback register value as it cleared by writting 1 */
      hadc1.Instance->ISR = (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

      /* ADC is done conversion, swap buffers to restart */
      temp = adcProcessingBuffer;
      
      /* Interrupt critical Start */
      
      adcProcessingBuffer = adcFillingBuffer;
      /* If an interrupt happens between these two points */
      /* that's acceptable as both ptrs have the buffer   */
      /* with the newest adc conversions                  */
      adcFillingBuffer = temp;
      
      /* Interrupt critical End */
     
      adcState = ADC_FREE;
    }
    else
    {
      /* ADC is still busy, return */
      return;
    }
  }

  /* If system update period has passed, restart conversion */
  if(TimDelayExpired(lastAdcUpdate, SYSTEM_UPDATE_PERIOD))
  {
    lastAdcUpdate += SYSTEM_UPDATE_PERIOD;
    AdcStartConversion();
    adcState = ADC_CONVERTING;
  }
}

