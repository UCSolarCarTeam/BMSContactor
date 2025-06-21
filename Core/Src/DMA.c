/*
 * DMA.c
 *
 *  Created on: Jun 21, 2025
 *      Author: violet
 */

#include "DMA.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

static void DmaArm(DMA_HandleTypeDef *hdma, uint32_t mData, uint32_t pData, uint16_t Size);

/*==============================================================================================*/
/**
 * \fn staticn void DmaArm(void)
 * Generic DMA initialization
 * \param    hdma   Dma handler
 * \param    mData  pointer to memory address (or dest address in memtomem mode)
 * \param    pData  pointer to peripheral address (or source address in memtomem mode)
 * \param    Size   Number of input bytes
 */
static void DmaArm(DMA_HandleTypeDef *hdma, uint32_t mData, uint32_t pData, uint16_t Size)
{
  /** -# First disable peripheral */
  __HAL_DMA_DISABLE(hdma);

  /** -# Clear all flags */
  __HAL_DMA_CLEAR_FLAG(hdma, (DMA_FLAG_GL1 << (hdma->ChannelIndex & 0x1CU)));

  /** -# Write size to CNDTR */
  hdma->Instance->CNDTR = Size;

  /** -# Write peripheral address to CPAR */
  hdma->Instance->CPAR = pData;

  /** -# Write memory address to CMAR */
  hdma->Instance->CMAR = mData;

  /** -# Only enable transfer error interrupt, no need for transfer complete */
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);
} /* End of DmaArm */

/*==============================================================================================*/
/**
 * \fn uint8_t DmaAdc1Arm(uint16_t *pData, uint16_t Size)
 * Initializes and starts ADC1 DMA.
 *
 * \param    pData  pointer to ADC Rx buffer
 * \param    Size   Number of input bytes
 */
void DmaAdcArm(uint16_t *pData, uint16_t Size)
{
  /** -# Initialize DMA */
  DmaArm(&hdma_adc1, (uint32_t)pData, (uint32_t)&(hadc1.Instance->DR), Size);

  __HAL_DMA_ENABLE(&hdma_adc1);
  
  return;
} /* End of DmaAdc1Arm */

