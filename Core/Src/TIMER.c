/*
 * TIMER.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Khadeeja Abbas, violet
 */

#include "TIMER.h"
#include "CAN.h"
#include "CONTACTOR.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern volatile SwitchInfo_t contactor;
extern volatile SwitchInfo_t precharger;
extern BoardIds boardIds;

/*==============================================================================================*/
/**
  * \fn uint32_t TimGetTime(void)
  * Returns current system time
  */
uint32_t TimGetTime(void)
{
  uint32_t currentTimerCounts = htim2.Instance->CNT; /* volatile value */

  return currentTimerCounts;
} /* End of TimGetTimeElapsedSince */

/*==============================================================================================*/
/**
  * \fn uint32_t TimGetTimeElapsedSince(void)
  * Returns difference between counter and argument in terms of system timer ticks, set
  * per TIM2, typically every 1us.
  */
uint32_t TimGetTimeElapsedSince(uint32_t time)
{
  uint32_t currentTimerCounts = htim2.Instance->CNT; /* volatile value */

  return (currentTimerCounts - time);
} /* End of TimGetTimeElapsedSince */

/*==============================================================================================*/
/**
  * \fn uint32_t TimDelayExpired(void)
  * Returns true if a delay has expired, flase otherwise. Start and delay must both be
  * in terms of TIM2 ticks (typically 1us)
  */
bool TimDelayExpired(uint32_t start, uint32_t delay)
{
  return (bool)(TimGetTimeElapsedSince(start) >= delay);
} /* End of TimDelayExpired */

/*==============================================================================================*/
/**
  * \fn uint32_t TimDelayExpired(void)
  * CAN Tx Task. Sends heartbeat and can messages
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t heartbeatCounter = 0;
  static uint16_t heartbeat = 0;
  uint8_t heartData[8];
  uint8_t TxData[8];
  uint32_t state_status;

  if (htim->Instance == TIM16)
  {
    checkState();
	  
    // Update heartbeat at 10Hz (Tim16 rate == 100Hz)
    if(heartbeatCounter == 10)
    {
      heartbeatCounter = 0;
      heartbeat++;
      
      // Store the 16-bit heartbeat into two bytes
      heartData[0] = heartbeat & 0xFF;         // Low byte (bits 0-7)
      heartData[1] = (heartbeat >> 8) & 0xFF;  // High byte (bits 8-15)
      
      // send heartbeat
      SendingCANMessage(HEARTBEAT_ID_BASE + boardIds.canIdOffset, heartData, HEARTBEAT_DLC);
    }
    heartbeatCounter++;

    // Send contactor and precharger state
    state_status = makingCANMessage();
    TxData[0] = state_status & 0xFF;
    TxData[1] = (state_status >> 8) & 0xFF;
    TxData[2] = (state_status >> 16) & 0xFF;
    TxData[3] = (state_status >> 24) & 0xFF;

    SendingCANMessage(STATUS_ID_BASE + boardIds.canIdOffset, TxData, STATUS_DLC);
  }
  else if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
}
