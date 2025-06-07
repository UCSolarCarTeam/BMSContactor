/*
 * SendingCANMessage.c
 *
 *  Created on: Jan 15, 2025
 *      Author: khadeejaabbas
 */

#include "changeSwitch.h"
#include "../Inc/Gatekeeper.h"
#include "stm32l4xx_hal.h"
#include <main.h>
#include <stdint.h>

//extern SwitchInfo_t precharger;
extern SwitchInfo_t contactor;

//void SendingCANMessage(uint8_t TxData[8], uint8_t DLC_num){
// DEBUGGING BELOW:::: UNCOMMENT THE ABOVE LINE WHERE THE RETURN IS VOID, WE'RE JUST DEBUGGING RN, U CAN DELETE THE 'CAN_TxHeaderTypeDef SendingCANMessage(uint8_t TxData[8], uint8_t DLC_num){'
CAN_TxHeaderTypeDef SendingCANMessage(uint32_t extendedID, uint8_t TxData[8], uint8_t DLC_num){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	// making the message we are going to send
	 TxHeader.DLC = DLC_num;
	 TxHeader.ExtId = extendedID;
	 TxHeader.IDE = CAN_ID_EXT;
	 TxHeader.RTR = CAN_RTR_DATA;
	 TxHeader.TransmitGlobalTime = DISABLE;

//	 // send the message! Otherwise, handle the error
	 if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
		 Error_Handler();
	 }
	 // DEBUGGING BELOW:::: UNCOMMENT THE ABOVE FUNCTION FOR SENDING THE MESSAGE, WE'RE JUST DEBUGGING RN, U CAN DELETE THE 'return TxHeader;'
	 return TxHeader;

}
