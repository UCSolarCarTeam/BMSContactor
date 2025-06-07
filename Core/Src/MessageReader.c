/*
 * MessageReader.cpp
 *
 *  Created on: Dec 27, 2024
 *      Author: khadeejaabbas
 */


#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <main.h>
#include "MessageReader.h"
#include <string.h>
#include <switchEnums.h>

// ok so we want it so that we check our fifofill level and sees if it's empty and if it's not, we read the message
// then we check if that message corresponds to our board
// creating a struct and initalizing
extern SwitchInfo_t precharger;
extern SwitchInfo_t contactor;

// ok now we get our message which is our ID
// we check our queue and see if it's empty
bool Check_CAN_Messages(CAN_Message *message){
	// ok so you get the message and now you need to filter through the bits and if the bit you're contactor is on is 1, then u process this
	uint32_t fifo_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
	if (fifo_level == 0){
		return false; // no available messages
	}

	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8]; // CAN data max size is 8 bytes

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
	{
		return false; // didn't recieve message
	}

	// check bits
	uint8_t bitOfInterest = contactor.WhichContactor;
	// ok now our payload is stored in rx_data (i think it should be now)
	bool isTheMessageForUs = rx_data[bitOfInterest] & 1;
	if (isTheMessageForUs){
		// we want to get that message, which already has been compared against the filter (only messages that pass the filter are added to the FIFO)
		// I want to return this message to say that we should start the other tasks
		// Populate the custom message structure
		message->id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
		message->dlc = rx_header.DLC;
		message->is_extended = (rx_header.IDE == CAN_ID_EXT);
		message->is_rtr = (rx_header.RTR == CAN_RTR_REMOTE);
		memcpy(message->data, rx_data, rx_header.DLC);
		return true; // we did it
	}

	return false;
}
