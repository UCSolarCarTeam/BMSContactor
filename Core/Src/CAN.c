/*
 * CAN.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Khadeeja Abbas, violet
 */

#include "CAN.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

extern Precharger_t precharger;
extern Contactor_t contactor;
//extern uint16_t raw_current;
//extern uint16_t raw_voltage;

extern BoardIds boardIds;

static CAN_Message receivedMessageBuffer;
bool contactorCommandClose = false;

#if UART_DEBUG
  //uint8_t msg[] = "Received\r\n";
  //uint8_t msg2[] = "Message Not Received\r\n";
#endif

bool Check_CAN_Messages(void)
{
	CAN_RxHeaderTypeDef rx_header;
	uint32_t fifo_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);

	if (fifo_level == 0){
		return false; // no available messages
	}

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, receivedMessageBuffer.data) != HAL_OK)
	{
		return false; // message read error
	}

#if UART_DEBUG
	HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
#endif

#if FAKE_CAN
    received_message.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
    received_message.dlc = rx_header.DLC;
    received_message.is_extended = (rx_header.IDE == CAN_ID_EXT);
    received_message.is_rtr = (rx_header.RTR == CAN_RTR_REMOTE);
    received_message.data[0] = 0b00001001;
    received_message.data[1] = 0b00000000;
    received_message.data[2] = 0b00000000;
    received_message.data[3] = 0b00000000;
#endif

	receivedMessageBuffer.id = rx_header.ExtId;
	receivedMessageBuffer.dlc = rx_header.DLC;

	// Process message
	switch(receivedMessageBuffer.id)
	{
		case CONTACTOR_COMMAND_ID:
			if(receivedMessageBuffer.data[0] & (1 << boardIds.type))
			{
				contactorCommandClose = true;
			}
			else
			{
				contactorCommandClose = false;
			}
			break;
		default:
		break;
	}

	return false;
}

uint32_t makingCANMessage()
{
	// defining varaiables
	uint8_t prechargerState;
	uint8_t contactorState;
	uint8_t prechargerError;
	uint8_t contactorError;
	uint8_t bpsError;
	uint16_t* adcBuffer = AdcReturnAdcBuffer();
	uint16_t lineCurrent = 0;

	prechargerState = precharger.Switch_State; 	// either 0 = OPEN, 1 = CLOSED, 2 = CLOSING, or 3 = SWITCH_ERROR
	contactorState = contactor.Switch_State;  	// ditto
	//contactorState = 1;  	// DEBUGGING LINEEEEEE. REMOVE THIS LINE AND UNCOMMENT THE ABOVE LINE WHEN DONE DEBUGGING

	prechargerError = precharger.switchError;	// if the bool is set to 1, there was an error
	contactorError = contactor.switchError;		// ditto

	bpsError = contactor.BPSError; // if the bool is set to 1, there was a BPS error, SERIOUS

	// now we gotta convert them to bytes
	uint32_t state_status = 0x000000; /* set output to 0 */
	if (prechargerState == 0 || prechargerState == 1)
	{
		state_status |= ((prechargerState & 0x1) << 0);  	  	// Bit 0: PrechargerState is open or closed
	}
	else if (prechargerState == 2)
	{
		state_status |= (0x1 << 1);  	  	// Bit 1: PrechargerState is closing or not
	}

	state_status |= ((prechargerError & 0x1) << 2); 			// Bit 2: If the precharger failed to get to the wanted state or not

	if (contactorState == 0 || contactorState == 1)
	{
		state_status |= ((contactorState & 0x1) << 3);  	  	// Bit 3: contactorState is open or closed
	}
	else if (contactorState == 2)
	{
		state_status |= (0x1 << 4);  	  	// Bit 4: contactorState is closing or not
	}

	state_status |= ((contactorError & 0x1) << 5); 			// Bit 5: If the contactor failed to get to the wanted state or not

	// adding math to convert line current before sending it out onto the CAN Line
	// now we have to times it by the amps to ADC voltage ratio
	lineCurrent = adcBuffer[1] * contactor.lineCurrentAmpsPerADCVoltage;

	state_status |= ((lineCurrent & 0xFFF) << 6);				// Bits 6 - 17: the line current

	state_status |= ((adcBuffer[0] & 0xFFF) << 18);				// Bits 18 - 30: the charge voltage

	state_status |= ((bpsError & 0x1) << 30); 					// Bit 31: If the contactor failed to open, it's a serious BPS Error

	return state_status;
}

// CAN_TxHeaderTypeDef return type is for debugging purposes only, may be turned to void
CAN_TxHeaderTypeDef SendingCANMessage(uint32_t extendedID, uint8_t* data, uint8_t DLC_num){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox; /* returns CAN mailbox used, unused */

	// making the message we are going to send
	TxHeader.DLC = DLC_num;
	TxHeader.ExtId = extendedID;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	// send the message! Otherwise, handle the error
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK){
		/* Added blink of bad LED, not error handler as it would stop functionaluty */
	}

	// return for debugging
	return TxHeader;

}

