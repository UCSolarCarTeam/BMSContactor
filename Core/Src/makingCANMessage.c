/*
 * makingCANMessage.c
 *
 *  Created on: Jan 18, 2025
 *      Author: khadeejaabbas
 */
#include "switchEnums.h"
#include "SendingCANMessage.h"
#include "MessageReader.h"
#include "../Inc/Gatekeeper.h"
#include "changeSwitch.h"
#include "stm32l4xx_hal.h"
#include <main.h>
#include <stdint.h>


extern SwitchInfo_t precharger;
extern SwitchInfo_t contactor;
extern volatile uint16_t rawValues[2];
//extern uint16_t raw_current;
//extern uint16_t raw_voltage;

uint32_t makingCANMessage()
{
	// defining varaiables
	uint8_t prechargerState;
	uint8_t contactorState;
	uint8_t prechargerError;
	uint8_t contactorError;
	uint8_t bpsError;

	prechargerState = precharger.Switch_State; 	// either 0 = OPEN, 1 = CLOSED, 2 = CLOSING, or 3 = CONTACTOR ERROR
	contactorState = contactor.Switch_State;  	// ditto
//	contactorState = 1;  	// DEBUGGING LINEEEEEE. REMOVE THIS LINE AND UNCOMMENT THE ABOVE LINE WHEN DONE DEBUGGING

	prechargerError = precharger.switchError;	// if the bool is set to 1, there was an error
	contactorError = contactor.switchError;		// ditto

	bpsError = contactor.BPSError; // if the bool is set to 1, there was a BPS error, SERIOUS

	// now we gotta convert them to bytes
	uint32_t state_status = 0;
	if (prechargerState == 0 || prechargerState == 1)
	{
		state_status = (state_status & 0xFFFFFFFE) | ((prechargerState & 0x1) << 0);  	  	// Bit 0: PrechargerState is open or closed
	}
	else if (prechargerState == 2)
	{
		state_status = (state_status & 0xFFFFFFFD) | (0x1 << 1);  	  	// Bit 1: PrechargerState is closing or not
	}

	state_status = (state_status & 0xFFFFFFFB) | ((prechargerError & 0x1) << 2); 			// Bit 2: If the precharger failed to get to the wanted state or not

	if (contactorState == 0 || contactorState == 1)
	{
		state_status = (state_status & 0xFFFFFFF7) | ((contactorState & 0x1) << 3);  	  	// Bit 3: contactorState is open or closed
	}
	else if (contactorState == 2)
	{
		state_status  = (state_status & 0xFFFFFFEF) | (0x1 << 4);  	  	// Bit 4: contactorState is closing or not
	}

	state_status = (state_status & 0xFFFFFFDF) | ((contactorError & 0x1) << 5); 			// Bit 5: If the contactor failed to get to the wanted state or not

	// adding math to convert line current before sending it out onto the CAN Line
	// need to offset the ADC since to account for negative values (-2 to 2), it was made into 0-4. So now we gotta convert back to -2 to 2. So we minus 2
	rawValues[1] = rawValues[1] - 2;
	// now we have to times it by the amps to ADC voltage ratio
	rawValues[1] = rawValues[1] * contactor.lineCurrentAmpsPerADCVoltage;

	state_status = (state_status & 0xFFFC003F) | ((rawValues[1] & 0xFFF) << 6);				// Bits 6 - 17: the line current

	state_status = (state_status & 0xC003FFFF) | ((rawValues[0] & 0xFFF) << 18);				// Bits 18 - 30: the charge voltage

	state_status = (state_status & 0xBFFFFFFF) | ((bpsError & 0x1) << 30); 					// Bit 31: If the contactor failed to open, it's a serious BPS Error


	return state_status;
}
