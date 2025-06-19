/*
 * Switches.c
 *
 *  Created on: Sep 7, 2024
 *      Author: Khadeeja Abbas, and Dominic Choi
 *      Role: Closes or opens whatever switch (contactor or the precharger) is passed to it
 */
#include <changeSwitch.h>
#include "switchEnums.h"
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <main.h>
#include <stdint.h>
#include "SendingCANMessage.h"
#include "makingCANMessage.h"

extern volatile SwitchInfo_t precharger;
extern volatile SwitchInfo_t contactor;
extern uint8_t TxData[8]; // for sending CAN messages

/*
 * This function is called from the Gatekeeper
 *
 * 	tryChangingSwitch variables:
 * 		switch: the switch we are working with. Either the contactor or the precharger
 * 		current_state: the current state of the switch
 * 		wanted_state: the state we want to change the switch to
 * 		delayTime: The time we want to wait before checking if the switch is in the state we want (if we want it closed, we will want to wait a little bit before checking if it's closed to allow for a current to flow through)
 * 		numOfTrials: how many times did we try closing the switch? If we keep failing, we will return an error
 * 		status: did we achieve our wanted state (true) or not (false)
 *
 * 		NOTE: Still need to work on checking if current is actually flowing through the switch or not
 */

bool changeSwitch(SwitchInfo_t* switch_to_change, SwitchState current_state, SwitchState wanted_state, uint32_t delayTime){
	if(wanted_state == CLOSED){
		for(int attempts = 0; attempts < MAX_NUM_OF_RETRIES; attempts++){
			// try to change the switch to the wanted state
			SwitchState switch_status = setSwitch(switch_to_change, CLOSED);
			if(switch_status == CLOSED){
//				setDebugLED(switch_to_change); // check if the switch is closed or not
				return true;

			} else if(switch_status == CONTACTOR_ERROR){ // try to change the switch to the wanted state
				switch_to_change->switchError = true;
				setSwitch(switch_to_change, CONTACTOR_ERROR); // if we get an error, open the switch
				return false;
			}
			HAL_Delay(delayTime); // wait for a bit before checking if we achieved our goal

		}
		// this case is reached in case we couldn't close the contactor
		switch_to_change->switchError = true;
		setSwitch(switch_to_change, CONTACTOR_ERROR); // if we get an error, open the switch
		return false;

	} else if (wanted_state == OPEN){
		switch(switch_to_change->isContactor){
				case true: // Contactor, and we want to open. try inf times, check if gpio input us open
					SwitchState switch_status = setSwitch(switch_to_change, OPEN); // try to open the switch
					while(switch_status != OPEN){ // while the switch is not open, keep trying to open it
						switch_status = setSwitch(switch_to_change, OPEN); // try to open the switch

//						HAL_Delay(delayTime); // wait for a bit before checking if we achieved our goal
					}
					switch_to_change->BPSError = false;
					return true;

				case false:
					// Precharger, and we want to open. try inf times, check if adc input > 0 (open)
					switch_status = setSwitch(switch_to_change, OPEN); // try to open the switch
					return true;
		}
	}

	// this should never be reached. This return is here just in case
	return false;
}

/*
 * 	This function will try physically changing the switch of interest into the wanted state
 * 		switch_to_change = switch we want to change the state of
 * 		WantedState = the state we want the switch to be in (open, closed, closing, error)
 *
 * 	Will return	GPIO_PIN_SET or GPIO_PIN_RESET
 */
SwitchState setSwitch(SwitchInfo_t* switch_to_change, SwitchState wanted_state)
{
	switch (wanted_state){
			case OPEN:
				HAL_GPIO_WritePin(switch_to_change->GPIO_Port, switch_to_change->GPIO_Pin, GPIO_PIN_RESET);
				HAL_Delay(switch_to_change->Delay); // wait for a bit before checking if we achieved our goal

				if(switch_to_change->isContactor == true){ // OPEN-ing Contactor
					if(HAL_GPIO_ReadPin(switch_to_change->GPIO_Port_Sense, switch_to_change->GPIO_Pin_Sense) == 0){
						switch_to_change->Switch_State = OPEN; // set the switch state to OPEN
						return OPEN;

					} else {
						switch_to_change->BPSError = true;
						switch_to_change->Switch_State = CLOSED; // still closed
						return CLOSED;
					}

				} else { // OPEN-ing Precharger, no need for checks
					switch_to_change->Switch_State = OPEN; // set the switch state to OPEN
					return OPEN; // if the switch is open, return open
				}
				break;

			case CLOSED:
				HAL_GPIO_WritePin(switch_to_change->GPIO_Port, switch_to_change->GPIO_Pin, GPIO_PIN_SET);

				if(switch_to_change->isContactor == true){ // CLOSE-ing Contactor, check if GPIO is SET (closed)
					HAL_Delay(switch_to_change->Delay); // wait for a bit before checking if we achieved our goal
					if(HAL_GPIO_ReadPin(switch_to_change->GPIO_Port_Sense, switch_to_change->GPIO_Pin_Sense) == 1){
						switch_to_change->Switch_State = CLOSED; // set the switch state to OPEN
						return CLOSED; // if the switch is closed, return closed
					} else {
						switch_to_change->Switch_State = CLOSING; // set the switch state to OPEN
						return CLOSING; // if the switch is not yet closed, return closing
					}

				} else { // CLOSE-ing Precharger, check if ADC is > 0 (current flowing through)
//					if(HAL_GPIO_ReadPin(DIAG_N_Input_GPIO_Port, DIAG_N_Input_Pin) != 0){ // we can read current now
				if(rawValues[1] >= PRECHARGE_COMPLETE_THRESHOLD_ADC_COUNT)
				{ // we can read current now

						switch_to_change->Switch_State = CLOSED; // set the switch state to OPEN
						return CLOSED; // if the switch is closed, return closed
					} else {
						switch_to_change->Switch_State = CLOSING; // set the switch state to OPEN
						return CLOSING; // if the switch is not yet closed, return closing
					}
				}

				break;

			case CONTACTOR_ERROR: // If we get an error, open the switch
				HAL_GPIO_WritePin(switch_to_change->GPIO_Port, switch_to_change->GPIO_Pin, GPIO_PIN_RESET);

				switch_to_change->Switch_State = OPEN; // set the switch state to OPEN
				return OPEN;
				break;

			default:
				// ???
				break;

	}

	return CONTACTOR_ERROR; // return open if we fail all the switch statements above. This case should never happen so it's an error!

}

/*
 * Check if the contactor is in the wanted state:
 * 		We need to if:
 * 			1) the state read of the pin is the same as the state we want
 * 			2) If current is flowing through or not (checking if HAL_GPIO_ReadPin already does this)
 *
 *		variables:
 *			contactor: contactor of interest
 *
 */
//void setDebugLED(SwitchInfo_t* switch_to_change){
//	// Check our gpio pin is recieving has current or not
//    HAL_GPIO_WritePin(
//		LED_Green_GPIO_Port,
//		LED_Green_Pin,
//		HAL_GPIO_ReadPin(switch_to_change->GPIO_Port_Sense, switch_to_change->GPIO_Pin_Sense)
//	); // Toggle Green LED based on the state of the contactor
//}
// We don't have an LED on this board
