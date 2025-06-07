/*
 * Gatekeeper.c
 *
 *  Created on: Sep 7, 2024
 *      Author: khadeejaabbas
 *      Role: Determines what to do if we recieve a message from the MBMS that address this specific contactor
 *
 */
#include <switchEnums.h>
#include <SendingCANMessage.h>
#include <makingCANMessage.h>
#include <MessageReader.h>
#include <../Inc/Gatekeeper.h>
#include <changeSwitch.h>
#include <stdbool.h>
#include <stm32l4xx_hal.h>
#include <main.h>
#include <stdint.h>

/*
 * Gloabl Variables
 */

//extern ADC_HandleTypeDef hadc1; **************************************************************************************************************************************************************UNCOMMENT
extern uint16_t rawValues[2];

// initializing a struct for the precharger and contactor

extern SwitchInfo_t contactor;
extern SwitchInfo_t precharger;


/*
 *  The CAN message from the Master BMS will be passed into this function and this will delegate how to open/close
 *		message: CAN message
 */

uint64_t func(float x)
{
	return (2*x*x + 1);
}

int64_t removeNoise()
{
	int64_t initial_adcCount_y1 = rawValues[0];

	// our ADC reads the analog value and converts it into a number by multiplying it by 4096 (number of total number values). Now, we want to get the actual current so we divide by 4095 (the total number of combinations (since we start at 0)) and then we multiply by 3.3V since we are measuring 3.3. ADC resolution.
	int64_t voltage_1  = ((initial_adcCount_y1 * 3.3) / 4095);

	// minus the 2 offset
	int64_t voltaget_with_offset_1 = voltage_1 - 2;

	// we have to convert adc voltage to shunt resistor (done through (Vadc)* 0.0025)
	int64_t shunt_resistor_voltage_1 = (voltaget_with_offset_1 * 0.0025);

	// get the current
	int64_t current_1 = shunt_resistor_voltage_1 / (precharger.resistance);


	HAL_Delay(1); // the clock frequency is 80 MHz. So it waits 1 ticks (1 millisecond)

	// we're going to sample 2 points very close to each other to bypass noise and make it nominal

	int64_t initial_adcCount_y2 = rawValues[0];




	// the 3.3 and 4096 are for the ADC resolution
	int64_t voltage_2  = ((initial_adcCount_y2 * 3.3) / 4095);

	// minus the 2 offset
	int64_t voltaget_with_offset_2 = voltage_2 - 2;

	// we have to convert adc voltage to shunt resistor (done through (Vadc)* 0.0025)
	int64_t shunt_resistor_voltage_2 = (voltaget_with_offset_2 * 0.0025);

	// get the current
	int64_t current_2 = shunt_resistor_voltage_2 / (precharger.resistance);


	// now we take the average of the two
	int64_t avg_current = (current_2 + current_1)/2;

	return avg_current;
}

//void checkState(){
//	// check our contactor
//	contactor.Switch_State = HAL_GPIO_ReadPin(contactor.GPIO_Port_Sense, contactor.GPIO_Pin_Sense);
//	if (contactor.Switch_State == CLOSED){
//		contactor.GPIO_State = GPIO_PIN_SET; // set the pin
//	}else{
//		contactor.GPIO_State = GPIO_PIN_RESET; // ensure it's reset
//	}
//	// if we're not common, check the same thing for our contactor
//	if (contactor.WhichContactor != COMMON){
//		precharger.Switch_State = HAL_GPIO_ReadPin(precharger.GPIO_Port_Sense, precharger.GPIO_Pin_Sense);
//
//		if (precharger.Switch_State == CLOSED){
//			precharger.GPIO_State = GPIO_PIN_SET; // set the pin
//		}else{
//			precharger.GPIO_State = GPIO_PIN_RESET; // ensure it's reset
//		}
//	}
//
//
//}

void Gatekeeper(CAN_Message *message)
{
//	// WHEN YOU'RE INTIALIZING CHECK THE STATE!!!!!!!!!
//	checkState();

	// initialize the variables
	SwitchState WantedState;
	uint8_t TxData[8];

	// get the bit that associates with our contactor (i.e the common contactor will be bit 0)
	uint8_t bitOfInterest = contactor.WhichContactor;
	// create a mask for that specifc bit
	uint8_t mask = 1 << bitOfInterest;

	// takes our payload and ANDs it with the mask so we only get the value of our bit of interest, then we shift that bit all the way to the right. If that value is equal to our wanted state (0 for open, 1 for closed), we good
	uint8_t what_MBMS_wants_us_to_be = (message->data[0] & mask) >> bitOfInterest;
	// the MBMS is going to give a message with the wanted states of all the contactors and we got to check if our contactor is in that wanted state, if it's not we try changing it to that state
	if (what_MBMS_wants_us_to_be == contactor.Switch_State){
	    // that means our current state is what the MBMS wants and we don't need to change it
		return;
	}
	// if we need to change our contactor state
	else{
		WantedState = (SwitchState)what_MBMS_wants_us_to_be;
	}

	// switch case between opening or closing
	switch (WantedState){
		case OPEN:
			// our switch is not open yet!
			// call the function to open the switch. if it opens it will return true. If it's open, it will return 1, else 0.
// let's first test if we can go from closed state to open state. So to do this, we expect the state the change from current to no current. To do that let's first test if we have a current, no light turns on. If a light turns on, uh-oh. If not, then lets test if the light turns on if we don't have a current flowing through
			changeSwitch(&contactor, contactor.Switch_State, OPEN, contactor.Delay);

			// making the message we are going to send
			// Convert state to uint8_t for CAN message
//			state_byte |= (ContactorOpenState ? 1 : 0);// If true, state_byte = 1; if false, state_byte = 0

			// end case
			break;

		case CLOSED:
			// initialize our sending variables
			bool PrechargerState = false;
			bool ContactorClosedState = false;

			if (contactor.WhichContactor == (SwitchState)COMMON) {	// the common doesn't have precharging so we're bypassing it

						ContactorClosedState = changeSwitch(&contactor, contactor.Switch_State, CLOSED, contactor.Delay);
						break; // break out of this state
				}

			// NOT COMMON:

			// check if the common is closed
			// create a mask for that specifc bit
			uint8_t mask = 1 << COMMON;

			// takes our payload and ANDs it with the mask so we only get the value of our bit of interest, then we shift that bit all the way to the right. If that value is equal to our wanted state (0 for open, 1 for closed), we good
			uint8_t checkCommonStatus = (message->data[0] & mask) >> COMMON;

			if (checkCommonStatus != CLOSED){
				contactor.switchError = true;
				break; // break out of this state

			}

			// we need to first tell the Precharge_Sense_ON pin to close
//			HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_SET); **************************************************************************************************************************************************************UNCOMMENT

			// to safely close the switch, we would need to first close the precharger to voltage spike.
//			PrechargerState = changeSwitch(&precharger, precharger.Switch_State, CLOSED, precharger.Delay); **************************************************************************************************************************************************************UNCOMMENT
			// this will return if it's closed or not

			if (PrechargerState){	// precharger is closed
				/*  we need to precharge before trying to close																		*/
				/*  we first take the ADC value from the pin that reads the PRCH_CURRENT_ADC										*/
				/*  from that we need to convert this ADC count into voltage then divide it by (2*Rf) to get the current!			*/
				/*  get the current: 																								*/
				int64_t avg_initial_current_1 = removeNoise();
				HAL_Delay(TICKS_BETWEEN_SAMPLING_POINTS); // the clock frequency is 80 MHz. So it waits 'TIME_BETWEEN_SAMPLING_POINTS' number of ticks or milliseconds

				// now we get our second sampling point
				int64_t avg_initial_current_2 = removeNoise();


				// now we gotta take the differential of the current
				// so we're gonna have time on the x-axis and the y-axis is the current
				float h = 0.000000125 * TICKS_BETWEEN_SAMPLING_POINTS; // we want h to be equal to 1 tick. 1 tick is 0.000125 milliseconds. So 0.000000125 seconds * how every many ticks we do
				float initial_deriv;

				initial_deriv = (func(avg_initial_current_2) - func(avg_initial_current_1))/h;

				// ok so we got the inital values yay
				// now we gotta wait a sec
//				HAL_Delay(8000*1000); // the clock frequency is 80 MHz. So it does 1 tick every 0.000125 milliseconds. So 8000 ticks is 1 milliseconds. We want to wait 1 seconds so we do (8000 * 1000) to get 1 second (1000 milliseconds = 1 second)
				HAL_Delay(1000); // the clock frequency is 80 MHz. So it does 1 tick every 0.000125 milliseconds. So 8000 ticks is 1 milliseconds. We want to wait 1 seconds so we do (8000 * 1000) to get 1 second (1000 milliseconds = 1 second)

				int64_t avg_latest_current_1 = removeNoise();

				HAL_Delay(TICKS_BETWEEN_SAMPLING_POINTS); // the clock frequency is 80 MHz. So it waits 100 ticks

				int64_t avg_latest_current_2 = removeNoise();

				float latest_deriv;
				latest_deriv = (func(avg_latest_current_2) - func(avg_latest_current_1))/h;

				// check if the derivative of the latest - the initial is above a certain threshold value
				if ((latest_deriv) <= precharger.derivative_threshold){ // if we pass the threshold value
					// now we gotta check if we pass the threshold for the value (is it close to 0?)
					if ((avg_latest_current <= precharger.threshold)){
						// yay, it's precharged, let's try closing the contactor:
						ContactorClosedState = changeSwitch(&contactor, contactor.Switch_State, CLOSED, contactor.Delay);
					}
				}

				PrechargerState = changeSwitch(&precharger, precharger.Switch_State, OPEN, precharger.Delay); // open the precharger no matter what
			}

			// open the Precharge Sense On pin
			HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);


			//000 000
			//      ^ for the precharger if it's open or closed
			//     ^  if the precharger is closing
			//    ^   if the precharger couldn't perform the wanted task
			//  ^     if the contactor is open or closed
			// ^      if the contactor is closing
			//^       if the contactor couldn't perform the wanted task
			// everything working state: 001 000
			// precharger broken but contactor closed: 001 101
			// precharger closed but contactor not: 100 001
			// precharger and contactor didn't close: 100 100

			// end case
			break;
		case CLOSING:	// should never happen, why would we want to be closing? We only want to close
			break;
		case CONTACTOR_ERROR: // we should never want an error state
			break;
		case BPS_ERROR:
			break;
	}
}
