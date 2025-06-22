/*
 * CONTACTOR..c
 *
 *  Created on: Jun 21, 2025
 *      Author: Khadeeja Abbas, Dominic Choi, violet
 */

#include "CONTACTOR.h"
#include "CAN.h"
#include "ADC.h"
#include "TIMER.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern volatile SwitchInfo_t contactor;
extern volatile SwitchInfo_t precharger;
extern BoardIds boardIds;
extern bool contactorCommandClose;

static Contactor_State contactorState = ALL_OPEN;
static uint32_t stateDelayStart = 0;
static uint32_t errorDelayStart = 0;
static uint32_t pointStartOne = 0;
static uint32_t pointStartTwo = 0;
static int64_t avg_current_1;
static int64_t avg_current_2;
static int64_t avg_current_3;
static int64_t avg_current_4;
static bool checkStateStatus1;
static bool checkStateStatus2;
static bool checkStateStatus3;
static bool checkStateStatus4;
float initial_deriv;
float latest_deriv;
float h = 0.000000125 * TICKS_BETWEEN_SAMPLING_POINTS; // we want h to be equal to 1 tick. 1 tick is 0.000125 milliseconds. So 0.000000125 seconds * how every many ticks we do


void ContactorTask(void)
{
	checkState();

	switch (contactorState)
	{
		case ALL_OPEN:
			if(contactorCommandClose)
			{
				if(boardIds.type != COMMON)
				{
					enterPrecharging1State();
				}
				else
				{
					/* common does not precharge */
					enterClosingContactorState();
				}
			}
			else{
				// be sure you're open
				enterAllOpenState();
				// check if you're open
				checkState();

				// handling the case where we want to open the contactor but it's not opening (BPS Error - very serious)
				if (contactor.GPIO_State != GPIO_PIN_RESET){
					enterAllOpenState();
					contactor.BPSError = true;
				} else {
					contactor.BPSError = false;
				}
			}
		break;
		case PRECHARGING1:
			if(!contactorCommandClose)
			{
				enterAllOpenState();
			}
			else
			{
				if(TimDelayExpired(stateDelayStart, 10000))
				{
					enterPrecharging2State();
				}
			}
		break;
		case PRECHARGING2:
			if(!contactorCommandClose)
			{
				enterAllOpenState();
			}
			else
			{
				// this is chatgbt's cleaner version of the code I hashed out below. Might not work cuz chatgbt is stupid sometimes. Clean but stupid
				if (!removeNoise(&avg_current_1)) break;
				if (!removeNoise(&avg_current_2)) break;
				initial_deriv = (func(avg_current_2) - func(avg_current_1))/h; 		// now we gotta take the differential of the current so we're gonna have time on the x-axis and the y-axis is the current

				if(!TimDelayExpired(stateDelayStart, 1000000)) break; /* Wait 1 second (1000000 microseconds) for precharging */

				if (!removeNoise(&avg_current_3)) break;
				if (!removeNoise(&avg_current_4)) break;

				latest_deriv = (func(avg_current_4) - func(avg_current_3))/h;

			    bool deriv_ok = (latest_deriv <= precharger.derivative_threshold);
			    bool current_ok = (avg_current_4 <= precharger.threshold);

			    if (deriv_ok && current_ok){
			    	enterClosingContactorState();
			    	break;
			    }

				precharger.switchError = true;
				HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);

	/*	This is my original hashed out version of the code
				checkStateStatus1 = removeNoise(&avg_current_1);
				if (checkStateStatus1){
					checkStateStatus2 = removeNoise(&avg_current_2);

					// now we gotta take the differential of the current so we're gonna have time on the x-axis and the y-axis is the current
					initial_deriv = (func(avg_current_2) - func(avg_current_1))/h;

					// ok so we got the inital values yay
					// now we gotta wait a sec
					if(TimDelayExpired(stateDelayStart, 1000000)) // Wait 1 second (1000000 microseconds) for precharging
					{
						checkStateStatus3 = removeNoise(&avg_current_3);
						if (checkStateStatus3){
							checkStateStatus4 = removeNoise(&avg_current_4);

							latest_deriv = (func(avg_current_4) - func(avg_current_3))/h;

							// check if the derivative of the latest - the initial is above a certain threshold value
							if ((latest_deriv) <= precharger.derivative_threshold){ // check if the derivative of the latest - the initial is above a certain threshold value
								if ((avg_current_4 <= precharger.threshold)){ // now we gotta check if we pass the threshold for the value (is it close to 0?)
									// yay, it's precharged, let's try closing the contactor:
									enterClosingContactorState();
								} else{ // Precharge Error
									precharger.switchError = true;
									HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
									HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);
								}
							} else{ // Precharge Error
								precharger.switchError = true;
								HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);
							}
						}
					}
				}
	*/

//				if(TimDelayExpired(stateDelayStart, 1000000)) /* Wait 1 second (1000000 microseconds) for precharging */
//				{
//					enterClosingContactorState();
//				}
			}
		break;
		case CLOSING_CONTACTOR:
			if(!contactorCommandClose)
			{
				enterAllOpenState();
			}
			else
			{
				if(TimDelayExpired(stateDelayStart, 250000)) /* delay of about half a second*/
				{
					if(HAL_GPIO_ReadPin(Contactor_Aux_Input_GPIO_Port, Contactor_Aux_Input_Pin) == GPIO_PIN_SET)
					{
						enterContactorClosedState();
					}
					else
					{
						/* Failed to close, return to ALL_OPEN*/
						enterAllOpenState();
						// error, should it try closing 3-5 times before calling it an error?
						contactor.switchError = true;
					}
				}
			}
		break;
		case CONTACTOR_CLOSED:
			if(!contactorCommandClose)
			{
				enterAllOpenState();
			} /* else nothing to do */
		break;
		case PRECHARGER_ERROR:
		case CONTACTOR_ERROR:
			/* Constantly open contactor until reboot */
			changeSwitch(&contactor, contactor.Switch_State, OPEN, contactor.Delay);
			contactor.switchError = true;
		break;
		default:
			/* unknown state, open contactor */
			enterAllOpenState();
			contactor.switchError = true;
		break;
	}

	return;
}

void enterAllOpenState()
{
	HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_RESET);
	contactorState = ALL_OPEN;
}

void enterPrecharging1State()
{
	if(boardIds.type != COMMON)
	{
		HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_SET);
	}
	stateDelayStart = TimGetTime();
	contactorState = PRECHARGING1;
}

void enterPrecharging2State()
{
	if(boardIds.type != COMMON)
	{
		HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_SET);
	}
	stateDelayStart = TimGetTime();
	contactorState = PRECHARGING2;
}

void enterClosingContactorState()
{
	HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_SET);
	stateDelayStart = TimGetTime();
	errorDelayStart = TimGetTime();
	contactorState = CLOSING_CONTACTOR;
}

void enterContactorClosedState()
{
	// open the precharger
	HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);

	contactorState = CONTACTOR_CLOSED;
}

void checkState(void)
{
	uint16_t* adcBuffer = AdcReturnAdcBuffer();

	// Update contactor state
	contactor.GPIO_State = HAL_GPIO_ReadPin(contactor.GPIO_Port_Sense, contactor.GPIO_Pin_Sense);
	if (contactor.GPIO_State == GPIO_PIN_SET)
  	{
		contactor.Switch_State = CLOSED;
	}
  	else
  	{
		contactor.Switch_State = OPEN;

	}

	// Update contactor state (No common precharger)
	if (boardIds.type != COMMON)
  	{
		if(adcBuffer[1] >= PRECHARGE_COMPLETE_THRESHOLD_ADC_COUNT)
    	{
			precharger.Switch_State = CLOSED;
			precharger.GPIO_State = GPIO_PIN_SET;
		}
    	else
    	{
			precharger.Switch_State = OPEN;
			precharger.GPIO_State = GPIO_PIN_RESET; // ensure it's reset

		}
#if 0
		if (precharger.Switch_State == CLOSED){
			precharger.GPIO_State = GPIO_PIN_SET; // set the pin
		} else {
			precharger.GPIO_State = GPIO_PIN_RESET; // ensure it's reset
		}
#endif
	}
}

/*
 *  The CAN message from the Master BMS will be passed into this function and this will delegate how to open/close
 *		message: CAN message
 */

float func(float x)
{
	return (2*x*x + 1);
}

bool removeNoise(int64_t *avg_current)
{
/* needs to be reworked for no delay handling */
#if 0

	pointStartOne = TimGetTime(); // get the current time
	int64_t initial_adcCount_y1 = adcBuffer[0];

	// our ADC reads the analog value and converts it into a number by multiplying it by 4096 (number of total number values). Now, we want to get the actual current so we divide by 4095 (the total number of combinations (since we start at 0)) and then we multiply by 3.3V since we are measuring 3.3. ADC resolution.
	int64_t voltage_1  = ((initial_adcCount_y1 * 3.3) / (4095));

	// minus the 2 offset
	int64_t voltaget_with_offset_1 = voltage_1 - 2;

	// we have to convert adc voltage to shunt resistor (done through (Vadc)* 0.0025)
	int64_t shunt_resistor_voltage_1 = (voltaget_with_offset_1 * 0.0025);

	// get the current
	int64_t current_1 = (shunt_resistor_voltage_1 / (precharger.resistance)/1000);

	if (TimDelayExpired(pointStartTwo, 1000)) /* Waits 1000 microseconds (1 milliseconds) */
	{
		// we're going to sample 2 points very close to each other to bypass noise and make it nominal

		int64_t initial_adcCount_y2 = adcBuffer[0];

		// the 3.3 and 4096 are for the ADC resolution
		int64_t voltage_2  = ((initial_adcCount_y2 * 3.3) / 4095);

		// minus the 2 offset
		int64_t voltaget_with_offset_2 = voltage_2 - 2;

		// we have to convert adc voltage to shunt resistor (done through (Vadc)* 0.0025)
		int64_t shunt_resistor_voltage_2 = (voltaget_with_offset_2 * 0.0025);

		// get the current
		int64_t current_2 = (shunt_resistor_voltage_2 / (precharger.resistance)/1000);


		// now we take the average of the two
		avg_current = (current_2 + current_1)/2;

		return true;
	}
	return false;


#endif

	return 0;
}

void Gatekeeper(CAN_Message *message)
{
	// WHEN YOU'RE INTIALIZING CHECK THE STATE!!!!!!!!!
	//checkState();

	// initialize the variables
	SwitchState WantedState;
	
	// get the bit that associates with our contactor (i.e the common contactor will be bit 0)
	uint8_t bitOfInterest = boardIds.type;
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

			if (boardIds.type == COMMON) {	// the common doesn't have precharging so we're bypassing it

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
			HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_SET);
			HAL_Delay(10);
			// to safely close the switch, we would need to first close the precharger to voltage spike.
			PrechargerState = changeSwitch(&precharger, precharger.Switch_State, CLOSED, precharger.Delay);
			// this will return if it's closed or not

			if (PrechargerState){	// precharger is closed
				/*  we need to precharge before trying to close																		*/
				/*  we first take the ADC value from the pin that reads the PRCH_CURRENT_ADC										*/
				/*  from that we need to convert this ADC count into voltage then divide it by (2*Rf) to get the current!			*/
				/*  get the current: 																								*/

				/* BEGINNING OF COMMENTED OUT PORTION OF PRECHARGE CODE 1
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
				END OF COMMENTED OUT PORTION OF PRECHARGE CODE 1*/
				HAL_Delay(1000); // wait 1 second to be safe it's precharged
				// the clock frequency is 80 MHz. So it does 1 tick every 0.000125 milliseconds. So 8000 ticks is 1 milliseconds. We want to wait 1 seconds so we do (8000 * 1000) to get 1 second (1000 milliseconds = 1 second)


				/* BEGINNING OF COMMENTED OUT PORTION OF PRECHARGE CODE 2
				int64_t avg_latest_current_1 = removeNoise();

				HAL_Delay(TICKS_BETWEEN_SAMPLING_POINTS); // the clock frequency is 80 MHz. So it waits 100 ticks

				int64_t avg_latest_current_2 = removeNoise();

				float latest_deriv;
				latest_deriv = (func(avg_latest_current_2) - func(avg_latest_current_1))/h;

				// check if the derivative of the latest - the initial is above a certain threshold value
				if ((latest_deriv) <= precharger.derivative_threshold){ // if we pass the threshold value
					// now we gotta check if we pass the threshold for the value (is it close to 0?)
					if ((avg_latest_current_2 <= precharger.threshold)){
				END OF COMMENTED OUT PORTION OF PRECHARGE CODE 2*/
						// yay, it's precharged, let's try closing the contactor:
						ContactorClosedState = changeSwitch(&contactor, contactor.Switch_State, CLOSED, contactor.Delay);
//					}
//				}

			}
			PrechargerState = changeSwitch(&precharger, precharger.Switch_State, OPEN, precharger.Delay); // open the precharger no matter what

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
		case SWITCH_ERROR: // we should never want an error state
			break;
		case BPS_ERROR:
			break;
	}
}

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

			} else if(switch_status == SWITCH_ERROR){ // try to change the switch to the wanted state
				switch_to_change->switchError = true;
				setSwitch(switch_to_change, SWITCH_ERROR); // if we get an error, open the switch
				return false;
			}
//			HAL_Delay(delayTime); // wait for a bit before checking if we achieved our goal

		}
		// this case is reached in case we couldn't close the contactor
		switch_to_change->switchError = true;
		setSwitch(switch_to_change, SWITCH_ERROR); // if we get an error, open the switch
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
	uint16_t* adcValues = AdcReturnAdcBuffer();
	switch (wanted_state){
			case OPEN:
				HAL_GPIO_WritePin(switch_to_change->GPIO_Port, switch_to_change->GPIO_Pin, GPIO_PIN_RESET);
//				HAL_Delay(switch_to_change->Delay); // wait for a bit before checking if we achieved our goal

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
//					HAL_Delay(switch_to_change->Delay); // wait for a bit before checking if we achieved our goal
					if(HAL_GPIO_ReadPin(switch_to_change->GPIO_Port_Sense, switch_to_change->GPIO_Pin_Sense) == 1){
						switch_to_change->Switch_State = CLOSED; // set the switch state to OPEN
						return CLOSED; // if the switch is closed, return closed
					} else {
						switch_to_change->Switch_State = CLOSING; // set the switch state to OPEN
						return CLOSING; // if the switch is not yet closed, return closing
					}

				} else { // CLOSE-ing Precharger, check if ADC is > 0 (current flowing through)
//					if(HAL_GPIO_ReadPin(DIAG_N_Input_GPIO_Port, DIAG_N_Input_Pin) != 0){ // we can read current now
				if(adcValues[1] >= PRECHARGE_COMPLETE_THRESHOLD_ADC_COUNT)
				{ // we can read current now

						switch_to_change->Switch_State = CLOSED; // set the switch state to OPEN
						return CLOSED; // if the switch is closed, return closed
					} else {
						switch_to_change->Switch_State = CLOSING; // set the switch state to OPEN
						return CLOSING; // if the switch is not yet closed, return closing
					}
				}

				break;

			case SWITCH_ERROR: // If we get an error, open the switch
				HAL_GPIO_WritePin(switch_to_change->GPIO_Port, switch_to_change->GPIO_Pin, GPIO_PIN_RESET);

				switch_to_change->Switch_State = OPEN; // set the switch state to OPEN
				return OPEN;
				break;

			default:
				// ???
				break;

	}

	return SWITCH_ERROR; // return open if we fail all the switch statements above. This case should never happen so it's an error!

}
