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
				if(TimDelayExpired(stateDelayStart, 1000000)) /* Wait 1 second (1000000 microseconds) for precharging */
				{
					enterClosingContactorState();
				}
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
			HAL_GPIO_WritePin(PRECHARGE_ON_Output_GPIO_Port, PRECHARGE_ON_Output_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PRECHARGE_Sense_On_Output_GPIO_Port, PRECHARGE_Sense_On_Output_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_RESET);
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
	}
}

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
