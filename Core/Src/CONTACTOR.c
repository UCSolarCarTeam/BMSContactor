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
#include "stm32l4xx_ll_gpio.h"  // Include the LL GPIO header


/* global variables */
extern ADC_HandleTypeDef hadc1;
extern BoardIds boardIds;
extern bool contactorCommandClose;
Precharger_t precharger = {0};
Contactor_t contactor = {0};

/* Local variables */
static Contactor_State contactorState = ALL_OPEN;
static uint32_t stateDelayStart = 0;
static uint32_t errorDelayStart = 0;

uint32_t lineCurrentPerContactor[] = {COMMON_LINE_CURRENT_RATIO,
									MOTOR_LINE_CURRENT_RATIO,
									ARRAY_LINE_CURRENT_RATIO,
									CHARGE_LINE_CURRENT_RATIO};
uint32_t prechargerResistancePerContactor[] = {COMMON_PRECHARGER_RESISTANCE,
											 MOTOR_PRECHARGER_RESISTANCE,
											 ARRAY_PRECHARGER_RESISTANCE,
											 CHARGE_PRECHARGER_RESISTANCE};
uint32_t contactorResistancePerContactor[] = {COMMON_CONTACTOR_RESISTANCE,
											MOTOR_CONTACTOR_RESISTANCE,
											ARRAY_CONTACTOR_RESISTANCE,
											CHARGE_CONTACTOR_RESISTANCE};

static void enterAllOpenState();
static void enterPrecharging1State();
static void enterPrecharging2State();
static void enterClosingContactorState();
static void enterContactorClosedState();

void initContactor(void)
{
	Contactor_Type type = boardIds.type;

	precharger.GPIO_Port = PRECHARGE_ON_Output_GPIO_Port,
	precharger.GPIO_Pin = PRECHARGE_ON_Output_Pin,
	precharger.GPIO_Port_Sense = PRECHARGE_Sense_On_Output_GPIO_Port,
	precharger.GPIO_Pin_Sense = PRECHARGE_Sense_On_Output_Pin,
	precharger.Switch_State = OPEN, // All pins except the common should start off as open.
	precharger.switchError = false,
	//precharger.Delay = 3000, // DOESN'T NEED A DELAY
	precharger.resistance = 0.005, // Cannot be 0. WILL CHANGE BASED ON CONACTOR ** IT COULD 0.3!!!
	precharger.threshold = 1, // WILL CHANGE BASED ON CONACTOR **
	precharger.derivative_threshold = 1; // WILL CHANGE BASED ON CONACTOR **

	contactor.GPIO_Port = Contactor_ON_Output_GPIO_Port,
  	contactor.GPIO_Pin = Contactor_ON_Output_Pin,
  	contactor.GPIO_Port_Sense = Contactor_Aux_Input_GPIO_Port,
  	contactor.GPIO_Pin_Sense = Contactor_Aux_Input_Pin,
  	contactor.GPIO_State = GPIO_PIN_RESET, // All pins except the common should start off as open. Reset = 0
  	contactor.Switch_State = OPEN, // All pins except the common should start off as open.
  	contactor.switchError = false,
  	contactor.BPSError = false,
  	//contactor.Delay = 8000*250, // NEEDS A DELAY OF ABOUT A 1/4 OF A SECOND
  	contactor.Delay = 250, // NEEDS A DELAY OF ABOUT A 1/4 OF A SECOND
  	//contactor.resistance = contactorResistancePerContactor[type]
  	contactor.lineCurrentAmpsPerADCVoltage = lineCurrentPerContactor[type];// WILL CHANGE BASED ON CONACTOR ** can be 100!!! or 30!!!
}

static uint32_t number_of_tries_closing_contactor = 0;

void ContactorTask(void)
{
//	checkState();

	switch (contactorState)
	{
		case ALL_OPEN:
			if(contactorCommandClose)
			{
				/* special code for array */
				/* common does not precharge */
				enterClosingContactorState();
			}
			else{
				// be sure you're open
				enterAllOpenState();
			}
		break;
		case PRECHARGING1:
		if(!contactorCommandClose)
		{
			enterAllOpenState();
			stateDelayStart = TimGetTime(); // added for trip
		} /* else nothing to do */
		break;
		case PRECHARGING2:
		if(!contactorCommandClose)
		{
			enterAllOpenState();
			stateDelayStart = TimGetTime(); // added for trip
		} /* else nothing to do */
		break;
		case CLOSING_CONTACTOR:
		if(!contactorCommandClose)
		{
			enterAllOpenState();
			stateDelayStart = TimGetTime(); // added for trip
		} /* else nothing to do */
		break;
		case CONTACTOR_CLOSED:
			if(!contactorCommandClose)
			{
				enterAllOpenState();
				stateDelayStart = TimGetTime(); // added for trip
			} /* else nothing to do */
		break;
		case PRECHARGER_ERROR:
		case CONTACTOR_ERROR:
			/* Constantly open contactor until reboot */
			HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_RESET);
			contactor.switchError = true;
		break;
		default:
			/* unknown state, open contactor */
			enterAllOpenState();
			stateDelayStart = TimGetTime(); // added for trip
			contactor.switchError = true;
		break;
	}

	return;
}

static void enterAllOpenState()
{
	HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_RESET);

	contactor.Switch_State = OPEN;
	contactorState = ALL_OPEN;
}


static void enterClosingContactorState()
{
	HAL_GPIO_WritePin(Contactor_ON_Output_GPIO_Port, Contactor_ON_Output_Pin, GPIO_PIN_SET);
	errorDelayStart = TimGetTime();

	/* special code for array */
	contactor.Switch_State = CLOSED;
	contactorState = CONTACTOR_CLOSED;

}

static void enterContactorClosedState()
{
	contactorState = CONTACTOR_CLOSED;
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
