/*
 * changeSwitch.h
 *
 *  Created on: Sep 7, 2024
 *      Author: Dominic Choi
 */

#ifndef INC_TASK_H_FILES_TRY_CHANGING_SWITCHES_H_
#define INC_TASK_H_FILES_TRY_CHANGING_SWITCHES_H_
#include <switchEnums.h>
#include <stdbool.h>

#define PRECHARGER_ADC_CHANNEL 0 // change this if I guessed wrong lol. You guessed right

bool changeSwitch(SwitchInfo_t* switch_to_change, SwitchState current_state, SwitchState wanted_state, uint32_t delayTime);
SwitchState setSwitch(SwitchInfo_t* switch_to_change, SwitchState wanted_state);
void setDebugLED(SwitchInfo_t* switch_to_change);


#endif /* INC_TASK_H_FILES_TRY_CHANGING_SWITCHES_H_ */

