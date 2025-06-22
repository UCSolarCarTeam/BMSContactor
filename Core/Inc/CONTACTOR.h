/*
 * CONTACTOR.h
 *
 *  Created on: Jun 21, 2025
 *      Author: RS7520
 */

#ifndef INC_CONTACTOR_H_
#define INC_CONTACTOR_H_

#include "boardDefines.h"

#define PRECHARGER_ADC_CHANNEL 0
#define PRECHARGE_COMPLETE_THRESHOLD_ADC_COUNT 100

#define TICKS_BETWEEN_SAMPLING_POINTS 100 // we can adjust this based off of testing the sampling points

void checkState(void);
bool changeSwitch(SwitchInfo_t* switch_to_change, SwitchState current_state, SwitchState wanted_state, uint32_t delayTime);
SwitchState setSwitch(SwitchInfo_t* switch_to_change, SwitchState wanted_state);
void setDebugLED(SwitchInfo_t* switch_to_change);

void Gatekeeper(CAN_Message *message);
void ContactorTask(void);
float func(float x);
bool removeNoise(int64_t *avg_current);
#endif /* INC_CONTACTOR_H_ */
