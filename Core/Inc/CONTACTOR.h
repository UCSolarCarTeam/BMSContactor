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
#define PRECHARGE_COMPLETE_THRESHOLD_ADC_COUNT 5000

#define TICKS_BETWEEN_SAMPLING_POINTS 100 // we can adjust this based off of testing the sampling points
#define MAX_NUM_OF_CLOSING_CONACTOR_TRIES 5

#define COMMON_LINE_CURRENT_RATIO 50U
#define MOTOR_LINE_CURRENT_RATIO  50U
#define ARRAY_LINE_CURRENT_RATIO  50U
#define LV_LINE_CURRENT_RATIO     50U
#define CHARGE_LINE_CURRENT_RATIO 50U

#define COMMON_PRECHARGER_RESISTANCE   0.1 // don't matter
#define MOTOR_PRECHARGER_RESISTANCE    0.005*1000 // in millohms
#define ARRAY_PRECHARGER_RESISTANCE    0.1*1000
#define LV_PRECHARGER_RESISTANCE       0.1*1000
#define CHARGE_PRECHARGER_RESISTANCE   0.1*1000

#define COMMON_CONTACTOR_RESISTANCE   1 // in millivolts/A
#define MOTOR_CONTACTOR_RESISTANCE    2
#define ARRAY_CONTACTOR_RESISTANCE    16
#define LV_CONTACTOR_RESISTANCE       16
#define CHARGE_CONTACTOR_RESISTANCE   8

void checkState(void);
void ContactorTask(void);
float func(float x);
bool removeNoise(int64_t *avg_current);
#endif /* INC_CONTACTOR_H_ */
