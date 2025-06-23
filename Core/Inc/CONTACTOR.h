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

#define COMMON_LINE_CURRENT_RATIO 50U
#define MOTOR_LINE_CURRENT_RATIO  50U
#define ARRAY_LINE_CURRENT_RATIO  50U
#define LV_LINE_CURRENT_RATIO     50U
#define CHARGE_LINE_CURRENT_RATIO 50U

#define COMMON_PRECHARGER_RESISTANCE   0.005 
#define MOTOR_PRECHARGER_RESISTANCE    0.005
#define ARRAY_PRECHARGER_RESISTANCE    0.005
#define LV_PRECHARGER_RESISTANCE       0.005
#define CHARGE_PRECHARGER_RESISTANCE   0.005

#define COMMON_CONTACTOR_RESISTANCE   6.6
#define MOTOR_CONTACTOR_RESISTANCE    6.6
#define ARRAY_CONTACTOR_RESISTANCE    6.6
#define LV_CONTACTOR_RESISTANCE       6.6
#define CHARGE_CONTACTOR_RESISTANCE   6.6

void checkState(void);
void ContactorTask(void);
float func(float x);
bool removeNoise(int64_t *avg_current);
#endif /* INC_CONTACTOR_H_ */
