/*
 * TIMER.h
 *
 *  Created on: Jun 21, 2025
 *      Author: violet
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "boardDefines.h"

uint32_t TimGetTime(void);
uint32_t TimGetTimeElapsedSince(uint32_t time);
bool TimDelayExpired(uint32_t start, uint32_t delay);

#endif /* INC_TIMER_H_ */
