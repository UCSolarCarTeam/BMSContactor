/*
 * Gatekeeper.h
 *
 *  Created on: Sep 7, 2024
 *      Author: khadeejaabbas
 */

#include <changeSwitch.h>

#ifndef INC_TASK_H_FILES_GATEKEEPER_H_
#define INC_TASK_H_FILES_GATEKEEPER_H_

#define TICKS_BETWEEN_SAMPLING_POINTS 100 // we can adjust this based off of testing the sampling points

// Declaration of the Gatekeeper function
void Gatekeeper(CAN_Message *message);


#endif /* INC_TASK_H_FILES_GATEKEEPER_H_ */





