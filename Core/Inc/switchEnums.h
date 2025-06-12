/*
 * SwitchState.h
 *
 *  Created on: Sep 18, 2024
 *      Author: khadeejaabbas
 */

#ifndef INC_TASK_H_FILES_CONTACTOR_STATE_H_
#define INC_TASK_H_FILES_CONTACTOR_STATE_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

// this ensure this file is included only once in the project (not repeated)
#pragma once

// macro for the precharger delay (1 second)
#define PRECHARGER_DELAY 1000
// macro for the time to close delay (1 second)
#define TIMETOCLOSE_DELAY 1000
// macro for the time to open delay (1 second)
#define TIMETOOPEN_DELAY 1000
// macro for the time to retry closing delay (10 seconds)
#define TIMETOCLOSERETRY_DELAY 10000
// max number of retrying closing/opening the swtich
#define MAX_NUM_OF_RETRIES 5


// assigning all the states of the switch a number
typedef enum
{
    OPEN = 0,
    CLOSED,
    CLOSING, // Intermediate state between open and closed
    CONTACTOR_ERROR,
	BPS_ERROR
} SwitchState;


// ok, so each contactor has it's own number
// and then we need to assign each contactor their number
// 0 = common
// 1 = Motor 1
// 2 = Motor 2
// 3 = Array
// 4 = LV
// 5 = Charge

// assigning all the contactors a number
typedef enum
{
	COMMON = 0,
	MOTOR,
//	MOTOR2,
	ARRAY,
	LV,
    CHARGE
} Contactors;

// a struct for the general info for the switch (precharger or contactor)
typedef struct
{
	GPIO_TypeDef * GPIO_Port;
	uint16_t GPIO_Pin;
	GPIO_TypeDef * GPIO_Port_Sense;
	uint16_t GPIO_Pin_Sense;
	GPIO_PinState GPIO_State;
	SwitchState Switch_State;
    bool switchError;
    bool BPSError;
    uint16_t Delay;
    uint16_t WhichContactor;
    uint32_t extendedID;
    uint32_t resistance;
    uint32_t threshold;
    uint32_t derivative_threshold;
    bool isContactor;
    uint32_t lineCurrentAmpsPerADCVoltage;

} SwitchInfo_t;

// this is the message struct
typedef struct {
    uint32_t id;           // Message ID
    uint8_t data[8];       // Message data (max 8 bytes for CAN)
    uint8_t dlc;           // Data Length Code
    uint8_t is_extended;   // 0 for standard ID, 1 for extended ID
    uint8_t is_rtr;        // 0 for data frame, 1 for remote frame
} CAN_Message;

#endif
