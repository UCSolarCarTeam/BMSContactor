/*
 * boardDefines.h
 *
 *  Created on: june 21, 2025
 *      Author: violet
 * System includes, defines, and structs
 */

#ifndef INC_BOARD_DEFINES_H
#define INC_BOARD_DEFINES_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>  // Include this header for memcpy
#include "stm32l4xx_hal.h"

/* debug defines */
#define DEBUG_CONTACTOR_TYPE COMMON   // WILL CHANGE BASED ON CONACTOR **
#define UART_DEBUG 0
#define FAKE_CAN 0
#define PRECHARGER_DEBUG 0

#define SYSTEM_UPDATE_PERIOD 10000 /* in us */

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

#define CONTACTOR_CLOSE 1
#define CONTACTOR_OPEN 0

// assigning all the contactors a number. Must match order in CAN message IDs
typedef enum
{
    COMMON = 0,
    MOTOR,
    ARRAY,
//    LV,
    CHARGE,
    CONTACTOR_TYPE_NUM,
    NONE,
} Contactor_Type;

// assigning all the states of the switch a number
typedef enum
{
    OPEN = 0,
    CLOSED,
    CLOSING, // Intermediate state between open and closed
    SWITCH_ERROR,
	BPS_ERROR
} SwitchState;

// assigning all the contactors a number
typedef enum
{
	ALL_OPEN = 0,
    PRECHARGING1,
    PRECHARGING2,
    CLOSING_CONTACTOR,
    CONTACTOR_CLOSED,
    PRECHARGER_ERROR,
    CONTACTOR_ERROR
} Contactor_State;

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
    uint32_t resistance;
    uint32_t lineCurrentAmpsPerADCVoltage;

} Contactor_t;

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
    uint32_t resistance;
    uint32_t threshold;
    uint32_t derivative_threshold;

} Precharger_t;

// this is the message struct
typedef struct {
    uint32_t id;           // Message ID
    uint8_t data[8];       // Message data (max 8 bytes for CAN)
    uint8_t dlc;           // Data Length Code
    uint8_t is_extended;   // 0 for standard ID, 1 for extended ID
    uint8_t is_rtr;        // 0 for data frame, 1 for remote frame
} CAN_Message;

typedef struct {
    Contactor_Type type;
    uint32_t canIdOffset;
    bool extendedId;
} BoardIds;

#endif /* INC_BOARD_DEFINES_H */
