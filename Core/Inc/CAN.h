/*
 * CAN.h
 *
 *  Created on: Jun 21, 2025
 *      Author: violet
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "boardDefines.h"

#define HEARTBEAT_ID_BASE 0x200
#define STATUS_ID_BASE 0x210

#define HEARTBEAT_DLC 2
#define STATUS_DLC 4

#define CONTACTOR_COMMAND_ID 0x101

uint32_t makingCANMessage();
bool Check_CAN_Messages(void);
CAN_TxHeaderTypeDef SendingCANMessage(uint32_t extendedID, uint8_t* Data, uint8_t DLC_num);

#endif /* INC_CAN_H_ */
