/*
 * SendingCANMessage.h
 *
 *  Created on: Jan 15, 2025
 *      Author: khadeejaabbas
 */

#ifndef INC_SENDINGCANMESSAGE_H_
#define INC_SENDINGCANMESSAGE_H_
#include <main.h>
#include <stdint.h>

//void SendingCANMessage(uint8_t TxData[8], uint8_t DLC_num);
CAN_TxHeaderTypeDef SendingCANMessage(uint32_t extendedID, uint8_t TxData[8], uint8_t DLC_num);

#endif /* INC_SENDINGCANMESSAGE_H_ */
