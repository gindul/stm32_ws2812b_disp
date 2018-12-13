/*
 * ble_task.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#ifndef BLE_TASK_H_
#define BLE_TASK_H_

//----------------------------------------------------------
typedef enum {
	OPCODE_STATUS = 0x00,

	OPCODE_CFG_SEGMENT = 0x01,
	OPCODE_START_SEGMENT = 0x02,
	OPCODE_SET_COLOR_SEGMENT = 0x03,
	OPCODE_SET_TEMP_SEGMENT = 0x04,

	OPCODE_GET_COLOR_SEGMENT = 0x13,
	OPCODE_GET_TEMP_SEGMENT = 0x14
} opcodes_e;

typedef enum {
	BT_STATUS_OK = 0,
	BT_STATUS_WRONG_CRC,
	BT_STATUS_WRONG_OPCODE,
	BT_STATUS_WRONG_PARAMS
} bt_status_e;

//----------------------------------------------------------
void BleTaskPutQueueByte(uint8_t rx);
void BleTask(void const * argument);

#endif /* BLE_TASK_H_ */
