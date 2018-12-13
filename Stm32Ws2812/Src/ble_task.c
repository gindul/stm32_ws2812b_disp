/*
 * ble_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os.h"
#include "ble_task.h"

//----------------------------------------------------------
#define BT_PROTO_HDR_SIZE (4)
#define BT_PROTO_CRC_SIZE (1)
#define BT_PROTO_MAX_PAYLOAD_SIZE  (255)
#define BT_PROTO_WRAPPER_SIZE      (BT_PROTO_HDR_SIZE + BT_PROTO_CRC_SIZE)
#define BT_PROTO_MAX_SIZE          (BT_PROTO_WRAPPER_SIZE + BT_PROTO_MAX_PAYLOAD_SIZE)

#define FLAG_1 (0x55)
#define FLAG_2 (0xAA)

//----------------------------------------------------------
typedef struct {
	int step;
	int data_idx;
	int expect_len;
	uint8_t data[BT_PROTO_MAX_SIZE];
} frame_holder_t;

typedef enum {
	STEP_FLAG_1 = 0,
	STEP_FLAG_2,
	STEP_OPCODE,
	STEP_LEN,
	STEP_PAYLOAD,
	STEP_CRC
} frame_step_e;

//----------------------------------------------------------
extern UART_HandleTypeDef huart1;
osMessageQId btMsgQueueHandle;
frame_holder_t g_frame;

//----------------------------------------------------------
static bool FrameParser(uint8_t rx);
static void ProcessFrame(void);
static void UartSendStatus(uint8_t status);
static uint8_t Crc8(uint8_t *buf, int len);

//----------------------------------------------------------
void BleTaskInit(void)
{
	/* definition and creation of cfgMutex */
	osMessageQDef(btMsgQueue, 16, uint8_t);
	btMsgQueueHandle = osMessageCreate(osMessageQ(btMsgQueue), NULL);

	memset(g_frame, 0x00, sizeof(frame_holder_t));
}

/**
* @brief Function implementing the bleTask thread.
* @param argument: Not used
* @retval None
*/
void BleTask(void const * argument)
{
	BleTaskInit();
	for(;;)
	{
		osEvent evt_rx = osMessageGet(btMsgQueueHandle, 10);
		if(evt_rx.status == osEventMessage) { //we have one byte, read it from queue
			uint8_t rx_byte = (uint8_t)evt_rx.value.v;
			bool ret = FrameParser(rx_byte);

			if(ret == true) {
				ProcessFrame(); //frame parsed, process it
			}
		}
		osDelay(1);
	}
}

void BleTaskPutQueueByte(uint8_t rx)
{
	osMessagePut(btMsgQueueHandle, rx, 0);
}

static bool FrameParser(uint8_t rx)
{
	switch(g_frame.step)
	{
	case STEP_FLAG_1:
		if(rx == FLAG_1) {
			g_frame.step++;
			g_frame.data[g_frame.data_idx++] = rx;
		}
		break;
	case STEP_FLAG_2:
		if(rx == FLAG_2) {
			g_frame.step++;
			g_frame.data[g_frame.data_idx++] = rx;
		} else if(rx == FLAG_1) {
			g_frame.data_idx = 0;
			g_frame.data[g_frame.data_idx++] = rx;
		}
		break;
	case STEP_OPCODE:
		g_frame.step++;
		g_frame.data[g_frame.data_idx++] = rx;
		break;
	case STEP_LEN:
		g_frame.step++;
		g_frame.data[g_frame.data_idx++] = rx;
		g_frame.expect_len = (BT_PROTO_WRAPPER_SIZE + rx);
		break;
	case STEP_PAYLOAD:
		if(g_frame.data_idx < (g_frame.expect_len - 1)) {
			g_frame.data[g_frame.data_idx++] = rx;
		} else {
			g_frame.step++;
			g_frame.data[g_frame.data_idx++] = rx;
		}
		break;
	case STEP_CRC:
		g_frame.data[g_frame.data_idx] = rx;
		g_frame.step = 0;
		g_frame.data_idx = 0;
		return true;
	default:
		g_frame.step = 0;
		g_frame.data_idx = 0;
		g_frame.expect_len = 0;
		break;
	}
	return false;
}

static void ProcessFrame(void)
{
	const int data_count = g_frame.expect_len - 1;
	const uint8_t crc_expect = g_frame.data[data_count];
	const uint8_t crc_calc = Crc8(g_frame.data, data_count);

	if(crc_expect != crc_calc) {
		UartSendStatus(BT_STATUS_WRONG_CRC);
		return;
	}

	uint8_t op_code = g_frame.data[2];
	uint8_t payload_len = g_frame.data[3];
	switch(op_code)
	{
	case OPCODE_STATUS:
		UartSendStatus(BT_STATUS_OK); //used for pool/keepalive
		break;

	case OPCODE_CFG_SEGMENT:
		break;
	case OPCODE_START_SEGMENT:
		break;
	case OPCODE_SET_COLOR_SEGMENT:
		break;
	case OPCODE_SET_TEMP_SEGMENT:
		break;

	case OPCODE_GET_COLOR_SEGMENT:
		break;
	case OPCODE_GET_TEMP_SEGMENT:
		break;

	default:
		UartSendStatus(BT_STATUS_WRONG_OPCODE);
		break;
	}
}

static void UartSendStatus(uint8_t status)
{
	const int frame_len = 6;
	uint8_t frame_wrog_crc[] = {FLAG_1, FLAG_2, OPCODE_STATUS, 0x01, 0x00, 0x00};
	frame_wrog_crc[4] = status;
	frame_wrog_crc[frame_len - 1] = Crc8(frame_wrog_crc, frame_len - 1);

	HAL_UART_Transmit(&huart1, frame_wrog_crc, frame_len, 100);
}

static uint8_t Crc8(uint8_t *buf, int len)
{
	uint8_t c, b, j, crc=0;

	while (len-- ) {
		b = *buf++;
		for ( j=0; j<8; j++ ) {
			c = ((crc^b) & 1) << 7;
			if (c) crc ^= 0x18;
			crc = (crc >> 1) | c;
			b >>= 1;
		}
	}
	return crc;
}

