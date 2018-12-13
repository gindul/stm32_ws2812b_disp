/*
 * buttons_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "buttons_task.h"
#include "cmsis_os.h"
#include "stm32l1xx_hal.h"

#include "leds_task.h"

//----------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;

//----------------------------------------------------------
static void ButtonsInit(void)
{
	//noting to do, CY8CMBR3108 is already init as we need
}

static void ParseButtonsState(uint8_t buttons_state)
{
	if(buttons_state & BUTTON_SEGM_BIT) {
		toggle_leds_segment();
	}

	// check inc/dec buttons -- mutual exclusive
	if(buttons_state & BUTTON_DEC_BIT) {
		dec_leds_run_time();
	} else if(buttons_state & BUTTON_INC_BIT) {
		inc_leds_run_time();
	}

	if(buttons_state & BUTTON_START_BIT) {
		toggle_leds_start();
	}
}

/**
* @brief Function implementing the buttonsTask thread.
* @param argument: Not used
* @retval None
*/
void ButtonsTask(void const * argument)
{
	const uint32_t rw_timeout_ms = 200;
	uint8_t cmd_read_stat[1] = { CY8CMBR3108_CMD_BUTTON_STAT };
	uint8_t buttons_status_buf[2];
	osEvent evt;

	ButtonsInit();
	for(;;)
	{
		evt = osSignalWait(TASK_READ_I2C_SIGNAL, osWaitForever);
		if(evt.status == osEventSignal)
		{
			//start reading of status from touch controller
			HAL_StatusTypeDef status;
			status = HAL_I2C_Master_Transmit(&hi2c1, CY8CMBR3108_I2C_ADDR, cmd_read_stat, 1, rw_timeout_ms);
			if(status != HAL_OK) {
				continue; //could not write to touch controller, retry next time
			}
			status = HAL_I2C_Master_Receive(&hi2c1, CY8CMBR3108_I2C_ADDR, buttons_status_buf, 2, rw_timeout_ms);
			if(status != HAL_OK) {
				continue; //could not read from touch controller, retry next time
			}

			ParseButtonsState(buttons_status_buf[1]); //buttons 0-3 is in least significant byte
		}
	}
}
