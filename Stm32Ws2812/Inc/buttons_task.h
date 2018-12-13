/*
 * buttons_task.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#ifndef BUTTONS_TASK_H_
#define BUTTONS_TASK_H_

//----------------------------------------------------------
#define TASK_READ_I2C_SIGNAL (0x00000001)
#define CY8CMBR3108_I2C_ADDR (0x55)
#define CY8CMBR3108_CMD_BUTTON_STAT (0xAA)

#define BUTTON_DEC_BIT		 (1<<0)
#define BUTTON_INC_BIT		 (1<<1)
#define BUTTON_START_BIT	 (1<<2)
#define BUTTON_SEGM_BIT		 (1<<3)

//----------------------------------------------------------
void ButtonsTask(void const * argument);

#endif /* BUTTONS_TASK_H_ */
