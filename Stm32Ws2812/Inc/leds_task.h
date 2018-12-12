/*
 * leds_task.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#ifndef LEDS_TASK_H_
#define LEDS_TASK_H_

#include <stdint.h>
#include <stdbool.h>

//----------------------------------------------------------
#define LEDS_TIME_MIN_SEC (50)
#define LEDS_TIME_MAX_SEC (150)

//----------------------------------------------------------
void CreateLedsTask(void);
void LedsTask(void const * argument);

bool set_leds_run_time(uint32_t sec);
void set_leds_color(uint8_t r, uint8_t g, uint8_t b);

#endif /* LEDS_TASK_H_ */
