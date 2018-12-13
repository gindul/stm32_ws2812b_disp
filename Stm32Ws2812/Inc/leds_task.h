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
#define MAX_SEGMENTS (2)

//----------------------------------------------------------
void CreateLedsTask(void);
void LedsTask(void const * argument);

bool set_leds_run_time(int segment, uint32_t sec);
void set_leds_color(int segment, uint8_t r, uint8_t g, uint8_t b);
void toggle_leds_segment();
void inc_leds_run_time();
void dec_leds_run_time();
void toggle_leds_start();

#endif /* LEDS_TASK_H_ */
