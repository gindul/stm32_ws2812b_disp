/*
 * leds_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "cmsis_os.h"
#include "leds_task.h"

//----------------------------------------------------------
typedef struct {
	uint32_t time_sec;
	uint8_t color_r;
	uint8_t color_g;
	uint8_t color_b;
} leds_cfg_t;

static volatile leds_cfg_t leds_cfg = {
	.time_sec = 50,
	.color_r = 0xFF,
	.color_g = 0xFF,
	.color_b = 0xFF
};

osThreadId ledsTaskHandle;
osMutexId cfgMutexHandle;
//----------------------------------------------------------
void CreateLedsTask(void)
{
	/* definition and creation of cfgMutex */
	osMutexDef(cfgMutex);
	cfgMutexHandle = osMutexCreate(osMutex(cfgMutex));

	/* definition and creation of ledsTask */
	osThreadDef(ledsTask, LedsTask, osPriorityNormal, 0, 128);
	ledsTaskHandle = osThreadCreate(osThread(ledsTask), NULL);
}

/**
  * @brief  Function implementing the ledsTask thread.
  * @param  argument: Not used
  * @retval None
  */
void LedsTask(void const * argument)
{
  for(;;)
  {
    osDelay(100);
  }
}

//----------------------------------------------------------
bool set_leds_run_time(uint32_t sec)
{
	if(sec < LEDS_TIME_MIN_SEC || sec > LEDS_TIME_MAX_SEC) {
		return false;
	}

	osMutexWait(cfgMutexHandle, osWaitForever);
	leds_cfg.time_sec = sec;
	osMutexRelease(cfgMutexHandle);

	return true;
}

void set_leds_color(uint8_t r, uint8_t g, uint8_t b)
{
	osMutexWait(cfgMutexHandle, osWaitForever);
	leds_cfg.color_r = r;
	leds_cfg.color_g = g;
	leds_cfg.color_b = b;
	osMutexRelease(cfgMutexHandle);
}
