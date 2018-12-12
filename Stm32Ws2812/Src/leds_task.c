/*
 * leds_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "cmsis_os.h"
#include "leds_task.h"

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
