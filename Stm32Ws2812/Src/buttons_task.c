/*
 * buttons_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "cmsis_os.h"
#include "buttons_task.h"

/**
* @brief Function implementing the buttonsTask thread.
* @param argument: Not used
* @retval None
*/
void ButtonsTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
