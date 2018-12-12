/*
 * ble_task.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "cmsis_os.h"
#include "ble_task.h"

/**
* @brief Function implementing the bleTask thread.
* @param argument: Not used
* @retval None
*/
void BleTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
