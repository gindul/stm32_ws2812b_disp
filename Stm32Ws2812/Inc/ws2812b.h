/*
 * ws2812b.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#ifndef WS2812B_H_
#define WS2812B_H_

#include <stdint.h>

#include <stm32l1xx.h>
#include <stm32l1xx_hal_rcc.h>
#include <stm32l1xx_hal_gpio.h>
#include <stm32l1xx_hal_tim.h>
#include <stm32l1xx_hal_dma.h>

#define WS2812B_BUFFER_SIZE     60
#define WS2812B_START_SIZE      2

#define WS2812B_FREQUENCY       24000000
#define WS2812B_PERIOD          30

#define WS2812B_PULSE_HIGH      21
#define WS2812B_PULSE_LOW       9

//----------------------------------------------------------
typedef struct RGB {
    uint8_t r, g, b;
} RGB_t;

//----------------------------------------------------------
void ws2812b_Init(void);
void ws2812b_SendRGB(RGB_t *rgb, unsigned count);

#endif /* WS2812B_H_ */
