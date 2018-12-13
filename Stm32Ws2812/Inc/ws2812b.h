/*
 * ws2812b.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#ifndef WS2812B_H_
#define WS2812B_H_

#include <stdint.h>

#define WS2812B_FREQUENCY       	(800000) // it is fixed: WS2812 require 800kHz
#define WS2812B_TIMER_CLOCK_FREQ	(16000000)   	// can be modified - multiples of 0.8MHz are suggested
#define WS2812B_PERIOD				(WS2812B_TIMER_CLOCK_FREQ / WS2812B_FREQUENCY)

#define WS2812_1       				(TIMER_PERIOD * 2 / 3)
#define WS2812_0       				(TIMER_PERIOD / 3)

#define LED_CFG_LEDS_CNT_STRIP      (750)
#define LED_CFG_LEDS_CNT_DISPLAY    (5*20)
#define LED_CFG_BYTES_PER_LED   	(3)
#define LED_CFG_RAW_BYTES_PER_LED   (LED_CFG_BYTES_PER_LED * 8)

//----------------------------------------------------------
typedef struct RGB {
    uint8_t r, g, b;
} RGB_t;

//----------------------------------------------------------
void ws2812b_Init(void);
void ws2812b_SendRGB(RGB_t *rgb, unsigned count);

#endif /* WS2812B_H_ */
