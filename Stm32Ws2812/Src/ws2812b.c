/*
 * ws2812b.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Radu Donos
 */

#include "ws2812b.h"
#include "cmsis_os.h"
#include "stm32l1xx_hal.h"

//----------------------------------------------------------
static uint8_t leds_strip_colors[LED_CFG_BYTES_PER_LED * LED_CFG_LEDS_CNT_STRIP];
static uint8_t leds_display_colors[LED_CFG_BYTES_PER_LED * LED_CFG_LEDS_CNT_DISPLAY];

//----------------------------------------------------------
/**
 * \brief           Temporary array for dual LED with extracted PWM duty cycles
 *
 * We need LED_CFG_RAW_BYTES_PER_LED bytes for PWM setup to send all bits.
 * Before we can send data for first led, we have to send reset pulse, which must be 50us long.
 * PWM frequency is 800kHz, to achieve 50us, we need to send 40 pulses with 0 duty cycle = make array size MAX(2 * LED_CFG_RAW_BYTES_PER_LED, 40)
 */
static uint32_t tmp_led_data[2 * LED_CFG_RAW_BYTES_PER_LED];

static uint8_t          is_reset_pulse;     /*!< Status if we are sending reset pulse or led data */
static volatile uint8_t is_updating;        /*!< Is updating in progress? */
static uint32_t         current_led;        /*!< Current LED number we are sending */

//----------------------------------------------------------
void ws2812b_Init(void)
{

}
