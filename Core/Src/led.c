/*
 * led.c
 *
 *  Created on: May 10, 2025
 *      Author: Michael Pillon
 */


#include "led.h"

led_t  led_new(
	uint16_t pin,
	GPIO_TypeDef *port,
	bool activeState) {
	led_t newLed;
	newLed.pin = pin;
	newLed.port = port;
	newLed.activeState = activeState;
	led_off(&newLed);
	return newLed;
}

/*
 * Turns LED on
 */
void led_on (led_t *led) {

}

/*
 * Turns LED off
 */
void led_off (led_t *led) {
	HAL_GPIO_WritePin(led->port, led->pin, !(led->activeState));
}

/*
 * Toggles LED
 */

void led_toggle(led_t *led) {
	HAL_GPIO_TogglePin(led->port, led->pin);
}
