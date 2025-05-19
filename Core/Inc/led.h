/**
 * led.h
 *
 *      Author: Michael Pillon
 *
 *      For discrete control of an LED
 *
 */

#ifndef INC_LED_H_
#define INC_LED_H_


#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
//#include "stdio.h"
//#include "ctype.h"
//#include "string.h"
//



typedef struct {
	uint16_t pin;
	GPIO_TypeDef *port;
	bool activeState; // Are LEDs active high or active low
} led_t;

led_t led_new(
	uint16_t pin,
	GPIO_TypeDef *port,
	bool activeState);

void led_on (led_t *led);
void led_off (led_t *led);
void led_toggle (led_t *led);



#endif /* INC_LED_H_ */
