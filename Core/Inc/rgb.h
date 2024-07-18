/**
 * rgb.h
 *
 *      Author: Michael Pillon
 *
 *      For discrete control of the red, green, and blue
 *      LEDs in an RGB LED.
 *      	- PWM not currently supported.
 *
 */

#ifndef INC_RGB_H_
#define INC_RGB_H_

#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "stdio.h"
#include "ctype.h"
#include "string.h"

#include "stdbool.h"

//
//#define RGB_LED_ON		1
//#define RGB_LED_OFF 	0


/**
 * RGB Data types
 */

/** Connection type is logic signal to turn on the LED */
typedef enum {
	RGB_OFF = 0,
	RGB_RED = 1,
	RGB_YELLOW = 2,
	RGB_GREEN = 3,
	RGB_TURQUOISE = 4,
	RGB_BLUE = 5,
	RGB_VIOLET = 6,
	RGB_WHITE = 7
} rgb_state_t ;

typedef struct {
	uint16_t r_pin;
	GPIO_TypeDef *r_port;
	uint16_t g_pin;
	GPIO_TypeDef *g_port;
	uint16_t b_pin;
	GPIO_TypeDef *b_port;
	rgb_state_t currentState;
	bool activeState; //Active high or active low
} rgb_t;

void RGB_setRed(rgb_t *rgb);
void RGB_setGreen(rgb_t *rgb);
void RGB_setBlue(rgb_t *rgb);
void RGB_setViolet(rgb_t *rgb);
void RGB_setYellow(rgb_t *rgb);
void RGB_setTurquoise(rgb_t *rgb);
void RGB_setWhite(rgb_t *rgb);
void RGB_setOff(rgb_t *rgb);

void RGB_setState(rgb_t *rgb);
void RGB_cycle(rgb_t *rgb);
void RGB_setUserInput(rgb_t *rgb);



#endif /* INC_RGB_H_ */
