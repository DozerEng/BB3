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

/**
 * Data types
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
} rgb_color_t ;

typedef struct {
	uint16_t r_pin;
	GPIO_TypeDef *r_port;
	uint16_t g_pin;
	GPIO_TypeDef *g_port;
	uint16_t b_pin;
	GPIO_TypeDef *b_port;
	rgb_color_t currentColor;
	bool activeState; // Are LEDs active high or active low
} rgb_t;

/*
 * Functions
 */
rgb_t rgb_new(
	uint16_t r_pin,
	GPIO_TypeDef *r_port,
	uint16_t g_pin,
	GPIO_TypeDef *g_port,
	uint16_t b_pin,
	GPIO_TypeDef *b_port,
	rgb_color_t currentColor,
	bool activeState
	);
void rgb_set_red(rgb_t *rgb);
void rgb_set_green(rgb_t *rgb);
void rgb_set_blue(rgb_t *rgb);
void rgb_set_violet(rgb_t *rgb);
void rgb_set_yellow(rgb_t *rgb);
void rgb_set_turquoise(rgb_t *rgb);
void rgb_set_white(rgb_t *rgb);
void rgb_set_off(rgb_t *rgb);

void rgb_set_color(rgb_t *rgb);

void rgb_cycle(rgb_t *rgb);
void rgb_reverse_cycle(rgb_t *rgb);

void rgb_set_user_input(rgb_t *rgb);



#endif /* INC_rgb_H_ */
