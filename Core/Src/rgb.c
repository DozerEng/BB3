/**
 * rgb.c
 *
 *      Author: Michael Pillon
 *
 */

#include "rgb.h"



rgb_t rgb_new(
	uint16_t r_pin,
	GPIO_TypeDef *r_port,
	uint16_t g_pin,
	GPIO_TypeDef *g_port,
	uint16_t b_pin,
	GPIO_TypeDef *b_port,
	rgb_color_t currentColor,
	bool activeState
	) {
	rgb_t newRgb;
	newRgb.r_pin = r_pin;
	newRgb.r_port = r_port;
	newRgb.g_pin = g_pin;
	newRgb.g_port = g_port;
	newRgb.b_pin = b_pin;
	newRgb.b_port = b_port;
	newRgb.currentColor = currentColor;
	newRgb.activeState = activeState;

	rgb_set_color(&newRgb);

	return newRgb;

}

void rgb_set_red(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentColor = RGB_RED;
}

void rgb_set_green(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentColor = RGB_GREEN;
}

void rgb_set_blue(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentColor = RGB_BLUE;
}

void rgb_set_violet(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentColor = RGB_VIOLET;
}

void rgb_set_yellow(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentColor = RGB_YELLOW;
}

void rgb_set_turquoise(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentColor = RGB_TURQUOISE;
}

void rgb_set_white(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentColor = RGB_WHITE;
}

void rgb_set_off(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentColor = RGB_OFF;
}

void rgb_set_color(rgb_t *rgb) {
	switch ( rgb->currentColor ) {
		case RGB_RED:
			rgb_set_red(rgb);
			break;
		case RGB_GREEN:
			rgb_set_green(rgb);
			break;
		case RGB_BLUE:
			rgb_set_blue(rgb);
			break;
		case RGB_VIOLET:
			rgb_set_violet(rgb);
			break;
		case RGB_YELLOW:
			rgb_set_yellow(rgb);
			break;
		case RGB_TURQUOISE:
			rgb_set_turquoise(rgb);
			break;
		case RGB_WHITE:
			rgb_set_white(rgb);
			break;
		case RGB_OFF:
			rgb_set_off(rgb);
			break;
		default:
			fprintf(stderr, "Invalid state RGB_setState: %i\n\r", rgb->currentColor);
		}
}

/**
 * @breif Cycle to next color combo, white exclusive
 * @param RGB
 */
void rgb_cycle(rgb_t *rgb) {
	if( rgb->currentColor >= (RGB_WHITE - 1) ) {
		rgb->currentColor = RGB_RED;
	} else {
		rgb->currentColor ++;
	}
	rgb_set_color(rgb);
}
void rgb_reverse_cycle(rgb_t *rgb) {
	if( rgb->currentColor<= RGB_RED ) {
		rgb->currentColor = (RGB_WHITE - 1);
	} else {
		rgb->currentColor --;
	}
	rgb_set_color(rgb);
}


/**
 * @breif Read from scanf and update RGB LED
 * @param RGB
 */
void rgb_set_user_input(rgb_t *rgb) {
	// Get color from user

	uint8_t STRING_LENGTH = 20;
    char color[STRING_LENGTH];
    printf("Enter color: ");
    scanf("%s", color);
    // Cover to activeStately lower case letters
    char colorLower[STRING_LENGTH];
    for(int i = 0; i < STRING_LENGTH; i++){
    	colorLower[i] = tolower(color[i]);
    }
    // Update RGB LED and
    if ((strcmp(colorLower, "r") == 0) || (strcmp(colorLower, "red") == 0 )) {
		rgb_set_red(rgb);
    } else if ((strcmp(colorLower, "g") == 0) || (strcmp(colorLower, "green") == 0 )) {
		rgb_set_green(rgb);
    } else if ((strcmp(colorLower, "b") == 0) || (strcmp(colorLower, "blue") == 0 )) {
    	rgb_set_blue(rgb);
    } else if ((strcmp(colorLower, "v") == 0) || (strcmp(colorLower, "violet") == 0 )) {
    	rgb_set_violet(rgb);
    } else if ((strcmp(colorLower, "y") == 0) || (strcmp(colorLower, "yellow") == 0 )) {
    	rgb_set_yellow(rgb);
    } else if ((strcmp(colorLower, "t") == 0) || (strcmp(colorLower, "turquoise") == 0 )) {
		rgb_set_turquoise(rgb);
    } else if ((strcmp(colorLower, "w") == 0) || (strcmp(colorLower, "white") == 0 ) || (strcmp(colorLower, "on") == 0 )) {
    	rgb_set_white(rgb);
    } else if (strcmp(colorLower, "off") == 0) {
		rgb_set_off(rgb);
	} else {
		// Invalid color
		printf("Unable to change color, Invalid color: %s\n\r", color);
		return;
	}
    printf("Color changed to %s.\n\r", color);

}

