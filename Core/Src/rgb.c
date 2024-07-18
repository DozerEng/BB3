/**
 * rgb.c
 *
 *      Author: Michael Pillon
 *
 */

#include "rgb.h"

/**
 * Violet = red + blue
 * Yellow = red + green
 * Turquoise = green + blue
 * White = red + green + blue
 * Off =
 *
*/
void RGB_setRed(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentState = RGB_RED;
}

void RGB_setGreen(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentState = RGB_GREEN;
}

void RGB_setBlue(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentState = RGB_BLUE;
}

void RGB_setViolet(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentState = RGB_VIOLET;
}

void RGB_setYellow(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentState = RGB_YELLOW;
}

void RGB_setTurquoise(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentState = RGB_TURQUOISE;
}

void RGB_setWhite(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, rgb->activeState);
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, rgb->activeState);
	rgb->currentState = RGB_WHITE;
}

void RGB_setOff(rgb_t *rgb) {
	HAL_GPIO_WritePin(rgb->r_port, rgb->r_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->g_port, rgb->g_pin, !(rgb->activeState));
	HAL_GPIO_WritePin(rgb->b_port, rgb->b_pin, !(rgb->activeState));
	rgb->currentState = RGB_OFF;
}

void RGB_setState(rgb_t *rgb) {
	switch ( rgb->currentState ) {
		case RGB_RED:
			RGB_setRed(rgb);
			break;
		case RGB_GREEN:
			RGB_setGreen(rgb);
			break;
		case RGB_BLUE:
			RGB_setBlue(rgb);
			break;
		case RGB_VIOLET:
			RGB_setViolet(rgb);
			break;
		case RGB_YELLOW:
			RGB_setYellow(rgb);
			break;
		case RGB_TURQUOISE:
			RGB_setTurquoise(rgb);
			break;
		case RGB_WHITE:
			RGB_setWhite(rgb);
			break;
		case RGB_OFF:
			RGB_setOff(rgb);
			break;
		default:
			fprintf(stderr, "Invalid state RGB_setState: %i\n\r", rgb->currentState);
		}
}

/**
 * @breif Cycle to next color combo, white exclusive
 * @param RGB
 */
void RGB_cycle(rgb_t *rgb) {
	if( rgb->currentState >= (RGB_WHITE - 1) ) {
		rgb->currentState = RGB_RED;
	} else {
		rgb->currentState ++;
	}
	RGB_setState(rgb);
}
void RGB_reverseCycle(rgb_t *rgb) {
	if( rgb->currentState <= RGB_RED ) {
		rgb->currentState = (RGB_WHITE - 1);
	} else {
		rgb->currentState --;
	}
	RGB_setState(rgb);
}


/**
 * @breif Read from scanf and update RGB LED
 * @param RGB
 */
void RGB_setUserInput(rgb_t *rgb) {
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
		RGB_setRed(rgb);
    } else if ((strcmp(colorLower, "g") == 0) || (strcmp(colorLower, "green") == 0 )) {
		RGB_setGreen(rgb);
    } else if ((strcmp(colorLower, "b") == 0) || (strcmp(colorLower, "blue") == 0 )) {
    	RGB_setBlue(rgb);
    } else if ((strcmp(colorLower, "v") == 0) || (strcmp(colorLower, "violet") == 0 )) {
    	RGB_setViolet(rgb);
    } else if ((strcmp(colorLower, "y") == 0) || (strcmp(colorLower, "yellow") == 0 )) {
    	RGB_setYellow(rgb);
    } else if ((strcmp(colorLower, "t") == 0) || (strcmp(colorLower, "turquoise") == 0 )) {
		RGB_setTurquoise(rgb);
    } else if ((strcmp(colorLower, "w") == 0) || (strcmp(colorLower, "white") == 0 ) || (strcmp(colorLower, "on") == 0 )) {
    	RGB_setWhite(rgb);
    } else if (strcmp(colorLower, "off") == 0) {
		RGB_setOff(rgb);
	} else {
		// Invalid color
		printf("Unable to change color, Invalid color: %s\n\r", color);
		return;
	}
    printf("Color changed to %s.\n\r", color);

}

