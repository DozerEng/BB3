/*
 * button.c
 *
 *  Created on: May 10, 2025
 *      Author: Michael Pillon
 */


#include "button.h"





button_t button_new (
	uint16_t pin,
	GPIO_TypeDef *port,
	uint8_t pressedState
	) {

	button_t newButton;
	newButton.pin = pin;
	newButton.port = port;
	newButton.pressedState = pressedState;
	newButton.releasedState = ~pressedState;

	// Get current button state
	button_read(&newButton);

	return newButton;

}


/*
 * Read button
 */

void button_read(button_t *button) {
	 uint32_t pinState = HAL_GPIO_ReadPin(button->port, button->pin);
	 if (((button->pressedState == 1) && (pinState == 1 )) ||
		 ((button->pressedState == 0) && (pinState == 0 ))) {
		 // Button is being pressed
		 button->currentState = BUTTON_PRESSED;
	 } else {
		 // Button is not being pressed
		 button->currentState = BUTTON_RELEASED;
	 }
}

/*
 * Read button and run appropriate handler function
 */

void button_handle(button_t *button) {
	button_read(button);
	if(button->currentState == button->releasedState) {
		button->released_handler();
	} else {
		button->pressed_handler();
	}
}



