/**
 * button.h
 *
 *  Author: Michael Pillon
 *
 *      Library for using push buttons, or really anything that has a high and low state.
 *
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_


#include "stm32g4xx_hal.h"
#include "stdint.h"


/*
 * Data types
 */

#define BUTTON_RELEASED 	0
#define BUTTON_PRESSED 		1
//typedef enum {
//	RELEASED,
//	PRESSED
//} buttonState_t;

typedef struct {
	// pins
	uint16_t pin;
	GPIO_TypeDef *port;
	// states
	uint8_t currentState;
	uint8_t pressedState;
	uint8_t releasedState;

	void (*pressed_handler)(void);
	void (*released_handler)(void);


} button_t;

/*
 * Functions
 */


button_t button_new (
	uint16_t pin,
	GPIO_TypeDef *port,
	uint8_t pressedState
);

void button_read(button_t *button);
void button_handle(button_t *button);


#endif /* INC_BUTTON_H_ */
