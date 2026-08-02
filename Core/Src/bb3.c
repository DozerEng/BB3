/*
 * bb3.c
 *
 *  Created on: May 9, 2025
 *      Author: mpill
 */


#include "bb3.h"

bb3_t bb3_new(
		  rgb_t *rgbLeft, rgb_t *rgbRight,
		  rgb_t *rgbEventButton, rgb_t *rgbEventProcess,
		  button_t *pbTop, button_t *pbMid, button_t *pbBot, button_t *pbLimit,
		  tmc2209_t *motorLeft, tmc2209_t *motorRight,
		  bool modeDebug,
		  uint8_t direction,
		  uint8_t speedMode,
		  int32_t setPoint,
		  int32_t speed,
		  controller_base_t *controller
		  ) {
	bb3_t newBb3;

	newBb3.rgbLeft = rgbLeft;
	newBb3.rgbRight = rgbRight;
	newBb3.rgbEventButton = rgbEventButton;
	newBb3.rgbEventProcess = rgbEventProcess;

	newBb3.pbTop = pbTop;
	newBb3.pbMid = pbMid;
	newBb3.pbBot = pbBot;
	newBb3.pbLimit = pbLimit;

	// Motor Configuration
	newBb3.motorLeft = motorLeft;
	newBb3.motorRight = motorRight;

	// Debug mode enables serial logging
	newBb3.modeDebug = modeDebug;

	newBb3.direction = direction;
	newBb3.speedMode = speedMode;
	newBb3.speed = 0;
	newBb3.setPoint = 0;
//	newBb3.acceleration = acceleration;
	newBb3.controller = controller;

	return newBb3;
}


/*
* Intro tasks
*/
void rgb_intro_task(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfSteps) {
	for (uint8_t i = 0; i < numberOfSteps; i++) {
		rgb_cycle(rgb1);
		rgb_reverse_cycle(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off between movements
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}


void rgb_green_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes) {
	for (uint8_t i = 0; i < numberOfFlashes; i++) {
		rgb_set_green(rgb1);
		rgb_set_green(rgb2);

		HAL_Delay(dwellTime);
		rgb_set_off(rgb1);
		rgb_set_off(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off to finish
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}


void rgb_white_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes) {
	for (uint8_t i = 0; i < numberOfFlashes; i++) {
		rgb_set_white(rgb1);
		rgb_set_white(rgb2);

		HAL_Delay(dwellTime);
		rgb_set_off(rgb1);
		rgb_set_off(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off to finish
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}

/*
* Button tasks
*/
void bb3_task_button(
		  uint32_t period,
		  bb3_t *bb3) {

	/*
	 *  Initialize Event Registers
	 */
	static uint32_t previousTick = 0;
	static uint32_t previousEventTick = 0;
	static uint32_t previousButtonPressedTick = 0;

	uint32_t currentTick = HAL_GetTick();
	if(previousTick == 0){
		// If zero, it is the first iteration
		previousTick = currentTick;
	}

	/*
	 * Event
	 */
	if(currentTick > (previousEventTick + period)) {
		// Reset previous event tracker
		previousEventTick = currentTick;


		button_read(bb3->pbTop);
		if (bb3->pbTop->currentState == BUTTON_PRESSED) {
			previousButtonPressedTick = currentTick;
			// Forward increase speed
//			bb3_set_direction(bb3, BB3_DIRECTION_FORWARD);
			bb3_set_speed(bb3, 25000);

			// Set LEDs to signal change
			rgb_set_green(bb3->rgbEventButton);

		}
		button_read(bb3->pbMid);
		if (bb3->pbMid->currentState == BUTTON_PRESSED) {
			previousButtonPressedTick = currentTick;
			// Reverse increase speed
//			bb3_set_direction(bb3, BB3_DIRECTION_REVERSE);
			bb3_set_speed(bb3, 0);

			// Set LEDs to signal change
			rgb_set_turquoise(bb3->rgbEventButton);

		}
		button_read(bb3->pbBot);
		if (bb3->pbBot->currentState == BUTTON_PRESSED) {
			previousButtonPressedTick = currentTick;
			// BRAKE
			bb3_set_direction(bb3, BB3_DIRECTION_BRAKING);
			bb3_set_speed(bb3, -25000);

			// Set LEDs to signal change
			rgb_set_blue(bb3->rgbEventButton);

		}
		button_read(bb3->pbLimit);
		if(bb3->pbLimit->currentState == BUTTON_PRESSED) {
			previousButtonPressedTick = currentTick;
			// BRAKE
			bb3_set_direction(bb3, BB3_DIRECTION_BRAKING);
			bb3_set_speed(bb3, 0);
			// Set LEDs to signal change
			rgb_set_red(bb3->rgbEventButton);
		}




		/*
		 * If there is not button activity, turn off LEDs
		 */
		if(currentTick > (previousButtonPressedTick + TASK_RGB_TIMEOUT)) {
			rgb_set_off(bb3->rgbEventButton);
		}
	}
}

void tmc2209_3_button_task(
	uint32_t period, // in ms
	rgb_t *rgb1,
	rgb_t *rgb2,
	button_t *but1,
	button_t *but2,
	button_t *but3,
	tmc2209_t *tmc1,
	tmc2209_t *tmc2) {

	static uint32_t previousTick = 0;
	static uint32_t previousEventTick = 0;
	static uint32_t previousButtonPressTick = 0;

	uint32_t currentTick = HAL_GetTick();
	if(previousTick == 0){
		// If zero, it is the first time through
		previousTick = currentTick;
	}

	/*
	 * Top PB
	 */
	if(currentTick > (previousEventTick + period)) {
		previousEventTick = currentTick;
		button_read(but1);
		if (but1->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// Velocity Control
			tmc1->vactual += tmc1->acceleration ;
			tmc2209_set_VACTUAL(tmc1);
			tmc2->vactual += tmc1->acceleration ;
			tmc2209_set_VACTUAL(tmc2);

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_OFF) {
				rgb_set_green(rgb1);
			} else {
				rgb_set_off(rgb1);
			}
		}

		/*
		 * Mid PB
		 */
		button_read(but2);
		if (but2->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// Velocity Control
			if (tmc1->mode == TMC2209_VELOCITY_CONTROL) {
				tmc1->vactual -= tmc1->acceleration ;
				tmc2209_set_VACTUAL(tmc1);
			}
			if(tmc2->mode == TMC2209_VELOCITY_CONTROL) {
				tmc2->vactual -= tmc2->acceleration ;
				tmc2209_set_VACTUAL(tmc2);
			}

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_OFF) {
				rgb_set_red(rgb1);
			} else {
				rgb_set_off(rgb1);
			}
		}


		/*
		 * Bot PB - Braking
		 */

		button_read(but3);
		if (but3->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// VACTUAL control
			if((tmc1->vactual < (5* tmc1->acceleration)) && tmc1->vactual > (-5 * tmc1->acceleration)) {
				tmc1->vactual = 0x000000;
			} else {
				if (tmc1->vactual > 0) {
					tmc1->vactual -= 5*tmc1->acceleration;
				} else {
					tmc1->vactual += 5*tmc1->acceleration;
				}
			}
			tmc2209_set_VACTUAL(tmc1);


			// VACTUAL control
			if((tmc2->vactual < (5*tmc2->acceleration)) && (tmc2->vactual > (-5 * tmc2->acceleration ))) {
				tmc2->vactual = 0x000000;
			} else {
				if (tmc2->vactual > 0) {
					tmc2->vactual -= 5*tmc2->acceleration;
				} else {
					tmc2->vactual += 5*tmc2->acceleration;
				}
			}
			tmc2209_set_VACTUAL(tmc2);

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_OFF) {
				rgb_set_blue(rgb1);
			} else {
				rgb_set_off(rgb1);
			}

		}

	}
	// If there is not button activity, turn off LEDs
	if(currentTick > (previousButtonPressTick + 2*period)) {// TASK_RGB_TIMEOUT)) {
		rgb_set_off(rgb1);
	}


}

void tmc2209_icm20608_3_button_task(
	uint32_t period,
	rgb_t *rgb1,
	rgb_t *rgb2,
	button_t *pbTop,
	button_t *pbMid,
	button_t *pbBot,
	icm20608_t *icm,
	tmc2209_t *tmc1,
	tmc2209_t *tmc2) {

	// Top PB
	button_read(pbTop);
	while (pbTop->currentState == pbTop->pressedState) {


		button_read(pbTop);
	}

	// Middle PB
	button_read(pbMid);
	while (pbMid->currentState == pbMid->pressedState) {


		button_read(pbTop);
	}

	// Bottom PB
	button_read(pbBot);
	while (pbBot->currentState == pbBot->pressedState) {


		button_read(pbBot);
	}
}

/*
* Logging tasks
*/
void tmc2209_log_task(uint32_t period, tmc2209_t *tmc) {
	if(LOG_USB == true) {

	}

	if(LOG_UART ==	false) {

	//		  tmc2209_read_request_t readDatagram = {
	//		  			.slaveAddress = TMC2209_ADDR_1,
	//		  			.registerAddress = TMC2209_GCONF
	//		  	};
	//		  uint32_t data = tmc2209_read(&motorRight, readDatagram);
	//		  char msg[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
	//
	//		  uint8_t status;
	//		  status = HAL_UART_Transmit(&huart2, &data, 4, 10);
	//		  HAL_UART_Transmit(&huart1, msg, 6, 10);


	}
}

void icm20608_log_task(uint32_t period, icm20608_t *icm) {
	if(LOG_USB == true) {
		//ToDo: this
		return;
	}

	if(LOG_UART ==	false) {
		//ToDo: this
		return;
	}
}


/*
* Program task(s)
*/
void bb3_task_control_loop(
		uint32_t period,
		bb3_t *bb3
	 ) {
	/*
	 *  Initialize Event Registers
	 */
	static uint32_t previousTick = 0;
	static uint32_t previousEventTick = 0;

	uint32_t currentTick = HAL_GetTick();
	if(previousTick == 0){
		// If zero, it is the first iteration
		previousTick = currentTick;
	}

	/*
	 * Event
	 */
	if(currentTick > (previousEventTick + period)) {
		// Reset previous event tracker
		previousEventTick = currentTick;
		// Update step
//		controller_step(&bb3->ctrl);

		// Update speed
		bb3_update_speed(bb3);
	}
}

/*
* Heartbeat(s)
*/
void bb3_task_heartbeat(
	led_t *led1,
	uint32_t led1Period,
	led_t *led2,
	uint32_t led2Period) {
	static uint32_t led1PreviousEvent = 0;
	static uint32_t led2PreviousEvent = 0;

	uint32_t currentTick = HAL_GetTick();

	// If it's the first run, set PreviousEvent variable to current tick
	if(led1PreviousEvent == 0) {
		led1PreviousEvent = currentTick;
		led2PreviousEvent = currentTick;
	}

	// Toggle system LEDs every period
	if (currentTick > (led1PreviousEvent + led1Period)) {
		led1PreviousEvent = currentTick;
		led_toggle(led1);
	}
	if (currentTick > (led2PreviousEvent + led2Period)) {
		led2PreviousEvent = currentTick;
		led_toggle(led2);
	}

}

/*
 * Helper Functions
 */
void bb3_set_direction(bb3_t *bb3, uint8_t direction) {
	bb3->direction = direction;
}

void bb3_setSpeedMode(bb3_t *bb3, uint8_t speedMode) {
	bb3->speedMode = speedMode;
}

void bb3_set_acceration(bb3_t *bb3, float acceleration) {
	// 1.8 degree steps, 200 steps per rotation
	if(bb3->speedMode == BB3_SPEED_MODE_RPM) {
		return;

	} else if(bb3->speedMode == BB3_SPEED_MODE_RADS) {

		return;
	} else if(bb3->speedMode == BB3_SPEED_MODE_MMS) {
		// Linear motion based on wheel radius * 2 * pi
		return;

	}
}

void bb3_set_speed(bb3_t *bb3,  int32_t speed) {
	// ToDo: Figure out which direction you're set to, and set speed accordingly

	// Adjust vactual based on microstep setting
	bb3->setPoint = speed;



}
void bb3_set_max_speed(bb3_t *bb3, float maxSpeed);

void bb3_update_speed(bb3_t *bb3) {


	// Velocity Control in [µsteps / t]
	bb3->speed = bb3->controller->step(bb3->controller, bb3->setPoint, bb3->speed);

	bb3->motorLeft->vactual = bb3->speed; //bb3->ctrl.outputCurrent;
	tmc2209_set_VACTUAL(bb3->motorLeft);
	bb3->motorRight->vactual = bb3->speed; //bb3->ctrl.outputCurrent;
	tmc2209_set_VACTUAL(bb3->motorRight);


	if((bb3->setPoint >= 0 && bb3->speed < 0) || (bb3->setPoint <= 0 && bb3->speed > 0) ) {
		// If braking, set LED to red
		rgb_set_red(bb3->rgbEventProcess);
	} else if(bb3->setPoint < 0) {
		// If accelerating in reverse, set LED to blue
		rgb_set_blue(bb3->rgbEventProcess);
	} else if (bb3->setPoint > 0 ){
		// If accelerating forward, set LED to green
		rgb_set_green(bb3->rgbEventProcess);
	} else {
		// If at speed, turn of LED
		rgb_set_off(bb3->rgbEventProcess);
	}


}

