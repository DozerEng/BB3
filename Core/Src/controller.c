/*
 * controller.c
 *
 *  Created on: Jul 26, 2026
 *      Author: mpill
 */

#include "controller.h"

controller_t controller_new(int32_t setPoint) {
	controller_t newController;
	newController.setPoint = setPoint;
	newController.acceleration = 5;
	newController.accelerationMax = 100;
	newController.inputCurrent = 0;
	newController.inputPrevious = 0;
	newController.outputCurrent = 0;
	newController.outputPrevious = 0;
	newController.errorCurrent = 0;
	newController.errorCurrentAbsolute = 0;
	newController.errorPrevious = 0;
//	newController.stepCurrent = 0;
//	newController.stepPrevious = 0;

	return newController;
}

void controller_step(controller_t *ctrl) {
	// Get timing information
	static uint32_t currentTick = 0;
	currentTick = HAL_GetTick();
	static uint32_t previousTick = 0;
	if(previousTick == 0){
		// If zero, it is the first iteration
		previousTick = currentTick;
	}
	// Get time since last tick
	uint32_t deltaTick = (currentTick - previousTick);
	// Update previous tick
	previousTick = currentTick;

	// Update previous values
	ctrl->outputPrevious = ctrl->outputCurrent;
	ctrl->errorPrevious = ctrl->errorCurrent;
	// Calculate error
	ctrl->errorCurrent =  ctrl->setPoint - ctrl->outputPrevious;
	// Check if we are already at desired set point
	if(ctrl->errorCurrent == 0) {
		return;
	}


//	// Get absolute error
//	if(ctrl->errorCurrent < 0) {
//		ctrl->errorCurrentAbsolute = -1 * ctrl->errorCurrent;
//
//	} else {
//		ctrl->errorCurrentAbsolute = ctrl->errorCurrent;
//	}
	// Compare error to max acceleration
	int32_t maxVelocityChange = ctrl->acceleration * deltaTick;
	if (ctrl->errorCurrent > maxVelocityChange) {
		// Error is greater than can be achieved with the maximum acceleration over this time step
		ctrl->outputCurrent = ctrl->outputPrevious + maxVelocityChange;
	} else if (ctrl->errorCurrent < (-1*maxVelocityChange)) {
		ctrl->outputCurrent = ctrl->outputPrevious - maxVelocityChange;
	} else {
		// Error is not limited by acceleration, set to desired speed
		ctrl->outputCurrent = ctrl->outputPrevious + ctrl->errorCurrent;
	}


}
