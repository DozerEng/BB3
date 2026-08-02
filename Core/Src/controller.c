/*
 * controller.c
 *
 *  Created on: Jul 26, 2026
 *      Author: mpill
 */

#include "controller.h"

controller_t controller_new(int32_t setPoint, int32_t acceleration) {
	controller_t newController;

	newController.setPoint = setPoint;
	newController.acceleration = acceleration;

	newController.base.step = controller_step;

	newController.accelerationMax = 100000;
	newController.inputCurrent = 0;
	newController.inputPrevious = 0;
	newController.outputCurrent = 0;
	newController.outputPrevious = 0;
	newController.errorCurrent = 0;
	newController.errorCurrentAbsolute = 0;
	newController.errorPrevious = 0;

	return newController;
}

int32_t controller_step(controller_base_t *self, int32_t setPoint, int32_t measurement) {
	controller_t *ctrl = (controller_t *)self;

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
	ctrl->errorCurrent =  setPoint - ctrl->outputPrevious;
	// Check if we are already at desired set point
	if(ctrl->errorCurrent == 0) {
		return ctrl->outputPrevious;
	}

	// Compare error to max acceleration change for this time interval
	int32_t maxVelocityChange = ctrl->acceleration * deltaTick;
	if (ctrl->errorCurrent > maxVelocityChange) {
		// Error is greater than can be achieved with the maximum acceleration over this time step
		ctrl->outputCurrent = ctrl->outputPrevious + maxVelocityChange;
	} else if (ctrl->errorCurrent < (-1*maxVelocityChange)) {
		// Error is greater than can be achieved with the maximum acceleration over this time step
		ctrl->outputCurrent = ctrl->outputPrevious - maxVelocityChange;
	} else {
		// Error is not limited by acceleration, set to desired speed
		ctrl->outputCurrent = ctrl->outputPrevious + ctrl->errorCurrent;
	}
	return ctrl->outputCurrent;

}
