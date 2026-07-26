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
	newController.acceleration = 0;
	newController.accelerationMax = 0;
	newController.inputCurrent = 0;
	newController.inputPrevious = 0;
	newController.outputCurrent = 0;
	newController.outputPrevious = 0;
	newController.errorCurrent = 0;
	newController.errorCurrentAbsolute = 0;
	newController.errorPrevious = 0;
	newController.stepCurrent = 0;
	newController.stepPrevious = 0;

	return newController;
}

int32_t controller_step(controller_t *ctrl, int32_t input) {
	// Get timing information
	static uint32_t currentTick = 0;
	currentTick = HAL_GetTick();
	static uint32_t previousTick = 0;
	if(previousTick == 0){
		// If zero, it is the first iteration
		previousTick = currentTick;
	}
	uint32_t deltaTick = currentTick - previousTick;

	// Calculate new output
	ctrl->errorCurrent =  input - ctrl->outputPrevious;
	if(ctrl->errorCurrent < 0) {
		ctrl->errorCurrentAbsolute = -1 * ctrl->errorCurrent;

	} else {
		ctrl->errorCurrentAbsolute = ctrl->errorCurrent;
	}

	if (ctrl->errorCurrentAbsolute > (ctrl->acceleration * deltaTick)) {

		return ctrl->outputCurrent;

	}


	ctrl->outputCurrent = ctrl->outputPrevious + ctrl->stepCurrent;

	return ctrl->outputCurrent;





}
