/*
 * eezybotarn.c
 *
 *      Author: Michael Pillon
 *
 */

#include "eezybotarm.h"

/**
 * @function eezybotarm_createNew
 * @brief Create, initialize, and return a new eezybotarm
 */

eezybotarm_t eezybotarm_new(servo_t* tool, servo_t* elbow, servo_t* shoulder, servo_t* base, rgb_t* eventRGB, rgb_t* modeRGB) {

	// Pause for dramatic effect... and to wait for USB to be ready again
	HAL_Delay(EEZYBOTARM_LOADING_DELAY);
	// Create buffer and send initialization message
	char* initializingMessage1 = "\n\rInitializing eezybotarm MK2   ";
	char* initializingMessage2 = "\rInitializing eezybotarm MK2.  ";
	char* initializingMessage3 = "\rInitializing eezybotarm MK2.. ";
	char* initializingMessage4 = "\rInitializing eezybotarm MK2...";
	CDC_Transmit_FS((uint8_t*)initializingMessage1, strlen(initializingMessage1));
	HAL_Delay(EEZYBOTARM_LOADING_DELAY);
	for(uint8_t i = 0; i < EEZYBOTARM_DRAMATIC_EFFECT_COUNT; i++) {
		CDC_Transmit_FS((uint8_t*)initializingMessage2, strlen(initializingMessage2));
		HAL_Delay(EEZYBOTARM_LOADING_DELAY * EEZYBOTARM_DRAMATIC_EFFECT_FACTOR);
		CDC_Transmit_FS((uint8_t*)initializingMessage3, strlen(initializingMessage3));
		HAL_Delay(EEZYBOTARM_LOADING_DELAY * EEZYBOTARM_DRAMATIC_EFFECT_FACTOR);
		CDC_Transmit_FS((uint8_t*)initializingMessage4, strlen(initializingMessage4));
		HAL_Delay(EEZYBOTARM_LOADING_DELAY * EEZYBOTARM_DRAMATIC_EFFECT_FACTOR);

	}
	CDC_Transmit_FS((uint8_t*)"\n\r", 2);
	HAL_Delay(EEZYBOTARM_LOADING_DELAY);

	eezybotarm_t newRobotArm = {
		.tool = tool,
		.elbow = elbow,
		.shoulder = shoulder,
		.base = base,
		.toolMin = EEZYBOTARM_TOOL_MIN,
		.toolMax = EEZYBOTARM_TOOL_MAX,
		.elbowMin = EEZYBOTARM_ELBOW_MIN,
		.elbowMax = EEZYBOTARM_ELBOW_MAX,
		.shoulderMin = EEZYBOTARM_SHOULDER_MIN,
		.shoulderMax = EEZYBOTARM_SHOULDER_MAX,
		.baseMin = EEZYBOTARM_BASE_MIN,
		.baseMax = EEZYBOTARM_BASE_MAX,
		.modeRGB = modeRGB,
		.eventRGB = eventRGB,
		.eventTicksRemaining = 0,
		.mode = eezybotarm_NORMAL,
		.jogStepSize = EEZYBOTARM_NORMAL_JOG
	};
	// Turn on and move the robot to the home position
	eezybotarm_setMode(&newRobotArm, eezybotarm_NORMAL);
	eezybotarm_home(&newRobotArm);
	eezybotarm_on(&newRobotArm);
	HAL_Delay(EEZYBOTARM_LOADING_DELAY); // Delay for message sent in eezybotarm_on()
	char* successMessage = "eezybotarm MK2 successfully initialized\n\r";
	CDC_Transmit_FS((uint8_t*)successMessage, strlen(successMessage));
	HAL_Delay(EEZYBOTARM_LOADING_DELAY);
	return newRobotArm;
}

/**
 * @function eezybotarm_on
 * @brief Turns on all of the servos
 *
 */

void eezybotarm_on(eezybotarm_t* eezy) {
	Servo_start(eezy->tool);
	Servo_start(eezy->elbow);
	Servo_start(eezy->shoulder);
	Servo_start(eezy->base);
}

/**
 * @function eezybotarm_off
 * @brief Turns off all of the servos
 *
 */

void eezybotarm_off(eezybotarm_t* eezy) {
	Servo_stop(eezy->tool);
	Servo_stop(eezy->elbow);
	Servo_stop(eezy->shoulder);
	Servo_stop(eezy->base);

}



/**
 * @function eezybotarm_update_qawsedrf
 * @brief Move arm using the aqwsedrf keys
 *
 */
void eezybotarm_update_qawsedrf(eezybotarm_t* eezy) {
	char command = (char) CDC_Get_Buffer()[0];
	command = tolower(command);
	switch(command){
	// Moving base
	case 'q':
		eezybotarm_setBase(eezy, eezy->base->setPoint + eezy->jogStepSize);
		break;
	case 'a':
		eezybotarm_setBase(eezy, eezy->base->setPoint - eezy->jogStepSize);
		break;
	// Moving shoulder
	case 'w':
		eezybotarm_setShoulder(eezy, eezy->shoulder->setPoint + eezy->jogStepSize);
		break;
	case 's':
		eezybotarm_setShoulder(eezy, eezy->shoulder->setPoint - eezy->jogStepSize);
		break;
	// Moving elbow
	case 'e':
		eezybotarm_setElbow(eezy, eezy->elbow->setPoint + eezy->jogStepSize);
		break;
	case 'd':
		eezybotarm_setElbow(eezy, eezy->elbow->setPoint - eezy->jogStepSize);
		break;
	// Moving tool
	case 't':
		eezybotarm_setTool(eezy, eezy->tool->setPoint + eezy->jogStepSize);
		break;
	case 'g':
		eezybotarm_setTool(eezy, eezy->tool->setPoint - eezy->jogStepSize);
		break;
	case 'r':
		eezybotarm_openTool(eezy);
		break;
	case 'f':
		eezybotarm_closeTool(eezy);
		break;
	// Move to location
	case 'h':
		eezybotarm_home(eezy);
		break;
	// Set mode
	case '0':
		eezybotarm_setMode(eezy, eezybotarm_OFF);
		return;
	case '1':
		eezybotarm_setMode(eezy, eezybotarm_SLOWEST);
		return;
	case '2':
		eezybotarm_setMode(eezy, eezybotarm_SLOW);
		return;
	case '3':
		eezybotarm_setMode(eezy, eezybotarm_NORMAL);
		return;
	case '4':
		eezybotarm_setMode(eezy, eezybotarm_FAST);
		return;
	case '5':
		eezybotarm_setMode(eezy, eezybotarm_FASTEST);
		return;
	case 'p':
		eezybotarm_runProgram(eezy);
		return;
	case '\r':
		// Allow the user to create spaces in the terminal to segment readings
		char *newLine = "\n\r";
		CDC_Transmit_FS((uint8_t*)newLine, 2);
	default:
		// Must call a no event to turn off LED after a finite duration
		eezybotarm_event(eezy, eezybotarm_NO_EVENT);
		return;
	}

	// Print new positions to terminal
	char updatedPositions[70];
	sprintf(updatedPositions, "Base: %0.3f  Shoulder: %0.3f  Elbow: %0.3f  Tool: %0.3f\n\r",
			eezy->base->setPoint,
			eezy->shoulder->setPoint,
			eezy->elbow->setPoint,
			eezy->tool->setPoint);
	// Print over USB
	CDC_Transmit_FS((uint8_t*)updatedPositions, strlen(updatedPositions));

}

/**
 * @function eezybotarm_setTool
 * @brief Set tool position as percentage of maximum range of motion
 *
 */
void eezybotarm_setTool(eezybotarm_t* eezy, double newPosition) {
	if(newPosition >= eezy->toolMax) {
		newPosition = eezy->toolMax;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else if (newPosition <= eezy->toolMin) {
		newPosition = eezy->toolMin;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else {
		eezybotarm_event(eezy, eezybotarm_TOOL_EVENT);
	}
	Servo_setPosition(eezy->tool, newPosition);
}
/**
 * @function eezybotarm_setElbow
 * @brief Set elbow position as percentage of maximum range of motion
 *
 */
void eezybotarm_setElbow(eezybotarm_t* eezy, double newPosition) {
	if(newPosition >= eezy->elbowMax) {
		newPosition = eezy->elbowMax;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else if (newPosition <= eezy->elbowMin) {
		newPosition = eezy->elbowMin;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else {
		eezybotarm_event(eezy, eezybotarm_ELBOW_EVENT);
	}
	Servo_setPosition(eezy->elbow, newPosition);

}
/**
 * @function eezybotarm_setShoulder
 * @brief Set shoulder position as percentage of maximum range of motion
 *
 */
void eezybotarm_setShoulder(eezybotarm_t* eezy, double newPosition) {
	if(newPosition >= eezy->shoulderMax) {
		newPosition = eezy->shoulderMax;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else if (newPosition <= eezy->shoulderMin) {
		newPosition = eezy->shoulderMin;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else {
		eezybotarm_event(eezy, eezybotarm_SHOULDER_EVENT);
	}
	Servo_setPosition(eezy->shoulder, newPosition);
}
/**
 * @function eezybotarm_setBase
 * @brief Set base position as percentage of maximum range of motion
 *
 */
void eezybotarm_setBase(eezybotarm_t* eezy, double newPosition) {
	if(newPosition >= eezy->baseMax) {
		newPosition = eezy->baseMax;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else if (newPosition <= eezy->baseMin) {
		newPosition = eezy->baseMin;
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
	} else {
		eezybotarm_event(eezy, eezybotarm_BASE_EVENT);
	}
	Servo_setPosition(eezy->base, newPosition);

}

/**
 * @function eezybotarm_openTool
 * @brief Set tool to open position
 *
 */
void eezybotarm_openTool(eezybotarm_t* eezy) {
	Servo_setPosition(eezy->tool, EEZYBOTARM_TOOL_OPEN);
	eezybotarm_event(eezy, eezybotarm_TOOL_EVENT);
}

/**
 * @function eezybotarm_closeTool
 * @brief Set tool to closed position
 *
 */
void eezybotarm_closeTool(eezybotarm_t* eezy) {
	Servo_setPosition(eezy->tool, EEZYBOTARM_TOOL_CLOSED);
	eezybotarm_event(eezy, eezybotarm_TOOL_EVENT);
}

/**
 * @function eezybotarm_home
 * @brief Set all joints to their home positions
 *
 */
void eezybotarm_home(eezybotarm_t* eezy) {
	eezybotarm_coordinate_t home = {
			.toolCoordinate = EEZYBOTARM_TOOL_HOME,
			.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
			.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
			.baseCoordinate = EEZYBOTARM_BASE_HOME
	};
	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);
}

/**
 * @function eezybotarm_setMode
 * @brief Updates the mode variable and appropriate RGB LED
 *
 */

void eezybotarm_setMode(eezybotarm_t* eezy, eezybotarm_mode_t mode) {

	// Check if a special mode value is passed
	if(mode == eezybotarm_MODE_INCREMENT) {
		if(eezy->mode >= eezybotarm_FASTEST) {
			mode = eezybotarm_FASTEST;
		} else {
			mode = eezy->mode + 1;
		}
	} else if (mode == eezybotarm_MODE_DECREMENT) {
		if(eezy->mode <= eezybotarm_SLOWEST) {
			mode = eezybotarm_SLOWEST;
		} else {
			mode = eezy->mode - 1;
		}
	}

	// Check if servos are off and if they should be turned back on
	if((eezy->mode == eezybotarm_OFF) && (mode != eezybotarm_OFF)) {
		eezybotarm_on(eezy);
	}

	// Create update message buffer
//	uint8_t updateMessageBufferSize = 60;
	char* updateMessage;

	// Configure mode
	switch(mode) {
	case eezybotarm_OFF:
		RGB_setOff(eezy->modeRGB);
		eezy->jogStepSize = 0.0;
		// Actually turn off servos
		eezybotarm_off(eezy);
		updateMessage = "Changed mode to off\n\r";
		break;
	case eezybotarm_SLOWEST:
		RGB_setBlue(eezy->modeRGB);
		eezy->jogStepSize = EEZYBOTARM_SLOWEST_JOG;
		updateMessage = "Changed mode to slowest\n\r";
		break;
	case eezybotarm_SLOW:
		RGB_setTurquoise(eezy->modeRGB);
		eezy->jogStepSize = EEZYBOTARM_SLOW_JOG;
		updateMessage = "Changed mode to slow\n\r";
		break;
	case eezybotarm_NORMAL:
		RGB_setWhite(eezy->modeRGB);
		eezy->jogStepSize = EEZYBOTARM_NORMAL_JOG;
		updateMessage = "Changed mode to normal\n\r";
		break;
	case eezybotarm_FAST:
		RGB_setGreen(eezy->modeRGB);
		eezy->jogStepSize = EEZYBOTARM_FAST_JOG;
		updateMessage = "Changed mode to fast\n\r";
		break;
	case eezybotarm_FASTEST:
		RGB_setYellow(eezy->modeRGB);
		eezy->jogStepSize = EEZYBOTARM_FASTEST_JOG;
		updateMessage = "Changed mode to fastest\n\r";
		break;
	case eezybotarm_FAULT:
		RGB_setRed(eezy->modeRGB);
		eezy->jogStepSize = 0.0;
		// Turn off servos for safety
		eezybotarm_off(eezy);
		updateMessage = "FAULT!!!\n\rEVERYBODY RUN FOR YOUR LIVES\n\r";
		break;
	default:
		// Clear update message buffer and fill with error message
		updateMessage = "Error: eezybotarm_setMode - Invalid mode index\n\r";
		break;
	}

	// Send update message over USB
	CDC_Transmit_FS((uint8_t*)updateMessage, strlen(updateMessage));
	// Update mode register value
	eezy->mode = mode;
	eezybotarm_event(eezy, eezybotarm_CONFIG_EVENT);
}

/**
 * @function eezybotarm_event
 * @brief Updates the event RGB LED
 *
 */

void eezybotarm_event(eezybotarm_t* eezy, eezybotarm_event_t eventType){
	// Set LED based on event type
	switch(eventType) {
	case eezybotarm_NO_EVENT:
		if(eezy->eventTicksRemaining <= 0) {
			RGB_setOff(eezy->eventRGB);
		} else {
			eezy->eventTicksRemaining = eezy->eventTicksRemaining - 1;
		}
		// Return so the count isn't reset
		return;
	case eezybotarm_TOOL_EVENT:
		RGB_setGreen(eezy->eventRGB);
		break;
	case eezybotarm_ELBOW_EVENT:
		RGB_setBlue(eezy->eventRGB);
		break;
	case eezybotarm_SHOULDER_EVENT:
		RGB_setViolet(eezy->eventRGB);
		break;
	case eezybotarm_BASE_EVENT:
		RGB_setYellow(eezy->eventRGB);
		break;
	case eezybotarm_MOVE_EVENT:
		RGB_setTurquoise(eezy->eventRGB);
		break;
	case eezybotarm_CONFIG_EVENT:
		RGB_setWhite(eezy->eventRGB);
		break;
	case eezybotarm_ERROR_EVENT:
		RGB_setRed(eezy->eventRGB);
		break;
	}
	// Reset event time remaining counter
	eezy->eventTicksRemaining = EEZYBOTARM_EVENT_LENGTH;
}

/**
 * @function eezybotarm_move
 * @brief Move robot arm to specified location based on joint positions
 * @param destination New setpoint
 * @param duration Time in ms to spread out movement
 */

void eezybotarm_move(eezybotarm_t* eezy, eezybotarm_coordinate_t destination, uint16_t duration) {
	// If arm is not in a valid movement state, return
	if ((eezy->mode < eezybotarm_SLOWEST) || (eezy->mode > eezybotarm_FASTEST) ) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	// Set off event LED
	eezybotarm_event(eezy, eezybotarm_MOVE_EVENT);
	// Solve total movement distances for each axis
	double toolDisplacement, elbowDisplacement, shoulderDisplacement, baseDisplacement;
	toolDisplacement = destination.toolCoordinate - eezy->tool->setPoint;
	elbowDisplacement = destination.elbowCoordinate - eezy->elbow->setPoint;
	shoulderDisplacement = destination.shoulderCoordinate - eezy->shoulder->setPoint;
	baseDisplacement = destination.baseCoordinate - eezy->base->setPoint;

	// Get current tick count and solve final tick count using duration
	const uint32_t startTime = HAL_GetTick(); // in ms
	const uint32_t endTime = startTime + duration;
	uint32_t currentTime = HAL_GetTick();
	// Move to destination using linear interpolation over the duration
	while(currentTime < endTime) {
		// Calculate current position
		double toolPosition = destination.toolCoordinate - toolDisplacement * (double)(endTime - currentTime) / (double)duration;
		double elbowPosition = destination.elbowCoordinate - elbowDisplacement * (double)(endTime - currentTime) / (double)duration;
		double shoulderPosition = destination.shoulderCoordinate - shoulderDisplacement * (double)(endTime - currentTime) / (double)duration;
		double basePosition = destination.baseCoordinate - baseDisplacement * (double)(endTime - currentTime) / (double)duration;
		// Move to current Position
		eezybotarm_setTool(eezy, toolPosition);
		eezybotarm_setElbow(eezy, elbowPosition);
		eezybotarm_setShoulder(eezy, shoulderPosition);
		eezybotarm_setBase(eezy, basePosition);
		// Get time
		currentTime = HAL_GetTick();
		// Set off event LED so it lasts past the duration of the movement
		eezybotarm_event(eezy, eezybotarm_MOVE_EVENT);
	}
	// Ensure final position is the destination
	eezybotarm_setTool(eezy, destination.toolCoordinate);
	eezybotarm_setElbow(eezy, destination.elbowCoordinate);
	eezybotarm_setShoulder(eezy, destination.shoulderCoordinate);
	eezybotarm_setBase(eezy, destination.baseCoordinate);
}

/**
 * @function eezybotarm_runProgram
 * @brief Prints a menu of programs and a prompt for a user to pick a program using 0-9
 *
 */

void eezybotarm_runProgram(eezybotarm_t* eezy) {
	if (EEZYBOTARM_INACTIVE_CHECK) {
		const char* errorMessage = "Unable to run programs while arm is inactive\n\r";
		CDC_Transmit_FS((uint8_t*)errorMessage, strlen(errorMessage));
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	const uint8_t PRINT_DELAY = 50; // To allow USB buffers to empty
	/*
	 * List of programs
	 * 	- Should be updated to include all programs you want available to the user
	 * 	- The second last item in the list should be to quit the program menu
	 * 	- The last item in the list should be NULL to signify the end of the list
	 * 	- Maximum of 9 programs + quit
	 * 		- Could be improved with support for getting multiple characters from user
	 */

	const char* eezybotarm_programList[] = {
		"\t1 - Range of motion test 1\n\r",
		"\t2 - Range of motion test 2\n\r",
		"\t3 - Towers of Hanoi setup\n\r",
		"\t4 - Towers of Hanoi\n\r",
		"\t5 - Dog treat feeder setup\n\r",
		"\t6 - Dog treat feeder\n\r",
		"\t7 - \n\r",
		"\t8 - \n\r",
		"\t9 - \n\r",
		"\tQ - Quit\n\r",
		NULL
	};

	/*
	 *  Introductory prompt
	 */
	const char* introMessage = "Select one of the following programs:\n\r";
	CDC_Transmit_FS((uint8_t*)introMessage, strlen(introMessage));
	HAL_Delay(PRINT_DELAY);

	/*
	 *	Print list of programs
	 */
	uint8_t i = 0;
	while( eezybotarm_programList[i] != NULL) {
		CDC_Transmit_FS((uint8_t*)eezybotarm_programList[i], strlen(eezybotarm_programList[i]));
		i++;
		HAL_Delay(PRINT_DELAY);
	}

	/*
	 *	Prompt user for input character, convert to lower case,
	 *	run program or exit based on the users selection
	 */
	bool validInput = false;
	while (validInput == false) {
		char programIndex = (char)CDC_Get_Char();
		programIndex = tolower(programIndex);
		/*
		 * Make sure the following switch statement matches eezybotarm_programList[]
		 */
		char* message;
		switch(programIndex) {
		case '1':
			eezybotarm_rangeOfMotionTest1(eezy);
			validInput = true;
			break;
		case '2':
			eezybotarm_rangeOfMotionTest2(eezy);
			validInput = true;
			break;
		case '3':
			eezybotarm_towersOfHanoi_setup(eezy);
			validInput = true;
			break;
		case '4':
			eezybotarm_towersOfHanoi(eezy);
			validInput = true;
			break;
		case '5':
			eezybotarm_dogTreatFeeder_setup(eezy);
			validInput = true;
			break;
		case '6':
			eezybotarm_dogTreatFeeder(eezy);
			validInput = true;
			break;
		case '7':
			message = "Program 7 does not exist\n\r";
			CDC_Transmit_FS((uint8_t*)message, strlen(message));
			//validInput = true;
			break;
		case '8':
			message = "Program 8 does not exist\n\r";
			CDC_Transmit_FS((uint8_t*)message, strlen(message));
			//validInput = true;
			break;
		case '9':
			message = "Program 9 does not exist\n\r";
			CDC_Transmit_FS((uint8_t*)message, strlen(message));
			//validInput = true;
			break;
		case 'q':
			message = "Leaving program menu\n\r";
			CDC_Transmit_FS((uint8_t*)message, strlen(message));
			validInput = true;
			break;
		}
	}
}

/**
 * @function eezybotarm_rangeOfMotionTest1
 * @brief Moves robot arm through full range of motion in both directions and then goes home
 *
 */
void eezybotarm_rangeOfMotionTest1(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
	if (EEZYBOTARM_INACTIVE_CHECK) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	char* message = "Running range of motion test program\n\r";
	CDC_Transmit_FS((uint8_t*)message, strlen(message));

	/*
	 * List of coordinates
	 */
	eezybotarm_coordinate_t home = {
		.toolCoordinate = EEZYBOTARM_TOOL_HOME,
		.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
		.baseCoordinate = EEZYBOTARM_BASE_HOME
	};
	eezybotarm_coordinate_t step1 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MAX,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MIN,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_MAX,
		.baseCoordinate = EEZYBOTARM_BASE_MAX
	};

	eezybotarm_coordinate_t step2 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MIN,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MAX,
		.shoulderCoordinate = 0.5*(EEZYBOTARM_SHOULDER_MAX + EEZYBOTARM_SHOULDER_MIN),
		.baseCoordinate = EEZYBOTARM_BASE_MAX
	};

	eezybotarm_coordinate_t step3 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MAX,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MAX,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_MIN,
		.baseCoordinate = EEZYBOTARM_BASE_MAX
	};

	eezybotarm_coordinate_t step4 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MIN,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MAX,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_MIN,
		.baseCoordinate = EEZYBOTARM_BASE_MIN
	};

	eezybotarm_coordinate_t step5 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MAX,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MAX,
		.shoulderCoordinate = 0.5*(EEZYBOTARM_SHOULDER_MAX + EEZYBOTARM_SHOULDER_MIN),
		.baseCoordinate = EEZYBOTARM_BASE_MIN
	};

	eezybotarm_coordinate_t step6 = {
		.toolCoordinate = EEZYBOTARM_TOOL_MIN,
		.elbowCoordinate = EEZYBOTARM_ELBOW_MIN,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_MAX,
		.baseCoordinate = EEZYBOTARM_BASE_MIN
	};

	/*
	 * Run program
	 */
	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step1, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step2, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step3, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step4, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step5, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step6, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step1, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step6, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step5, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step4, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step3, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step2, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step1, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step6, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, step1, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	eezybotarm_move(eezy, home, delay);
	EEZYBOTARM_PROGRAM_COMPLETED
}

/**
 * @function eezybotarm_rangeOfMotionTest2
 * @brief Moves robot arm through a range of motion where all axis move in every movement
 *
 */
void eezybotarm_rangeOfMotionTest2(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
	if (EEZYBOTARM_INACTIVE_CHECK) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	char* message = "Running range of motion test 2 program\n\r";
	CDC_Transmit_FS((uint8_t*)message, strlen(message));

	/*
	 * List of coordinates
	 */
	eezybotarm_coordinate_t home = {
		.toolCoordinate = EEZYBOTARM_TOOL_HOME,
		.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
		.baseCoordinate = EEZYBOTARM_BASE_HOME
	};

	/*
	 * Run program
	 */
	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	EEZYBOTARM_PROGRAM_COMPLETED
}

/**
 * @function eezybotarm_towersOfHanoi_setup
 * @brief Moves robot arm through all 3 locations so the blocks can be set up
 *
 */
void eezybotarm_towersOfHanoi_setup(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
	if (EEZYBOTARM_INACTIVE_CHECK) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	char* introMessage = "Beginning Towers of Hanoi setup program\n\r";
	CDC_Transmit_FS((uint8_t*)introMessage, strlen(introMessage));

	eezybotarm_coordinate_t home = {
		.toolCoordinate = EEZYBOTARM_TOOL_HOME,
		.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
		.baseCoordinate = EEZYBOTARM_BASE_HOME
	};

	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);


	/*
	 * List of joint positions at each height, base is at each tower location
	 *
	 * These should come from the eezybotarm_towersOfHanoi() program
	 *
	 */
	const double TOOL_PREP_POSITION = 0.370;

	const double ELBOW_POSITION_1 = 0.600;
	const double ELBOW_APPROACH_POSITION_1 = 0.600;
	const double SHOULDER_POSITION_1 = 0.650;
	const double SHOULDER_APPROACH_POSITION_1 = 0.565;

	const double ELBOW_POSITION_2 = 0.635;
	const double ELBOW_APPROACH_POSITION_2 = 0.635;
	const double SHOULDER_POSITION_2 = 0.615;
	const double SHOULDER_APPROACH_POSITION_2 = 0.535;

	const double ELBOW_POSITION_3 = 0.670;
	const double ELBOW_APPROACH_POSITION_3 = 0.670;
	const double SHOULDER_POSITION_3 = 0.585;
	const double SHOULDER_APPROACH_POSITION_3 = 0.500;

	const double BASE_POSITION_1 = 0.775;
	const double BASE_POSITION_2 = 0.625;
	const double BASE_POSITION_3 = 0.475;

	/*
	 * Position 1 height 1
	 */
	eezybotarm_coordinate_t position_1_1 = {
		.toolCoordinate = TOOL_PREP_POSITION,
		.elbowCoordinate = ELBOW_POSITION_1,
		.shoulderCoordinate = SHOULDER_POSITION_1,
		.baseCoordinate = BASE_POSITION_1
	};
	eezybotarm_coordinate_t position_1_1_approach = {
		.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
		.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
		.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
		.baseCoordinate = BASE_POSITION_1
	};

	/*
	 * Position 1 height 2
	 */
	eezybotarm_coordinate_t position_1_2 = {
		.toolCoordinate = TOOL_PREP_POSITION,
		.elbowCoordinate = ELBOW_POSITION_2,
		.shoulderCoordinate = SHOULDER_POSITION_2,
		.baseCoordinate = BASE_POSITION_1
	};
	eezybotarm_coordinate_t position_1_2_approach = {
		.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
		.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
		.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
		.baseCoordinate = BASE_POSITION_1
	};

	/*
	 * Position 1 height 3
	 */
	eezybotarm_coordinate_t position_1_3 = {
		.toolCoordinate = TOOL_PREP_POSITION,
		.elbowCoordinate = ELBOW_POSITION_3,
		.shoulderCoordinate = SHOULDER_POSITION_3,
		.baseCoordinate = BASE_POSITION_1
	};
	eezybotarm_coordinate_t position_1_3_approach = {
		.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
		.elbowCoordinate = ELBOW_APPROACH_POSITION_3,
		.shoulderCoordinate = SHOULDER_APPROACH_POSITION_3,
		.baseCoordinate = BASE_POSITION_1
	};
	/*
	 * Position 2 height 1
	 */
	eezybotarm_coordinate_t position_2_1 = {
		.toolCoordinate = TOOL_PREP_POSITION,
		.elbowCoordinate = ELBOW_POSITION_1,
		.shoulderCoordinate = SHOULDER_POSITION_1,
		.baseCoordinate = BASE_POSITION_2
	};
	eezybotarm_coordinate_t position_2_1_approach = {
		.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
		.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
		.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
		.baseCoordinate = BASE_POSITION_2
	};
	/*
	 * Position 3 height 1
	 */
	eezybotarm_coordinate_t position_3_1 = {
		.toolCoordinate = TOOL_PREP_POSITION,
		.elbowCoordinate = ELBOW_POSITION_1,
		.shoulderCoordinate = SHOULDER_POSITION_1,
		.baseCoordinate = BASE_POSITION_3
	};
	eezybotarm_coordinate_t position_3_1_approach = {
		.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
		.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
		.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
		.baseCoordinate = BASE_POSITION_3
	};



	eezybotarm_coordinate_t program[] = {
			position_1_1_approach,
			position_1_1,
			position_1_1_approach,
			position_1_2_approach,
			position_1_2,
			position_1_2_approach,
			position_1_3_approach,
			position_1_3,
			position_1_3_approach,
			home,
			position_2_1_approach,
			position_2_1,
			position_2_1_approach,
			home,
			position_3_1_approach,
			position_3_1,
			position_3_1_approach,
			home
	};
	// Manually count the number of moves above
	const uint8_t numberOfMoves = 18;
	/*
	 * Run program
	 */
	for(uint8_t i; i < numberOfMoves;) {
		// Move to next position
		eezybotarm_move(eezy, program[i], delay);
		EEZYBOTARM_CHECK_FOR_ABORT
		// Get user prompt to either increase or decrease position
		uint8_t NEXT_POSITION_CHAR = 'a';
		uint8_t PREV_POSITION_CHAR = 'z';
		// Prompt user for movement and direction
		char* directionPrompt = "Press 'a' for next or 'z' for previous position\n\r";
		uint8_t directionPromptLength = strlen(directionPrompt);
		CDC_Transmit_FS((uint8_t*)directionPrompt, directionPromptLength);
		uint8_t userInput;
		// Loop until the next or previous position characters are entered
		do {
			userInput = (uint8_t) tolower(CDC_Get_Char());
		} while((userInput != NEXT_POSITION_CHAR) && (userInput != PREV_POSITION_CHAR) && (userInput != EEZYBOTARM_ABORT_CHARACTER));
		switch(userInput) {
		case 'a':
			i = i + 1;
			break;
		case 'z':
			if( i > 0 ) {
				i--;
			}
			break;
		case EEZYBOTARM_ABORT_CHARACTER:
			// Exit
			i = numberOfMoves;
			EEZYBOTARM_SEND_ABORT_MESSAGE
			break;
		}
	}
	EEZYBOTARM_PROGRAM_COMPLETED
}

/**
 * @function eezybotarm_towersOfHanoi
 * @brief Moves robot arm through the Towers of Hanoi program
 *
 */
void eezybotarm_towersOfHanoi(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
		if (EEZYBOTARM_INACTIVE_CHECK) {
			eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
			return;
		}
		char* introMessage = "Starting Towers of Hanoi program\n\r";
		CDC_Transmit_FS((uint8_t*)introMessage, strlen(introMessage));


		/*
		 * List of joint positions at each height, base is at each tower location
		 */
		const double ELBOW_POSITION_1 = 0.600;
		const double ELBOW_APPROACH_POSITION_1 = 0.600;
		const double SHOULDER_POSITION_1 = 0.650;
		const double SHOULDER_APPROACH_POSITION_1 = 0.565;

		const double ELBOW_POSITION_2 = 0.635;
		const double ELBOW_APPROACH_POSITION_2 = 0.635;
		const double SHOULDER_POSITION_2 = 0.615;
		const double SHOULDER_APPROACH_POSITION_2 = 0.535;

		const double ELBOW_POSITION_3 = 0.670;
		const double ELBOW_APPROACH_POSITION_3 = 0.670;
		const double SHOULDER_POSITION_3 = 0.585;
		const double SHOULDER_APPROACH_POSITION_3 = 0.500;

		const double BASE_POSITION_1 = 0.775;
		const double BASE_POSITION_2 = 0.625;
		const double BASE_POSITION_3 = 0.475;


		eezybotarm_coordinate_t home_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_HOME,
			.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
			.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t home_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
			.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
			.baseCoordinate = BASE_POSITION_2
		};

		uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
		eezybotarm_move(eezy, home_open, delay);

		/*
		 * Position 1 height 1
		 */
		eezybotarm_coordinate_t position_1_1_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_1_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_1_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_1_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_1
		};
		/*
		 * Position 1 height 2
		 */
		eezybotarm_coordinate_t position_1_2_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_2_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_2_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_2_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_1
		};
		/*
		 * Position 1 height 3
		 */
		eezybotarm_coordinate_t position_1_3_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_3,
			.shoulderCoordinate = SHOULDER_POSITION_3,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_3_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_3,
			.shoulderCoordinate = SHOULDER_POSITION_3,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_3_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_3,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_3,
			.baseCoordinate = BASE_POSITION_1
		};
		eezybotarm_coordinate_t position_1_3_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_3,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_3,
			.baseCoordinate = BASE_POSITION_1
		};
		/*
		 * Position 2 height 1
		 */
		eezybotarm_coordinate_t position_2_1_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_1_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_1_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_1_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_2
		};
		/*
		 * Position 2 height 2
		 */
		eezybotarm_coordinate_t position_2_2_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_2_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_2_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_2
		};
		eezybotarm_coordinate_t position_2_2_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_2
		};
		/*
		 * Position 3 height 1
		 */
		eezybotarm_coordinate_t position_3_1_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_1_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_1,
			.shoulderCoordinate = SHOULDER_POSITION_1,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_1_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_1_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_1,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_1,
			.baseCoordinate = BASE_POSITION_3
		};
		/*
		 * Position 3 height 2
		 */
		eezybotarm_coordinate_t position_3_2_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_2_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_2,
			.shoulderCoordinate = SHOULDER_POSITION_2,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_2_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_2_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_2,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_2,
			.baseCoordinate = BASE_POSITION_3
		};
		/*
		 * Position 3 height 3
		 */
		eezybotarm_coordinate_t position_3_3_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_POSITION_3,
			.shoulderCoordinate = SHOULDER_POSITION_3,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_3_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_POSITION_3,
			.shoulderCoordinate = SHOULDER_POSITION_3,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_3_approach_open = {
			.toolCoordinate = EEZYBOTARM_TOOL_OPEN,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_3,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_3,
			.baseCoordinate = BASE_POSITION_3
		};
		eezybotarm_coordinate_t position_3_3_approach_closed = {
			.toolCoordinate = EEZYBOTARM_TOOL_CLOSED,
			.elbowCoordinate = ELBOW_APPROACH_POSITION_3,
			.shoulderCoordinate = SHOULDER_APPROACH_POSITION_3,
			.baseCoordinate = BASE_POSITION_3
		};

		/*
		 * Setup program[]
		 */
		eezybotarm_program_step_t program[] = {
				// Pick up in position 1 height 3
				{&position_1_3_approach_open, delay},
				{&position_1_3_open, delay},
				{&position_1_3_closed, delay},
				{&position_1_3_approach_closed, delay},
				{&home_closed, delay},
				// Drop off in position 3 height 1
				{&position_3_1_approach_closed, delay},
				{&position_3_1_closed, delay},
				{&position_3_1_open, delay},
				{&position_3_1_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 1 height 2
				{&position_1_2_approach_open, delay},
				{&position_1_2_open, delay},
				{&position_1_2_closed, delay},
				{&position_1_2_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 2 height 1
				{&position_2_1_approach_closed, delay},
				{&position_2_1_closed, delay},
				{&position_2_1_open, delay},
				{&position_2_1_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 3 height 1
				{&position_3_1_approach_open, delay},
				{&position_3_1_open, delay},
				{&position_3_1_closed, delay},
				{&position_3_1_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 2 height 2
				{&position_2_2_approach_closed, delay},
				{&position_2_2_closed, delay},
				{&position_2_2_open, delay},
				{&position_2_2_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 1 height 1
				{&position_1_1_approach_open, delay},
				{&position_1_1_open, delay},
				{&position_1_1_closed, delay},
				{&position_1_1_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 3 height 1
				{&position_3_1_approach_closed, delay},
				{&position_3_1_closed, delay},
				{&position_3_1_open, delay},
				{&position_3_1_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 2 height 2
				{&position_2_2_approach_open, delay},
				{&position_2_2_open, delay},
				{&position_2_2_closed, delay},
				{&position_2_2_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 1 height 1
				{&position_1_1_approach_closed, delay},
				{&position_1_1_closed, delay},
				{&position_1_1_open, delay},
				{&position_1_1_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 2 height 1
				{&position_2_1_approach_open, delay},
				{&position_2_1_open, delay},
				{&position_2_1_closed, delay},
				{&position_2_1_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 3 height 2
				{&position_3_2_approach_closed, delay},
				{&position_3_2_closed, delay},
				{&position_3_2_open, delay},
				{&position_3_2_approach_open, delay},
				{&home_open, delay},
				// Pick up in position 1 height 1
				{&position_1_1_approach_open, delay},
				{&position_1_1_open, delay},
				{&position_1_1_closed, delay},
				{&position_1_1_approach_closed, delay},
				{&home_closed, delay},
				// Place in position 3 height 3
				{&position_3_3_approach_closed, delay},
				{&position_3_3_closed, delay},
				{&position_3_3_open, delay},
				{&position_3_3_approach_open, delay},
				{&home_open, delay},
				// Go back home
				{&home_open, delay}
		};
		// Manually count the number of moves in program[]
		const uint8_t numberOfMoves = 71;
		/*
		 * Run program[]
		 */
		for(uint8_t i; i < numberOfMoves; i++) {
			// Move to next position

			eezybotarm_move(eezy, *(program[i].coordinate), program[i].duration);
			EEZYBOTARM_CHECK_FOR_ABORT
		}
		EEZYBOTARM_PROGRAM_COMPLETED
}

/**
 * @function eezybotarm_dogTreatFeeder_setup
 * @brief Moves robot arm through a setup program for the eezybotarm_dogTreatFeeder program
 *
 */
void eezybotarm_dogTreatFeeder_setup(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
	if (EEZYBOTARM_INACTIVE_CHECK) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	char* message = "Running range of motion test 2 program\n\r";
	CDC_Transmit_FS((uint8_t*)message, strlen(message));

	/*
	 * List of coordinates
	 */
	eezybotarm_coordinate_t home = {
		.toolCoordinate = EEZYBOTARM_TOOL_HOME,
		.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
		.baseCoordinate = EEZYBOTARM_BASE_HOME
	};

	/*
	 * Run program
	 */
	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	EEZYBOTARM_PROGRAM_COMPLETED
}

/**
 * @function eezybotarm_dogTreatFeeder
 * @brief Moves robot arm through a procedure that feeds a good boy some treats, run as often as possible.
 *
 */
void eezybotarm_dogTreatFeeder(eezybotarm_t* eezy){
	// Abort if robot is not in a movement mode
	if (EEZYBOTARM_INACTIVE_CHECK) {
		eezybotarm_event(eezy, eezybotarm_ERROR_EVENT);
		return;
	}
	char* message = "Running range of motion test 2 program\n\r";
	CDC_Transmit_FS((uint8_t*)message, strlen(message));

	/*
	 * List of coordinates
	 */
	eezybotarm_coordinate_t home = {
		.toolCoordinate = EEZYBOTARM_TOOL_HOME,
		.elbowCoordinate = EEZYBOTARM_ELBOW_HOME,
		.shoulderCoordinate = EEZYBOTARM_SHOULDER_HOME,
		.baseCoordinate = EEZYBOTARM_BASE_HOME
	};

	/*
	 * Run program
	 */
	uint16_t delay = (uint16_t)(1000.0 * ((double)(eezybotarm_FASTEST - eezy->mode + 1) * 0.5));
	eezybotarm_move(eezy, home, delay);
	EEZYBOTARM_CHECK_FOR_ABORT
	EEZYBOTARM_PROGRAM_COMPLETED
}


