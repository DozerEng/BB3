/*
 * eezybotarm.h
 *
 *		Author: Michael Pillon
 *      Description: Control for EEZYbotARM MK2 robot arm.
 *
 *      Link: https://www.thingiverse.com/thing:1454048
 *
 *	Sample Main():







 *
 *
 */

#ifndef INC_EEZYBOTARM_H_
#define INC_EEZYBOTARM_H_


#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"
#include "servo.h"
#include "rgb.h"

#include "stdint.h"
#include "stdbool.h"
#include "ctype.h"
#include "string.h"


//#include "stdbool.h"

/**
 * Constants
 */

/** Boot sequence configuration */
#define EEZYBOTARM_LOADING_DELAY	50 	// In ms, allows USB transmit buffer to clear
#define EEZYBOTARM_DRAMATIC_EFFECT_FACTOR	2
#define EEZYBOTARM_DRAMATIC_EFFECT_COUNT	5


/**
 * Default max and min limits
 */
#define EEZYBOTARM_TOOL_OPEN		0.5
#define EEZYBOTARM_TOOL_CLOSED		0.25

#define EEZYBOTARM_TOOL_MAX			1.0
#define EEZYBOTARM_TOOL_MIN			0.22
#define EEZYBOTARM_ELBOW_MAX		0.9
#define EEZYBOTARM_ELBOW_MIN		0.5
#define EEZYBOTARM_SHOULDER_MAX		0.7
#define EEZYBOTARM_SHOULDER_MIN		0.25 // This limit can be exceeded but tool position veries beyond this point
#define EEZYBOTARM_BASE_MAX			1.0
#define EEZYBOTARM_BASE_MIN			0.0

/**
 * Default home positions
 */
#define EEZYBOTARM_TOOL_HOME		EEZYBOTARM_TOOL_OPEN
#define EEZYBOTARM_ELBOW_HOME		0.75
#define EEZYBOTARM_SHOULDER_HOME	0.35
#define EEZYBOTARM_BASE_HOME		0.5

/**
 * Gain constants for jog movements
 */
#define EEZYBOTARM_SLOWEST_JOG	0.005
#define EEZYBOTARM_SLOW_JOG		0.01
#define EEZYBOTARM_NORMAL_JOG	0.025
#define EEZYBOTARM_FAST_JOG		0.05
#define EEZYBOTARM_FASTEST_JOG	0.1

#define EEZYBOTARM_EVENT_LENGTH 100	// Number of "No event" event calls

// True if arm is inactive
#define EEZYBOTARM_INACTIVE_CHECK ((eezy->mode < eezybotarm_SLOWEST) || (eezy->mode > eezybotarm_FASTEST))
// Blocking code until a key is pressed
#define EEZYBOTARM_PRESS_ANY_KEY_TO_CONTINUE CDC_Get_Char();

/**
 * Data types and enums
 */

typedef enum {
	eezybotarm_TOOL,
	eezybotarm_ELBOW,
	eezybotarm_SHOULDER,
	eezybotarm_BASE
} eezybotarm_servoPositions_t;

typedef enum {
	eezybotarm_NO_EVENT,
	eezybotarm_TOOL_EVENT,
	eezybotarm_ELBOW_EVENT,
	eezybotarm_SHOULDER_EVENT,
	eezybotarm_BASE_EVENT,
	eezybotarm_MOVE_EVENT,
	eezybotarm_CONFIG_EVENT,
	eezybotarm_ERROR_EVENT
} eezybotarm_event_t;

typedef enum {
	eezybotarm_OFF = 0,
	eezybotarm_SLOWEST = 1,
	eezybotarm_SLOW = 2,
	eezybotarm_NORMAL = 3,
	eezybotarm_FAST = 4,
	eezybotarm_FASTEST = 5,
	eezybotarm_MODE_INCREMENT = 50,
	eezybotarm_MODE_DECREMENT = 51,
	eezybotarm_FAULT = 99
} eezybotarm_mode_t;

typedef struct {
	bool initialBootFlag;
	servo_t* tool;
	servo_t* elbow;
	servo_t* shoulder;
	servo_t* base;

	double toolOpen;
	double toolClosed;

	double toolMin;
	double toolMax;
	double elbowMin;
	double elbowMax;
	double shoulderMin;
	double shoulderMax;
	double baseMin;
	double baseMax;

	rgb_t* modeRGB;		// Indicates current mode
	rgb_t* eventRGB;	// Events pulse this with color indicating the type of event
	uint8_t eventTicksRemaining; // Number of "no events" before the LED is turned off

	uint8_t mode;
	double jogStepSize;

} eezybotarm_t;

typedef struct {
	double toolCoordinate;
	double elbowCoordinate;
	double shoulderCoordinate;
	double baseCoordinate;
} eezybotarm_coordinate_t;

typedef struct {
	eezybotarm_coordinate_t* coordinate;
	uint16_t duration;
} eezybotarm_program_step_t;


/**
 * Exported functions
 */

eezybotarm_t eezybotarm_new(servo_t* tool, servo_t* elbow, servo_t* shoulder, servo_t* base, rgb_t* event, rgb_t* mode);

void eezybotarm_on(eezybotarm_t* eezy);
void eezybotarm_off(eezybotarm_t* eezy);

/**
 * Helpers
 */
void eezybotarm_home(eezybotarm_t* eezy);
void eezybotarm_setTool(eezybotarm_t* eezy, double newPosition);
void eezybotarm_setElbow(eezybotarm_t* eezy, double newPosition);
void eezybotarm_setShoulder(eezybotarm_t* eezy, double newPosition);
void eezybotarm_setBase(eezybotarm_t* eezy, double newPosition);

void eezybotarm_openTool(eezybotarm_t* eezy);
void eezybotarm_closeTool(eezybotarm_t* eezy);

void eezybotarm_setMode(eezybotarm_t* eezy, eezybotarm_mode_t mode);
void eezybotarm_event(eezybotarm_t* eezy, eezybotarm_event_t eventType);
void eezybotarm_move(eezybotarm_t* eezy, eezybotarm_coordinate_t destination, uint16_t duration);

/*
 * Control Methods
 */
void eezybotarm_update_qawsedrf(eezybotarm_t* eezy);


/*
 * Program Macros
 */

// For aborting programs between move commands
#define EEZYBOTARM_ABORT_CHARACTER	'q'
#define EEZYBOTARM_SEND_ABORT_MESSAGE	char* message = "Aborting program\n\r";\
		CDC_Transmit_FS((uint8_t*)message, strlen(message));
#define EEZYBOTARM_CHECK_FOR_ABORT if( CDC_Get_Buffer()[0] == EEZYBOTARM_ABORT_CHARACTER) {\
	EEZYBOTARM_SEND_ABORT_MESSAGE\
	return;\
	}
// Completed program
#define EEZYBOTARM_PROGRAM_COMPLETED 	const char* completedMessage = "Program complete\n\r";\
	CDC_Transmit_FS((uint8_t*)completedMessage, strlen(completedMessage));



/**
 * Programs
 *
 * Note: The program list enum and string array must be updated when adding new programs
 * 		- The first program must be 0 and there is a maximum of 9 programs
 * 		- See the eezybotarm_runProgram() implementation the .c file
 */

void eezybotarm_runProgram(eezybotarm_t* eezy);

void eezybotarm_rangeOfMotionTest1(eezybotarm_t* eezy);
void eezybotarm_rangeOfMotionTest2(eezybotarm_t* eezy);
void eezybotarm_towersOfHanoi_setup(eezybotarm_t* eezy);
void eezybotarm_towersOfHanoi(eezybotarm_t* eezy);
void eezybotarm_dogTreatFeeder_setup(eezybotarm_t* eezy);
void eezybotarm_dogTreatFeeder(eezybotarm_t* eezy);




#endif /* INC_EEZYBOTARM_H_ */
