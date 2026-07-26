/*
 * bb3.h
 *
 *  Created on: May 9, 2025
 *      Author: mpill
 */

#ifndef INC_BB3_H_
#define INC_BB3_H_


#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"

#include "led.h"
#include "rgb.h"
#include "button.h"
#include "servo.h"
#include "eezybotarm.h"
#include "tmc2209.h"
#include "icm20608.h"



#define HEARTBEAT_TOP_LED_INTERVAL 	10		// Period in ms
#define HEARTBEAT_BOT_LED_INTERVAL 	250 	// Period in ms

#define TASK_RGB_TIMEOUT			1000	// in ms

// Pick which interfaces for logging and/or data collection
#define LOG_USB 	true
#define LOG_UART 	false


/*
 * Routines
 */
typedef struct
{
	  uint32_t period;

	  rgb_t *rgbLeft, *rgbRight;
	  rgb_t *rgbEventButton, *rgbEventProcess;
	  button_t *pbTop, *pbMid, *pbBot;
	  tmc2209_t *motorLeft, *motorRight;

	  bool modeDebug;

	  uint8_t direction;
	  uint8_t speedMode;

	  double setPoint;
	  double speed;
	  double acceleration;


} bb3_t;


bb3_t bb3_new(
		  rgb_t *rgbLeft, rgb_t *rgbRight,
		  rgb_t *rgbEventButton, rgb_t *rgbEventProcess,
		  button_t *pbTop, button_t *pbMid, button_t *pbBot,
		  tmc2209_t *motorLeft, tmc2209_t *motorRight,
		  bool modeDebug,
		  uint8_t direction,
		  uint8_t speedMode,
		  double acceleration
		  );



/*
 * Routines
 */

// Intro tasks
void rgb_green_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
void rgb_white_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
void rgb_intro_task(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfSteps);

// Button tasks
void tmc2209_3_button_task(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		tmc2209_t *tmc1, tmc2209_t *tmc2);
void tmc2209_icm20608_3_button_task(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		icm20608_t *icm,
		tmc2209_t *tmc1, tmc2209_t *tmc2);


// Program task(s)

void bb3_task_button( uint32_t period,bb3_t *bb3);
void bb3_task_control_loop(uint32_t period,bb3_t *bb3);

// Logging tasks
void tmc2209_task_log(uint32_t period, tmc2209_t *tmc);
void icm20608_task_log(uint32_t period, icm20608_t *icm);

 // Heartbeat(s)
void bb3_task_heartbeat(led_t *led1,uint32_t led1Period,led_t *led2,uint32_t led2Period);

/*
 * Helper Functions
 */


/*
 * Direction
 */
#define BB3_DIRECTION_FORWARD 		0
#define BB3_DIRECTION_REVERSE 		1
#define BB3_DIRECTION_RIGHT_TURN	2
#define BB3_DIRECTION_LEFT_TURN		3
#define BB3_DIRECTION_BRAKING		4
#define BB3_DIRECTION_OFF			5

void bb3_set_direction(bb3_t *bb3, uint8_t direction);

/*
 *  Velocity control
 */

#define BB3_SPEED_RPM 	0
#define BB3_SPEED_RADS	1	// rad/s
#define BB3_SPEED_MMS	2 	// mm/s

void bb3_set_speed_mode(bb3_t *bb3, uint8_t speedMode);

void bb3_set_acceration(bb3_t *bb3, float acceleration);
void bb3_set_speed(bb3_t *bb3,  int32_t speed);
void bb3_set_max_speed(bb3_t *bb3, float maxSpeed);






#endif /* INC_BB3_H_ */
