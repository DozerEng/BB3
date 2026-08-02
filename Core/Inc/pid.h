/*
 * pid.h
 *
 *  Created on: Aug 7, 2023
 *      Author: mpill
 */

#ifndef INC_PID_H_
#define INC_PID_H_


#include "stm32g4xx_hal.h"
#include "stdint.h"
#include "float.h"
//#include "stdio.h"
//#include "ctype.h"
//#include "string.h"

//#include "stdbool.h"
#include "controller.h"


/**
 * PID Data types
 */

typedef struct {
	controller_base_t base;

	float kp;
	float ki;
	float kd;

	float previousError;
	float integrator;

	float windupMax;
	float outputMax;

} pid_t;

pid_t pid_new(float kp, float ki, float kd);

int32_t pid_step(controller_base_t *self, int32_t setPoint, int32_t measurement);


#endif /* INC_PID_H_ */
