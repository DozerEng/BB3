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
//#include "stdio.h"
//#include "ctype.h"
//#include "string.h"

#include "stdbool.h"



/**
 * PID Data types
 */

typedef struct {
	float kp;
	float ki;
	float kd;
	float dt;

	float windupMax;
	float windupMin;

	float outputMax;
	float outputMin;

	float previousInput;
	float previousOutput;
	float previousError;
	float integrator;

    /**
     * Derivative low-pass filter time constant
     * The time constant of the filter (-3dB frequency in Hz, fc = 1 / (2*pi*tau)).
     * A larger value of tau means the signal is filtered more heavily.
     * As tau approaches zero, the differentiator approaches a 'pure differentiator' with no filtering.
     */
    float tau;

} PID;



#endif /* INC_PID_H_ */
