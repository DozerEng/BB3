/*
 * controller.h
 *
 *  Created on: Jul 26, 2026
 *      Author: mpill
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_



#include "stm32g4xx_hal.h"
#include "stdint.h"
//#include "stdio.h"
//#include "ctype.h"
//#include "string.h"

//#include "stdbool.h"

typedef struct {
	int32_t setPoint;

	int32_t acceleration;
	int32_t accelerationMax;

	int32_t inputCurrent;
	int32_t inputPrevious;

	int32_t outputCurrent;
	int32_t outputPrevious;

	int32_t errorCurrent;
	int32_t errorCurrentAbsolute;
	int32_t errorPrevious;

//	int32_t stepCurrent;
//	int32_t stepPrevious;

} controller_t;

controller_t controller_new(int32_t setPoint);

void controller_step(controller_t *ctrl);

#endif /* INC_CONTROLLER_H_ */
