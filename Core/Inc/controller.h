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


/*
 * Polymorphic controller for use with linear, PID, LQR, etc...
 */
typedef struct controller_base controller_base_t;

struct controller_base
{
    int32_t (*step)(controller_base_t *self,
            		int32_t setPoint,
					int32_t measurement);
};

/*
 * Basic linear controller
 */
typedef struct {
	controller_base_t base;

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


} controller_t;

controller_t controller_new(int32_t setPoint, int32_t acceleration);

int32_t controller_step(controller_base_t *self, int32_t setPoint, int32_t measurement);

#endif /* INC_CONTROLLER_H_ */
