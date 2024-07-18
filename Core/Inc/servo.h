/*
 * servo.h
 *
 *	Description: Control servo motors using floating point values of 0.0 to 1.0
 *			representing 0 to 100% of the full range of motion.
 *
 *
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32g4xx_hal.h"


/**
 * Defines
 */

/**
 * Hardware max and min limits represent duty cycle on a 20ms period square wave
 */
#define MG90S_MAX_LIMIT 0.118 // Duty cycle, MAX: 2.360944ms
#define MG90S_MIN_LIMIT 0.026 // Duty cycle, MIN: 0.520208ms
#define MG996R_MAX_LIMIT 0.115
#define MG996R_MIN_LIMIT 0.026



/**
 * Servo Data types
 */

typedef struct {
	/** Hardware */
	TIM_HandleTypeDef *tim; /** Pointer to timer channel */
	uint32_t channel;

	/** Parameters */
	double setPoint; /* 0.0 to 0.1 */
	double home; /* 0.0 to 0.1 */

	uint16_t maxPulseWidth; /** Movement limits of servo */
	uint16_t minPulseWidth;

	const uint16_t absMaxPulseWidth; /** Physical limits of servo */
	const uint16_t absMinPulseWidth;
} servo_t;


servo_t Servo_newMG90S(TIM_HandleTypeDef *htim, uint32_t channel);
servo_t Servo_newMG996R(TIM_HandleTypeDef *htim, uint32_t channel);

void Servo_start(servo_t *servo);
void Servo_stop(servo_t *servo);
void Servo_setPosition(servo_t *servo, double newSetPoint);
void Servo_setMax(servo_t *servo, double newMax);
void Servo_setMin(servo_t *servo, double newMin);




#endif /* INC_SERVO_H_ */
