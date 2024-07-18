/**
 * servo.c
 *
 *      Author: Michael Pillon
 *
 */

#include "servo.h"

/**
 * Create and initialize a new MG90S servo
 */

servo_t Servo_newMG90S(TIM_HandleTypeDef *htim, uint32_t channel) {
	// TODO: Check for valid values
	servo_t  newServo = {
			.tim = htim,
			.channel = channel,
			.setPoint = 0.5,
			.home = 0.5,
			// Period x Duty cycle
			.maxPulseWidth = (uint16_t)(htim->Instance->ARR * MG90S_MAX_LIMIT),
			.minPulseWidth = (uint16_t)(htim->Instance->ARR * MG90S_MIN_LIMIT),
			.absMaxPulseWidth = (uint16_t)(htim->Instance->ARR * MG90S_MAX_LIMIT),
			.absMinPulseWidth = (uint16_t)(htim->Instance->ARR * MG90S_MIN_LIMIT)
	};
	//Servo_start(&newServo);
	return newServo;
}

/**
 * Create and initialize a new MG996 servo
 */

servo_t Servo_newMG996R(TIM_HandleTypeDef *htim, uint32_t channel) {
	// TODO: Check for valid values
	servo_t  newServo = {
			.tim = htim,
			.channel = channel,
			.setPoint = 0.5,
			.home = 0.5,
			// Period x Duty cycle
			.maxPulseWidth = (uint16_t)(htim->Instance->ARR * MG996R_MAX_LIMIT),
			.minPulseWidth = (uint16_t)(htim->Instance->ARR * MG996R_MIN_LIMIT),
			.absMaxPulseWidth = (uint16_t)(htim->Instance->ARR * MG996R_MAX_LIMIT),
			.absMinPulseWidth = (uint16_t)(htim->Instance->ARR * MG996R_MIN_LIMIT)
	};
	//Servo_start(&newServo);
	return newServo;
}

/**
 * Start and stop the servo motor
 */
void Servo_start(servo_t *servo) {
	HAL_TIM_PWM_Start(servo->tim, servo->channel);
	Servo_setPosition(servo, servo->setPoint);
}

void Servo_stop(servo_t *servo) {
	HAL_TIM_PWM_Stop(servo->tim, servo->channel);
}

/**
 * Set Position based on a floating point value between 0.0 and 1.0
 */
void Servo_setPosition(servo_t *servo, double newSetPoint) {
	/*
 	 */
	if(newSetPoint >= 1.0) {
		servo->setPoint = 1.0;
	} else if(newSetPoint <= 0.0 ) {
		servo->setPoint = 0.0;
	} else {
		servo->setPoint = newSetPoint;
	}

	uint32_t pulseWidth = (uint32_t) (newSetPoint * (double) (servo->maxPulseWidth - servo->minPulseWidth)) + servo->minPulseWidth;
    __HAL_TIM_SET_COMPARE(servo->tim, servo->channel, pulseWidth);
}


/**
 * @function Servo_setMax
 * @brief Sets soft end stop value based on percent of absolute range of motion
 *
 * @param newMaxDutyCycle Upper cutoff as percentage of absolute range of motion
 */
void Servo_setMax(servo_t *servo, double newMax) {
	if(newMax >= 1.0) {
		/* Set to physical maximum limit */
		servo->maxPulseWidth = servo->absMaxPulseWidth;
	} else if (newMax <= 0.0 ) {
		/* Set to same value as min pulse width, the 0.0 position */
		servo->maxPulseWidth = servo->absMinPulseWidth;
	} else {
		servo->maxPulseWidth = (uint32_t) (newMax * (double) (servo->absMaxPulseWidth - servo->absMinPulseWidth)) + servo->absMinPulseWidth;
	}
}

/**
 * @function Servo_setMin
 * @brief Sets soft end stop value based on percent of absolute range of motion
 *
 * @param newMinDutyCycle Lower cutoff as percentage of absolute range of motion
 */
void Servo_setMin(servo_t *servo, double newMin) {
	if(newMin >= 1.0) {
		/* Set to physical maximum limit */
		servo->minPulseWidth = servo->absMaxPulseWidth;
	} else if (newMin <= 0.0 ) {
		/* Set to same value as min pulse width, the 0.0 position */
		servo->minPulseWidth = servo->absMinPulseWidth;
	} else {
		servo->minPulseWidth = (uint32_t) (newMin * (double) (servo->absMaxPulseWidth - servo->absMinPulseWidth)) + servo->absMinPulseWidth;
	}
}


