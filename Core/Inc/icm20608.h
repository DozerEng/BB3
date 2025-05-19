/**
 * icm20608.h
 *
 *  Author: Michael Pillon
 *
 *      Library for the ICM-20608 IMU with 3 axis accelerometer, 3 axis gyroscope, temperature sensor (i think)
 *
 *	Product Page:	https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20608-2/
 *	Datasheet: 		https://invensense.tdk.com/wp-content/uploads/2015/03/DS-000081-v1.01.pdf
 *	Register Map: 	https://invensense.tdk.com/wp-content/uploads/2015/03/RM-000030-v1.0.pdf
 *
 */


#ifndef INC_ICM20608_H_
#define INC_ICM20608_H_

#include "stm32g4xx_hal.h"
//#include "usbd_cdc_if.h"
#include "stdint.h"



/*
 * Macros
 */




/*
 * Data types
 */

typedef struct {

	// ToDo: Add SPI IO details

	uint16_t interrupt_pin;
	GPIO_TypeDef *interrupt_port;


} icm20608_t;

/*
 * Functions
 */


icm20608_t icm20608_new (


	uint16_t interrupt_pin,
	GPIO_TypeDef *interruptPort
);


/*
 * Functions
 */






#endif /* INC_ICM20608_H_ */
