/*
 * icm-20608.c
 *
 *  Created on: May 5, 2025
 *      Author: Michael Pillon
 */


#include "icm20608.h"








icm20608_t icm20608_new (


	uint16_t interrupt_pin,
	GPIO_TypeDef *interrupt_port
) {
	icm20608_t newIcm;
	newIcm.interrupt_pin = interrupt_pin;
	newIcm.interrupt_port = interrupt_port;

	return newIcm;
}







