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

#include "rgb.h"
#include "servo.h"
#include "eezybotarm.h"
#include "tmc2209.h"
#include "icm-20608.h"


/*
 * Routines
 */

void tmc2209_button_task(void);



#endif /* INC_BB3_H_ */
