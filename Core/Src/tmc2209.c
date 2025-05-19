/**
 * tmc2209.c
 *
 *      Author: Michael Pillon
 *
 */


#include "tmc2209.h"




/**
 *
 *
 */

tmc2209_t tmc2209_new(
		// Functional setup
		uint8_t mode,
		uint8_t shaft,
		// Hardware setup
		TIM_HandleTypeDef *stepTimer,
		uint32_t stepTimerChannel,
		uint16_t dirPin,
		GPIO_TypeDef *dirPort,
		UART_HandleTypeDef *uart,
		uint8_t uartAddr
		) {
	tmc2209_t newMotor = {
			// Functional variables
			.mode = mode,
			.dir = TMC2209_FORWARD,
			// Hardware interface
			.stepTimer = stepTimer,
			.stepTimerChannel = stepTimerChannel,
			.dirPin = dirPin,
			.dirPort = dirPort,
			.uart = uart,
			.uartAddr = uartAddr,
			// Blank registers
			.gconf = 0x00000000,
			.gstat = 0x00000000,
			.ifcnt = 0x00000000,
			.slaveconf = 0x00000000,
			.otp_prog = 0x00000000,
			.otp_read = 0x00000000,
			.ioin = 0x00000000,
			.factory_conf = 0x00000000,
			.ihold_irun = 0x00000000,
			.tpowerdown = 0x00000000,
			.tstep = 0x00000000
	};
	/*
	 * Set initial register states
	 */

	//HAL_GPIO_WritePin(stepPort, stepPin, 0);
	HAL_GPIO_WritePin(dirPort, dirPin, 0);

	/*
	 * Configure driver using UART
	 */
	// General registers
	newMotor.gconf =	TMC2209_internal_rsense |
						TMC2209_I_scale_analog | 	// 0: Internal reference 1: From VREF
//						TMC2209_multistep_filt |		// Filtering > 750Hz
//						TMC2209_en_SpreadCycle | 	// 0: StealthChop 1: SpreadCycle
						TMC2209_index_step; 		// INDEX output shows pulse each step

	// Turn on shaft bit to invert motor direction
	if(shaft == TMC2209_INVERSE_MOTOR_DIR) {
		newMotor.gconf |= TMC2209_shaft;
	}

	tmc2209_set_GCONF(&newMotor);

	// CHOPCONF
	newMotor.chopconf = 	TMC2209_CHOPCONF_MRES_32 |	// Micro-step resolution
							TMC2209_CHOPCONF_intpol | 	// Interpolation to 256 micro-steps
//							TMC2209_CHOPCONF_dedge	|	// Step on rising and falling edges
							TMC2209_CHOPCONF_TBL_24 |	// blank_time
							TMC2209_CHOPCONF_vsense |	// 0: lowsense resistor voltage 1: high sense resistor voltage
							TMC2209_CHOPCONF_HEND_n1 |	// Hysteresis end
							TMC2209_CHOPCONF_HSTRT_2 |	// Hysteresis_start
							TMC2209_CHOPCONF_TOFF_10;	// Off time

	tmc2209_set_CHOPCONF(&newMotor);


	//	st.rms_current(mA, hold_multiplier);

	// Currents
	uint32_t newHoldCurrent = 0; // 0: Free wheel/passive breaking
	uint32_t newRunCurrent = 16; // 0=1/32 … 31=32/32 ratio of max current
	uint32_t newHoldDelay = 0; // 0: instant power down
	newMotor.ihold_irun =	newHoldCurrent |
							(newRunCurrent << 8) |
							(newHoldDelay << 16);
	tmc2209_set_IHOLD_IRUN(&newMotor);

	// TPOWERDOWN
	newMotor.tpowerdown = 128;	// ~2s until driver lowers to hold current
	tmc2209_set_TPOWERDOWN(&newMotor);

	// Values mostly taken from the Marlin default
	newMotor.pwmconf = 	(((uint32_t) 12 << TMC2209_PWMCONF_PWM_LIM_shift) & TMC2209_PWMCONF_PWM_LIM) |
						(((uint32_t) 8 << TMC2209_PWMCONF_PWM_REG_shift) & TMC2209_PWMCONF_PWM_REG) |
						TMC2209_PWMCONF_pwm_autograd |
						TMC2209_PWMCONF_pwm_autoscale |
						TMC2209_PWMCONF_freewheel_freewheeling |
						TMC2209_PWMCONF_pwm_freq_2_1024 |
						(((uint32_t) 14 << TMC2209_PWMCONF_PWM_GRAD_shift) & TMC2209_PWMCONF_PWM_GRAD) |
						(((uint32_t) 36 << TMC2209_PWMCONF_PWM_OFS_shift) & TMC2209_PWMCONF_PWM_OFS)  ;
	tmc2209_set_PWMCONF(&newMotor);


	newMotor.tpwmthrs = 0x00000000;
	tmc2209_set_TPWMTHRS(&newMotor);

	tmc2209_set_mode(&newMotor);

	// Reset statistics
	tmc2209_reset_GSTAT(&newMotor);


	/*
	 * Set other default values
	 */



//	HAL_Delay(100);
//	tmc2209_on(&newMotor);

	return newMotor;
}

/**
 *
 */
uint32_t tmc2209_read(tmc2209_t *tmc, tmc2209_read_request_t readDatagram) {

	// Disable until reading can be implemented, currently the Tx can't release the pullup, making reading impossible
	return 0;

	/**
	 * Request data
	 */
	uint8_t reqMsg[TMC2209_READ_REQUEST_DATAGRAM_LENGTH];

	reqMsg[0] = TMC2209_SYNC_BYTE;
	reqMsg[1] = readDatagram.slaveAddress;
	reqMsg[2] = readDatagram.registerAddress | TMC2209_RW_READ;
	reqMsg[3] = 0;
	tmc2209_calculateCRC(reqMsg, TMC2209_READ_REQUEST_DATAGRAM_LENGTH);

	uint8_t resMsg[TMC2209_READ_RESPONSE_DATAGRAM_LENGTH];
	memset(resMsg, 0, TMC2209_READ_RESPONSE_DATAGRAM_LENGTH);

	HAL_UART_Transmit(tmc->uart, reqMsg, TMC2209_READ_REQUEST_DATAGRAM_LENGTH, TMC2209_UART_TIMEOUT);
	/**
	 * Receive response
	 */

	uint8_t status = HAL_UART_Receive(tmc->uart, resMsg, TMC2209_READ_RESPONSE_DATAGRAM_LENGTH, TMC2209_UART_TIMEOUT);

	if(status != 0) {
		return 0;
	}

	/* Format bytes into single int */
	uint32_t dataBuffer[] = { resMsg[3] << 24, resMsg[4] << 16, resMsg[5] << 8, resMsg[6] };
	return dataBuffer[0] | dataBuffer[1] | dataBuffer[2] | dataBuffer[3];
}


/**
 *
 */
void tmc2209_write(tmc2209_t *tmc, tmc2209_write_t writeDatagram) {

	uint8_t msg[TMC2209_WRITE_DATAGRAM_LENGTH];

	msg[0] = TMC2209_SYNC_BYTE;
	msg[1] = writeDatagram.slaveAddress;
	msg[2] = writeDatagram.registerAddress | TMC2209_RW_WRITE;
	msg[3] = (uint8_t)((writeDatagram.data >> 24) & 0x000000ff);
	msg[4] = (uint8_t)((writeDatagram.data >> 16) & 0x000000ff);
	msg[5] = (uint8_t)((writeDatagram.data >> 8) & 0x000000ff);
	msg[6] = (uint8_t)(writeDatagram.data & 0x000000ff);
	msg[7] = 0;
	tmc2209_calculate_CRC(msg, TMC2209_WRITE_DATAGRAM_LENGTH);

	HAL_UART_Transmit(tmc->uart, msg, TMC2209_WRITE_DATAGRAM_LENGTH, TMC2209_UART_TIMEOUT);
}

/**
 *	@function tmc2209_setGCONF
 *	@brief programs CGONF register
 */
void tmc2209_set_GCONF(tmc2209_t *tmc) {
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_GCONF_ADDR,
		.data = tmc->gconf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_getGCONF
 *	@brief reads CGONF register
 */
void tmc2209_get_GCONF(tmc2209_t *tmc){

}

/**
 *	@function
 *	@brief
 */
void tmc2209_get_GSTAT(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_reset GSTAT
 *	@brief Reset the status registers
 */
void tmc2209_reset_GSTAT(tmc2209_t *tmc){
	tmc2209_write_t msg = {
			.slaveAddress = tmc->uartAddr,
			.registerAddress = TMC2209_GSTAT_ADDR,
			.data = TMC2209_GSTAT_RESET
	  	};
		tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_IFCNT(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_SLAVECONF(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_SLAVECONF_ADDR,
		.data = tmc->slaveconf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_OTP_PROG(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_OTP_PROG_ADDR,
		.data = tmc->otp_prog
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_OTP_READ(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_IOIN(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_FACTORY_CONF(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_FACTORY_CONF_ADDR,
		.data = tmc->factory_conf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_FACTORY_CONF(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_setIHOLD_IRUN
 *	@brief Programs holding current, run current, and hold delay time
 */
void tmc2209_set_IHOLD_IRUN(tmc2209_t *tmc) {
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_IHOLD_IRUN_ADDR,
		.data = tmc->ihold_irun
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_TPOWERDOWN(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_TPOWERDOWN_ADDR,
		.data = tmc->tpowerdown
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_TSTEP(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_TPWMTHRS(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_TPWMTHRS_ADDR,
		.data = tmc->tpwmthrs
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_VACTUAL(tmc2209_t *tmc){
	// Max sure speed isn't above the maximum allowable
	if(tmc->vactual >= TMC2209_VACTUAL_MAX_P) {
		tmc->vactual = TMC2209_VACTUAL_MAX_P;
	}
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_VACTUAL_ADDR,
		.data = (tmc->vactual & TMC2209_VACTUAL)
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_TCOOLTHRS(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_TCOOLTHRS_ADDR,
		.data = tmc->tcoolthrs
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_SGTHRS(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_SGTHRS_ADDR,
		.data = tmc->sgthrs
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_SG_RESULT(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_COOLCONF(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_COOLCONF_ADDR,
		.data = tmc->coolconf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_MSCNT(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_MSCURACT(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_CHOPCONF(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_CHOPCONF_ADDR,
		.data = tmc->chopconf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_CHOPCONF(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_DRV_STATUS(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_set_PWMCONF(tmc2209_t *tmc){
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_PWMCONF_ADDR,
		.data = tmc->pwmconf
  	};
	tmc2209_write(tmc, msg);
}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_PWMCONF(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_PWM_SCALE(tmc2209_t *tmc){

}

/**
 *	@function tmc2209_
 *	@brief
 */
void tmc2209_get_PWM_AUTO(tmc2209_t *tmc){

}


/**
 *	Calculate CRC
 */
void tmc2209_calculate_CRC(uint8_t* datagram, uint8_t datagramLength){
	// Taken from TMC2209 datasheet
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
		currentByte = datagram[i]; // Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
			{
				*crc = (*crc << 1) ^ 0x07;
			}
			else
			{
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}




/**
 *
 */
void tmc2209_reset(tmc2209_t *tmc) {
	// Do whatever is necessary to reset the device

}

/**
 *
 */
void tmc2209_set_mode(tmc2209_t *tmc) {
	// Do whatever is necessary to set the mode of the device
	if(tmc->mode == TMC2209_FULL_GPIO_CONTROL) {

		//ToDo: Is anything else required here?



	} else if (tmc->mode == TMC2209_VELOCITY_CONTROL) {
		// Ensure UART is set up correctly
		tmc->gconf |= (TMC2209_pdn_disable | TMC2209_mstep_reg_select);
		tmc2209_set_GCONF(tmc);

		// Set vsense control related variables
		tmc->vactual_MAX = TMC2209_VACTUAL_MAX_P;
		tmc->acceleration = 1000;
		tmc->vactual = 0x00000000;
		tmc2209_set_VACTUAL(tmc);

	} else if (tmc->mode == TMC2209_UART_STEP_DIR_CONTROL) {
		// Ensure UART is set up correctly
		tmc->gconf |= TMC2209_pdn_disable | TMC2209_mstep_reg_select;
		tmc2209_set_GCONF(tmc);

		//ToDo: Is anything else required here?

	} else if (tmc->mode == TMC2209_FULL_UART_STEPPING_CONTROL) {
		// Ensure UART is set up correctly
		tmc->gconf |= TMC2209_pdn_disable | TMC2209_mstep_reg_select;
		tmc2209_set_GCONF(tmc);

		//ToDo: Is anything else required here?

	} else {
		// Invalid mode
	}

}

/**
 *
 */
void tmc2209_set_dir(tmc2209_t *tmc) {
	// Check mode

	// Do whatever is necessary to set the dir of the device

}

/**
 *
 */
void tmc2209_step_off(tmc2209_t *tmc) {
	// Check you are in the right mode

	// Do thing
	HAL_TIM_PWM_Stop(tmc->stepTimer, tmc->stepTimerChannel);
}

/**
 *
 */
void tmc2209_step_on(tmc2209_t *tmc) {
	// Check you are in the right mode

	// Do thing
	HAL_TIM_PWM_Start(tmc->stepTimer, tmc->stepTimerChannel);
}

/**
 *
 */
void tmc2209_step(tmc2209_t *tmc) {
	// Check you are in the right mode

	// Do thing

}

/**
 *
 */
void tmc2209_step_count(tmc2209_t *tmc, uint16_t count) {
	// Check you are in the right mode

	// Do thing

}




/**
 *
 */
void tmc2209_set_speed(tmc2209_t *tmc, uint32_t speed) {

}

/**
 *
 */
void tmc2209_set_acceleration(tmc2209_t *tmc, uint32_t acceleration) {

}

/**
 *
 */
void tmc2209_on(tmc2209_t *tmc) {

}

/**
 *
 */
void tmc2209_off(tmc2209_t *tmc) {

}


