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

tmc2209_t tmc2209_new(TIM_HandleTypeDef *stepTimer, uint32_t stepTimerChannel, uint16_t dirPin, GPIO_TypeDef *dirPort, UART_HandleTypeDef *uart, uint8_t uartAddr) {
	tmc2209_t newMotor = {
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
			.tpower_down = 0x00000000,
			.tstep = 0x00000000
	};
	/*
	 * Set initial pin states
	 */
	// Turn off motors
	//HAL_GPIO_WritePin(stepPort, stepPin, 0);
	HAL_GPIO_WritePin(dirPort, dirPin, 0);

//	tmc2209_off(&newMotor);


	/*
	 * Configure driver using UART
	 */



	/*
	 * Set general configuration register
	 */
	uint32_t newGCONF = TMC2209_internal_rsense | TMC2209_pdn_disable | TMC2209_mstep_reg_select;//multistep_filt
	tmc2209_setGCONFG(&newMotor, newGCONF);
	/*
	 *  Set currents
	 *  Min: 0
	 *  Max: 31 / 0b00011111
	 */
	uint32_t newHoldCurrent = 0; // Free wheel/passive breaking
	uint32_t newRunCurrent = 10;
	uint32_t newHoldDelay = 0; // Auto off
	uint32_t newIHOLD_IRUN = newHoldCurrent | (newRunCurrent << 8) | (newHoldDelay << 16);
	tmc2209_setIHOLD_IRUN(&newMotor, newIHOLD_IRUN);


	// Set microstep value
	// See MRES bits in CHOPCONF register - Chopper configuration


	return newMotor;
}

/**
 *
 */
uint32_t tmc2209_read(tmc2209_t *tmc, tmc2209_read_request_t readDatagram) {
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
	tmc2209_calculateCRC(msg, TMC2209_WRITE_DATAGRAM_LENGTH);

	HAL_UART_Transmit(tmc->uart, msg, TMC2209_WRITE_DATAGRAM_LENGTH, TMC2209_UART_TIMEOUT);
}
/**
 *	@function tmc2209_setGCONF
 *	@brief programs CGONF register
 */
void tmc2209_setGCONFG(tmc2209_t *tmc, uint32_t data) {
	tmc->gconf = data;
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_GCONF,
		.data = data
  	};
	tmc2209_write(tmc, msg);


}

/**
 *	@function tmc2209_setIHOLD_IRUN
 *	@brief Programs holding current, run current, and hold delay time
 */
void tmc2209_setIHOLD_IRUN(tmc2209_t *tmc, uint32_t data) {
	tmc->ihold_irun = data;
	tmc2209_write_t msg = {
		.slaveAddress = tmc->uartAddr,
		.registerAddress = TMC2209_IHOLD_IRUN,
		.data = data
  	};
	tmc2209_write(tmc, msg);
}


/**
 * @function tmc2209_step
 * @brief
 */

void tmc2209_step(tmc2209_t *tmc) {
//	HAL_GPIO_WritePin(tmc->stepPort, tmc->stepPin, 1);
//	HAL_Delay(50);
//	HAL_GPIO_WritePin(tmc->stepPort, tmc->stepPin, 0);
//	HAL_Delay(50);
}

















/**
 *
 */
void tmc2209_reset(tmc2209_t *tmc) {

}
/**
 *
 */
void tmc2209_off(tmc2209_t *tmc) {
	HAL_TIM_PWM_Stop(tmc->stepTimer, tmc->stepTimerChannel);
}
/**
 *
 */
void tmc2209_on(tmc2209_t *tmc) {
	HAL_TIM_PWM_Start(tmc->stepTimer, tmc->stepTimerChannel);
}

/**
 *
 */
void tmc2209_setinternalRSense(tmc2209_t *tmc, uint8_t data) {
	// Validate valid data

	// Transmit message

}
/**
 *
 */
void tmc2209_setCurrent(tmc2209_t *tmc, double mACurrent) {

}
/**
 *
 */
void tmc2209_setMode(tmc2209_t *tmc, uint8_t mode){

}
/**
 *
 */
void tmc2209_setMicroStep(tmc2209_t *tmc, uint8_t step) {

}
/**
 *
 */
void tmc2209_resetCounter(tmc2209_t *tmc) {

}
/**
 *
 */
void tmc2209_calculateCRC(uint8_t* datagram, uint8_t datagramLength){
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
