/**
 * tmc2209.h
 *
 *  Author: Michael Pillon
 *
 *      UART control of TMC2209 stepper motor drivers
 *
 *	References:
 *		https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/696/TMC2209_Rev.1.08.pdf
 *
 */


#ifndef INC_TMC2209_H_
#define INC_TMC2209_H_

#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

#include "stdint.h"
//#include "stdbool.h"
//#include "ctype.h"
//#include "string.h"


/**
 * 64-bit Write Access Datagram Structure
 * - Each byte is LSB..MSB
 * - Read is an identical 32-bit datagram with no data bits
 * - The read response is identical to the write datagram addressed to master address 0xFF
 *
 *  0xA_   _ _   _ _   _ _ _ _ _ _ _ _   _ _
 *
 * 	Sync + reserved 1 0 1 0 X X X X
 * 	8 bit slave address 0..3
 * 	7 bit register address + RW bit
 * 	32 bit data
 * 	8 bit CRC
 */

#define TMC2209_SYNC_BYTE	0b00000101

#define TMC2209_ADDR_0 		0b00000000
#define TMC2209_ADDR_1 		0b00000001
#define TMC2209_ADDR_2 		0b00000010
#define TMC2209_ADDR_3 		0b00000011

#define TMC2209_RW_READ 	0b00000000
#define TMC2209_RW_WRITE    0b00000001


/**
 * General Configuration Registers
 * 0x00..0x0F
 */
#define TMC2209_GCONF				0x00
#define TMC2209_I_scale_analog 		0b00000000000000000000000000000001	// 0: internal reference from 5VOUT 1: VREF
#define TMC2209_internal_rsense 	0b00000000000000000000000000000010	// 0: external sense resistors 1: internal
#define TMC2209_en_SpreadCycle 		0b00000000000000000000000000000100	// 0: Stealthchop	1: SpreadCycle
#define TMC2209_shaft				0b00000000000000000000000000001000	// 1: Inverse motor direction
#define TMC2209_index_otpw 			0b00000000000000000000000000010000	// 0: INDEX shows the first microstep position of sequencer
#define TMC2209_index_step 			0b00000000000000000000000000100000	// 1: INDEX output shows pulse each step
#define TMC2209_pdn_disable 		0b00000000000000000000000001000000	// 1: Disable PDN function, use this for UART
#define TMC2209_mstep_reg_select	0b00000000000000000000000010000000	// 0: Ms1, Ms2	1: MRES register
#define TMC2209_multistep_filt		0b00000000000000000000000100000000	// Filtering > 750Hz
#define TMC2209_test_mode			0b00000000000000000000001000000000	// 0 for normal operation, don't be a hero

#define TMC2209_GSTAT				0x01
#define TMC2209_reset				0b00000000000000000000000000000001
#define TMC2209_drv_err				0b00000000000000000000000000000010
#define TMC2209_uv_cp				0b00000000000000000000000000000100

#define TMC2209_IFCNT				0x02	// 0 - 255 Counts UART write accesses
#define TMC2209_IFCNT_DATA			0b00000000000000000000000011111111

#define TMC2209_SLAVECONF			0x03	// delay for read access, time until reply is sent
#define TMC2209_SENDDELAY_8			0b00000000000000000000000000000011 	// 8 bit times
#define TMC2209_SENDDELAY_3x8		0b00000000000000000000000000001100	// 3x8 bit times
#define TMC2209_SENDDELAY_5x8		0b00000000000000000000000000110000	// 5x8 bit times
#define TMC2209_SENDDELAY_7x8		0b00000000000000000000000011000000	// 7x8 bit times
#define TMC2209_SENDDELAY_9x8		0b00000000000000000000001100000000	// 9x8 bit times
#define TMC2209_SENDDELAY_11x8		0b00000000000000000000110000000000	// 11x8 bit times
#define TMC2209_SENDDELAY_13x8		0b00000000000000000011000000000000	// 13x8 bit times
#define TMC2209_SENDDELAY_15x8		0b00000000000000001100000000000000	// 15x8 bit times

#define TMC2209_OTP_PROG			0x04
#define TMC2209_OTPBIT				0b00000000000000000000000000000111
#define TMC2209_OTPBYTE				0b00000000000000000000000000110000
#define TMC2209_OTPMAGIC			0b00000000000000001111111100000000

#define TMC2209_OTP_READ			0x05
#define TMC2209_OTP_READ_OTP0		0b00000000000000000000000011111111
#define TMC2209_OTP_READ_OTP1		0b00000000000000001111111100000000
#define TMC2209_OTP_READ_OTP2		0b00000000111111110000000000000000
/*
 * ToDo Add OTP configuration memory section 5.1.1
 */

#define TMC2209_IOIN				0x06	// Reads the state of all input pins available
#define TMC2209_IOIN_ENN			0b00000000000000000000000000000001
//#define TMC2209_IOIN_0			0b00000000000000000000000000000010
#define TMC2209_IOIN_MS1			0b00000000000000000000000000000100
#define TMC2209_IOIN_MS2			0b00000000000000000000000000001000
#define TMC2209_IOIN_DIAG			0b00000000000000000000000000010000
//#define TMC2209_IOIN_0			0b00000000000000000000000000100000
#define TMC2209_IOIN_PDN_UART		0b00000000000000000000000001000000
#define TMC2209_IOIN_STEP			0b00000000000000000000000010000000
#define TMC2209_IOIN_SPREAD_EN		0b00000000000000000000000100000000
#define TMC2209_IOIN_DIR			0b00000000000000000000001000000000
#define TMC2209_IOIN_VERSION		0b11111111000000000000000000000000

#define TMC2209_FACTORY_CONF		0x07
#define TMC2209_FCLKTRIM			0b00000000000000000000000000011111 // 0..31 ~ 0..12MHz
#define TMC2209_OTTRIM				0b00000000000000000000001100000000

/**
 * Velocity Dependent Driver Feature Control Register Set
 * 0x10..0x1F
 */
#define TMC2209_IHOLD_IRUN			0x10 	// Driver current control
#define TMC2209_IHOLD				0b00000000000000000000000000011111 // 0=1/32 .. 31=32/32 , 0 = Free wheel / passive braking
#define TMC2209_IRUN				0b00000000000000000001111100000000 // 0=1/32 .. 31=32/32 , 16/32 is ideal for microstep performance
#define TMC2209_IHOLDDELAY			0b00000000000011110000000000000000 // 0=Instant power down, 1..15 delay in multiple of 2^18 clocks

#define TMC2209_TPOWERDOWN			0x11	// Delay from standstill to current power down
#define TMC2209_TPOWERDOWN_DATA		0b00000000000000000000000011111111	// 0…((2^8)-1) * 2^18 tCLK ~ 0 to 5.6 seconds

/*
 *  TMC2209_TPWMTHRS_TIME > TMC2209_TSTEP_TIME = SpreadCycle
 * 	TMC2209_TPWMTHRS_TIME =< TMC2209_TSTEP_TIME = StealthChop
 */
#define TMC2209_TSTEP				0x12 	// Read actual measured time between two 1/256 microsteps
#define TMC2209_TSTEP_DATA			0b00000000000111111111111111111111	// 0..(2^20)-1

#define TMC2209_TPWMTHRS			0x13	// Set upper velocity for StealthChop voltage PWM mode
#define TMC2209_TPWMTHRS_DATA		0b00000000000111111111111111111111	// 0..(2^20)-1

#define TMC2209_VACTUAL				0x22	// 0: Respond to step input, else use VACTUAL
#define TMC2209_VACTUAL_DATA		0b00000001111111111111111111111111	// +/- (2^23)-1 [usteps/t]

// LEFT OFF HERE, section 5.3 stallguard control







/** Chopper Register Set */
#define TMC2209_register_set

//CoolStep and StallGuard Control Registers */
#define TMC2209_coolstep_stallguard





/**
 * Data types and enums
 */






typedef struct {
	// HW Interface
//	uint16_t stepPin;		// THESE NEEDC TO BE TIMER NOT GPIO
//	GPIO_TypeDef *stepPort;
	TIM_HandleTypeDef *stepTimer; /** Pointer to timer channel */
	uint32_t stepTimerChannel;

	uint16_t dirPin;
	GPIO_TypeDef *dirPort;

	UART_HandleTypeDef *uart;
	uint8_t uartAddr; // 0x00 to 0x03, set with hardware jumpers

	/*
	 * 32-bit register data
	 */
	// General registers
	uint32_t gconf;
	uint32_t gstat;
	uint32_t ifcnt;
	uint32_t slaveconf;
	uint32_t otp_prog;
	uint32_t otp_read;
	uint32_t ioin;
	uint32_t factory_conf;
	// Velocity dependent registers
	uint32_t ihold_irun;
	uint32_t tpower_down;
	uint32_t tstep;






} tmc2209_t;

/**
 * UART Message Structs
 */
#define TMC2209_UART_TIMEOUT	10 // in ms
#define TMC2209_WRITE_DATAGRAM_LENGTH 8 // in Bytes
#define TMC2209_READ_REQUEST_DATAGRAM_LENGTH 4 // in Bytes
#define TMC2209_READ_RESPONSE_DATAGRAM_LENGTH 8 // in Bytes


typedef struct {
	uint8_t slaveAddress;
	uint8_t registerAddress;
	uint32_t data;
}tmc2209_write_t;

typedef struct {
	uint8_t slaveAddress;
	uint8_t registerAddress;
}tmc2209_read_request_t;

typedef struct {
	uint8_t slaveAddress;
	uint8_t registerAddress;
	uint32_t data;
}tmc2209_read_response_t;


/**
 * Exported functions
 */


tmc2209_t tmc2209_new(
		TIM_HandleTypeDef *stepTimer, uint32_t stepTimerChannel,
		uint16_t dirPin, GPIO_TypeDef *dirPort,
		UART_HandleTypeDef *uart, uint8_t uartAddr);
/**
 * UART
 */
uint32_t tmc2209_read(tmc2209_t *tmc, tmc2209_read_request_t readDatagram);
void tmc2209_write(tmc2209_t *tmc, tmc2209_write_t writeDatagram);

/**
 * Read and write registers
 */
void tmc2209_setGCONFG(tmc2209_t *tmc, uint32_t data);

void tmc2209_setIHOLD_IRUN(tmc2209_t *tmc, uint32_t data);
void tmc2209_setTPOWERDOWN(tmc2209_t *tmc, uint32_t data);
void tmc2209_setTSTEP(tmc2209_t *tmc, uint32_t data);



/**
 * Speed and control
 */

void tmc2209_step(tmc2209_t *tmc);

void tmc2209_on(tmc2209_t *tmc);
void tmc2209_off(tmc2209_t *tmc);




void tmc2209_reset(tmc2209_t *tmc);
void tmc2209_setRSense(tmc2209_t *tmc, uint8_t data);
void tmc2209_setCurrent(tmc2209_t *tmc, double mACurrent);
void tmc2209_setMode(tmc2209_t *tmc, uint8_t mode);
void tmc2209_setMicroStep(tmc2209_t *tmc, uint8_t step);
void tmc2209_resetCounter(tmc2209_t *tmc);

void tmc2209_calculateCRC(uint8_t* datagram, uint8_t datagramLength);




#endif /* INC_TMC2209_H_ */
