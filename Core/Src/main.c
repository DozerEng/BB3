/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdint.h"

#include "led.h"
#include "rgb.h"
#include "button.h"
#include "servo.h"
#include "eezybotarm.h"
#include "tmc2209.h"
#include "icm20608.h"
#include "bb3.h"
#include "circularBuffer.h"

//#include "math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define USB_BUFLEN	128


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */




//uint8_t usbTxBuf[USB_BUFLEN];
//uint16_t usbTxBufLen;
//
//uint8_t usbRxBuf[USB_BUFLEN];
//uint16_t usbRxBufLen;
//uint8_t	usbRxFlag = 0;

//void USB_RXCallback(uint8_t* Buf, uint32_t* Len) {
//	memcpy(usbRxBuf, Buf, *Len);
//	usbRxBufLen = *Len;
//	usbRxFlag = 1;
//}


/* Buffer sizes */
/* Must be a power of 2 */
#define USB_BUFFER_SIZE 64
#define UART_BUFFER_SIZE 64

/* Actual memory arrays */
static uint8_t usbRxMemory[USB_BUFFER_SIZE];
static uint8_t usbTxMemory[USB_BUFFER_SIZE];
static uint8_t uartRxMemory[UART_BUFFER_SIZE];
static uint8_t uartTxMemory[UART_BUFFER_SIZE];

/* CircularBuffer_t instances */
CircularBuffer_t usbRxBuffer;//  = circularBuffer_new(usbRxMemory, USB_BUFFER_SIZE);
CircularBuffer_t usbTxBuffer;//  = circularBuffer_new(usbTxMemory, USB_BUFFER_SIZE);
CircularBuffer_t uartRxBuffer;// = circularBuffer_new(uartRxMemory, UART_BUFFER_SIZE);
CircularBuffer_t uartTxBuffer;// = circularBuffer_new(uartTxMemory, UART_BUFFER_SIZE);

typedef struct
{
    CircularBuffer_t *rxBuffer;
    CircularBuffer_t *txBuffer;

    void (*startTx)(void);

} CommInterface_t;

CommInterface_t usbInterface;
CommInterface_t uartInterface;


/* Comms interfaces */


//typedef struct
//{
//    CircularBuffer_t *rxBuffer;
//    CircularBuffer_t *txBuffer;
//
//    void (*startTx)(void);
//
//} CommInterface_t;
//
//
//CommInterface_t usbInterface =
//{
//    .rxBuffer = &usbRxBuffer,
//    .txBuffer = &usbTxBuffer,
//    .startTx = usb_startTx // maybe a flag, maybe a function?
//
//};
//
//
//CommInterface_t uartInterface =
//{
//    .rxBuffer = &usbRxBuffer,
//    .txBuffer = &usbTxBuffer,
//    .startTx = uart_startTx
//};
//
//
//


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*
 *
 * 	Events and loops
 * 		- Events run once a loop
 * 		- Loops are blocking and run forever
 *
 */




// Intro tasks
//void rgb_green_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
//void rgb_white_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
//void rgb_intro_task(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfSteps);
//
//// Button tasks
//void tmc2209_3_button_task(
//		uint32_t period,
//		rgb_t *rgb1, rgb_t *rgb2,
//		button_t *topPB, button_t *midPB, button_t *botPB,
//		tmc2209_t *tmc1, tmc2209_t *tmc2);
//void tmc2209_icm20608_3_button_task(
//		uint32_t period,
//		rgb_t *rgb1, rgb_t *rgb2,
//		button_t *topPB, button_t *midPB, button_t *botPB,
//		icm20608_t *icm,
//		tmc2209_t *tmc1, tmc2209_t *tmc2);
//
//// Logging tasks
//void tmc2209_log_task(uint32_t period, tmc2209_t *tmc);
//void icm20608_log_task(uint32_t period, icm20608_t *icm);
//
//// Program task(s)
//
//void bb3_loop(
//		uint32_t period,
//		rgb_t *rgb1, rgb_t *rgb2,
//		button_t *topPB, button_t *midPB, button_t *botPB,
//		icm20608_t *imu,
//		tmc2209_t *rightWheel, tmc2209_t *leftWheel
// 	 );
//
// // Heartbeat(s)
//void heartbeat_task(
//		led_t *led1,
//		uint32_t led1Period,
//		led_t *led2,
//		uint32_t led2Period);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_ADC2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */



  usbRxBuffer  = circularBuffer_new(usbRxMemory, USB_BUFFER_SIZE);
  usbTxBuffer  = circularBuffer_new(usbTxMemory, USB_BUFFER_SIZE);
  uartRxBuffer = circularBuffer_new(uartRxMemory, UART_BUFFER_SIZE);
  uartTxBuffer = circularBuffer_new(uartTxMemory, UART_BUFFER_SIZE);

  /* Comms interfaces */





//  usbInterface =
//  {
//      .rxBuffer = &usbRxBuffer,
//      .txBuffer = &usbTxBuffer,
//      .startTx = usb_startTx // maybe a flag, maybe a function?
//
//  };
//
//
//  uartInterface =
//  {
//      .rxBuffer = &usbRxBuffer,
//      .txBuffer = &usbTxBuffer,
//      .startTx = uart_startTx
//  };





  /*
   *  Servo motors
   */
  servo_t servo1 = Servo_newMG90S(&htim1, TIM_CHANNEL_2);
  servo_t servo2 = Servo_newMG90S(&htim1, TIM_CHANNEL_1);
  servo_t servo3 = Servo_newMG90S(&htim8, TIM_CHANNEL_4);
  servo_t servo4 = Servo_newMG90S(&htim8, TIM_CHANNEL_3);
  servo_t servo5 = Servo_newMG90S(&htim8, TIM_CHANNEL_2);
  servo_t servo6 = Servo_newMG90S(&htim8, TIM_CHANNEL_1);

  /*
   * Status LEDs
   */
  led_t topStatusLed = led_new(
      STATUS_LED1_Pin,
      STATUS_LED1_GPIO_Port,
      1);
  led_t botStatusLed = led_new(
      STATUS_LED2_Pin,
      STATUS_LED2_GPIO_Port,
      1);

  /**
   *  RGB LEDs
   */
  rgb_t rgb1 = rgb_new(
      LED1_R_Pin,
      LED1_R_GPIO_Port,
      LED1_G_Pin,
      LED1_G_GPIO_Port,
      LED1_B_Pin,
      LED1_B_GPIO_Port,
      RGB_OFF,
      1);
  rgb_t rgb2 = rgb_new(
      LED2_R_Pin,
      LED2_R_GPIO_Port,
      LED2_G_Pin,
      LED2_G_GPIO_Port,
      LED2_B_Pin,
      LED2_B_GPIO_Port,
      RGB_OFF,
      1);

  /**
   *  Buttons
   */
  button_t topPb = button_new(
      TOP_PB_Pin,
      TOP_PB_GPIO_Port,
      0);
  button_t midPb = button_new(
	MID_PB_Pin,
	MID_PB_GPIO_Port,
	0);
  button_t botPb = button_new(
    BOT_PB_Pin,
    BOT_PB_GPIO_Port,
    0);

  /**
   * Stepper motor drivers
   */
  icm20608_t icm = icm20608_new(
    IMU_INTERRUPT_Pin,
    IMU_INTERRUPT_GPIO_Port);

  /**
   * Stepper motor drivers
   */
  tmc2209_t rightMotor = tmc2209_new(
    TMC2209_VELOCITY_CONTROL,
	TMC2209_INVERSE_MOTOR_DIR,
    &htim2,
	TIM_CHANNEL_2,
	RIGHT_DIR_Pin,
	RIGHT_DIR_GPIO_Port,
	&huart1,
	TMC2209_ADDR_1);

  tmc2209_t leftMotor = tmc2209_new(
	TMC2209_VELOCITY_CONTROL,
	TMC2209_STANDARD_MOTOR_DIR,
    &htim2,
    TIM_CHANNEL_1,
    LEFT_DIR_Pin,
    LEFT_DIR_GPIO_Port,
    &huart1,
    TMC2209_ADDR_2);

 bb3_t bb3 = bb3_new(
	  &rgb1, &rgb2,
	  &rgb1, &rgb2, // event LED: button, process
	  &topPb, &midPb, &botPb,
	  &leftMotor, &rightMotor,
	  true,
	  BB3_DIRECTION_FORWARD,
	  BB3_SPEED_RPM,
	  1.0 // acceleration
	  );

 //  eezybotarm_t robotArm = eezybotarm_new(&servo6, &servo3, &servo4, &servo5, &rgb1, &rgb2);





  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Delay to wait for USB to be ready
  // TODO wait until USB is ready for transmission, Note: CDC_Transit_FS returns success or failure
  //  HAL_Delay(250);

  /*
   * Start of infinite loop!
   */


  // Intro tasks
  rgb_intro_task(&rgb1, &rgb2, 20, 40);
//  HAL_Delay(100);
// rgb_green_blink(&rgb1, &rgb2, 100, 5);
   rgb_white_blink(&rgb1, &rgb2, 100, 3);


  while (1)
  {

	  /*
	   *  System tasks
	   */

//	  if(usbRxFlag) {
//		  char initializingMessage[50];
//		  usbRxBuf[usbRxBufLen] = '\0';
//		  snprintf(initializingMessage, 50, "Received: %d bytes containing %s\n\r", usbRxBufLen, usbRxBuf);
//		  CDC_Transmit_FS((uint8_t*)initializingMessage, strlen(initializingMessage));
//		  usbRxFlag = 0;
//		  usbRxBufLen = 0;
//	  }
//		char initializingMessage[50];
//		uint32_t i = HAL_GetTick();
//		sprintf(initializingMessage, "Tick: %lu\n\r", i);
//		CDC_Transmit_FS((uint8_t*)initializingMessage, strlen(initializingMessage));
//		char command = (char) CDC_Get_Buffer()[0];
//		buffer = (char)CDC_Get_Buffer();
//		while (buffer[i] != 0) {
//			if ( buffer[i] == 0) {
//				i = 0;
//			}
//			// Print out contents and reset register to 0
//			CDC_Transmit_FS((uint8_t*)buffer, 1);
//			buffer[i] = 0;
//			i ++;
//		}
//		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
//		HAL_Delay(100);
//		command = tolower(command);
//		switch(command){
//			// Moving base
//			case 'q':




//		  uint8_t Test[] = "Hello World !!!\r\n"; //Data to send
//		  HAL_UART_Transmit(&huart2,Test,sizeof(Test),10);// Sending in normal mode
//
//		  char msg[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
//		  uint8_t status;
////		  status = HAL_UART_Transmit(&huart2, msg, 6, 10);
//		  HAL_UART_Transmit(&huart1, msg, 6, 10);


//		  tmc2209_read_request_t readDatagram = {
//		  			.slaveAddress = TMC2209_ADDR_1,
//		  			.registerAddress = TMC2209_GCONF
//		  	};
//		  uint32_t data = tmc2209_read(&rightMotor, readDatagram);
//		  char msg[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
//
//		  uint8_t status;
//		  status = HAL_UART_Transmit(&huart2, &data, 4, 10);
//		  HAL_UART_Transmit(&huart1, msg, 6, 10);


//		  char msg[] = {0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00};
//		  uint8_t status;
//		  status = HAL_UART_Transmit(&huart2, msg, 18, 10);
//		  HAL_UART_Transmit(&huart1, msg, 18, 10);







	  /*
	   *  Button tasks
	   */

	   bb3_task_button(100, &bb3);

	   bb3_task_control_loop(10, &bb3);

//	  tmc2209_task_3_button(
//			  100,
//			  &rgb1, &rgb2,
//			  &topPb, &midPb, &botPb,
//			  &rightMotor, &leftMotor);


//	  tmc2209_task_icm20608_3_button(
//			  &rightMotor, &leftMotor,
//			  &icm,
//			  &topPB, &midPB, &botPB);



	  /*
	   *  Logging tasks
	   */
//	  tmc2209_log_task(&rightMotor);
//	  tmc2209_log_task(&leftMotor);
//	  icm20608_log_task(&icm);


	   /*
	    *  Heartbeat(s)
	    */
	  bb3_task_heartbeat(
			  &topStatusLed,
			  HEARTBEAT_TOP_LED_INTERVAL,
			  &botStatusLed,
			  HEARTBEAT_BOT_LED_INTERVAL);







	  /*
	   * EEZYBOTARM MK2
	   * - Un-comment the start and end comment blocks to enable/disable
	   */
	  /* Enable/Disable
	  eezybotarm_update_qawsedrf(&robotArm);

	  // Check buttons
	  uint8_t topPB, botPB, midPB;
	  topPB = HAL_GPIO_ReadPin(TOP_PB_GPIO_Port, TOP_PB_Pin);
	  midPB = HAL_GPIO_ReadPin(MID_PB_GPIO_Port, MID_PB_Pin);
	  botPB = HAL_GPIO_ReadPin(BOT_PB_GPIO_Port, BOT_PB_Pin);
	  // Set top to max, mid to mid, and bot to min servo values
	  if (topPB == PB_PRESSED) {
		  eezybotarm_setMode(&robotArm, eezybotarm_MODE_INCREMENT);
		  while(HAL_GPIO_ReadPin(TOP_PB_GPIO_Port, TOP_PB_Pin) == PB_PRESSED);
	  }
	  if (midPB == PB_PRESSED) {
		  if(robotArm.mode != eezybotarm_OFF) {
			  eezybotarm_setMode(&robotArm, eezybotarm_OFF);
		  } else {
			  eezybotarm_setMode(&robotArm, eezybotarm_NORMAL);
		  }
		  while(HAL_GPIO_ReadPin(MID_PB_GPIO_Port, MID_PB_Pin) == PB_PRESSED);
	  }
	  if (botPB == PB_PRESSED ) {
		  eezybotarm_setMode(&robotArm, eezybotarm_MODE_DECREMENT);
		  while(HAL_GPIO_ReadPin(BOT_PB_GPIO_Port, BOT_PB_Pin) == PB_PRESSED);
	  }

	  */ // EO EEZYBOTARM MK2



//	  HAL_Delay(WAIT_ms); // Delay if needed



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return EXIT_SUCCESS;
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV12;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
