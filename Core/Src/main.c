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
#include "usb_device.h"

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

//#include "math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HEARTBEAT_TOP_LED_INTERVAL 	10		// Period in ms
#define HEARTBEAT_BOT_LED_INTERVAL 	250 	// Period in ms

#define TASK_RGB_TIMEOUT			1000	// in ms

// Pick which interfaces for logging and/or data collection
#define LOG_USB 	true
#define LOG_UART 	false


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/*
 *
 * 	Events and loops
 * 		- Events run once a loop
 * 		- Loops are blocking and run forever
 *
 */



// Intro tasks
void rgb_green_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
void rgb_white_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes);
void rgb_intro_task(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfSteps);

// Button tasks
void tmc2209_3_button_task(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		tmc2209_t *tmc1, tmc2209_t *tmc2);
void tmc2209_icm20608_3_button_task(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		icm20608_t *icm,
		tmc2209_t *tmc1, tmc2209_t *tmc2);

// Logging tasks
void tmc2209_log_task(uint32_t period, tmc2209_t *tmc);
void icm20608_log_task(uint32_t period, icm20608_t *icm);

// Program task(s)

void bb3_loop(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		icm20608_t *imu,
		tmc2209_t *rightWheel, tmc2209_t *leftWheel
 	 );

 // Heartbeat(s)
void heartbeat_task(
		led_t *led1,
		uint32_t led1Period,
		led_t *led2,
		uint32_t led2Period);

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
  button_t topPB = button_new(
	TOP_PB_Pin,
	TOP_PB_GPIO_Port,
	0);
  button_t midPB = button_new(
	MID_PB_Pin,
	MID_PB_GPIO_Port,
	0);
  button_t botPB = button_new(
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
	TMC2209_STANDARD_MOTOR_DIR,
    &htim2,
	TIM_CHANNEL_2,
	RIGHT_DIR_Pin,
	RIGHT_DIR_GPIO_Port,
	&huart1,
	TMC2209_ADDR_1);

  tmc2209_t leftMotor = tmc2209_new(
	TMC2209_VELOCITY_CONTROL,
	TMC2209_INVERSE_MOTOR_DIR,
    &htim2,
    TIM_CHANNEL_1,
    LEFT_DIR_Pin,
    LEFT_DIR_GPIO_Port,
    &huart1,
    TMC2209_ADDR_2);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // Delay to wait for USB to be ready
  // TODO wait until USB is ready for transmission, Note: CDC_Transit_FS returns success or failure
//  HAL_Delay(250);

  /*
   * Create new object
   * 	- Remove double-slashes to select desired firmware
   * 	- All other objects should be commented out
   */
//  eezybotarm_t robotArm = eezybotarm_new(&servo6, &servo3, &servo4, &servo5, &rgb1, &rgb2);
//  bb3_t bb3 = bb3_new(
//  ledDisplay_t display = ledDisplay_new(


  /*
   * Start of infinite loop!
   */


  // Intro tasks
  rgb_intro_task(&rgb1, &rgb2, 50, 40);
  HAL_Delay(500);
//  rgb_green_blink(&rgb1, &rgb2, 100, 5);
  rgb_white_blink(&rgb1, &rgb2, 100, 5);


  while (1)
  {

	  // Button tasks
	  tmc2209_3_button_task(
			  100,
			  &rgb1, &rgb2,
			  &topPB, &midPB, &botPB,
			  &rightMotor, &leftMotor);
//	  tmc2209_icm20608_3_button_task(
//			  &rightMotor, &leftMotor,
//			  &icm,
//			  &topPB, &midPB, &botPB);

	  // Logging tasks
//	  tmc2209_log_task(&rightMotor);
//	  tmc2209_log_task(&leftMotor);
//	  icm20608_log_task(&icm);

	  // Program task(s)

//	  bb3_loop(
//			  &rgb1, &rgb2,
//			  &topPB, &midPB, &botPB,
//			  &icm,
//			  &rightMotor, &leftMotor);

	   // Heartbeat(s)
	  heartbeat_task(
			  &topStatusLed,
			  HEARTBEAT_TOP_LED_INTERVAL,
			  &botStatusLed,
			  HEARTBEAT_BOT_LED_INTERVAL);






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

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 1;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 39999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 150;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 339;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 39999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_B_Pin|LED2_G_Pin|LED2_R_Pin|STATUS_LED1_Pin
                          |STATUS_LED2_Pin|LED1_R_Pin|LED1_G_Pin|LED1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_B_Pin LED2_G_Pin LED2_R_Pin STATUS_LED1_Pin
                           STATUS_LED2_Pin LED1_R_Pin LED1_G_Pin LED1_B_Pin */
  GPIO_InitStruct.Pin = LED2_B_Pin|LED2_G_Pin|LED2_R_Pin|STATUS_LED1_Pin
                          |STATUS_LED2_Pin|LED1_R_Pin|LED1_G_Pin|LED1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : nRST_Pin */
  GPIO_InitStruct.Pin = nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FDCAN1_FAULT_Pin FDCAN1_S_Pin */
  GPIO_InitStruct.Pin = FDCAN1_FAULT_Pin|FDCAN1_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_DIR_Pin */
  GPIO_InitStruct.Pin = RIGHT_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RIGHT_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_DIR_Pin */
  GPIO_InitStruct.Pin = LEFT_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEFT_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_SW_Pin TOP_PB_Pin MID_PB_Pin BOT_PB_Pin
                           IMU_INTERRUPT_Pin FDCAN2_FAULT_Pin FDCAN2_S_Pin */
  GPIO_InitStruct.Pin = LIMIT_SW_Pin|TOP_PB_Pin|MID_PB_Pin|BOT_PB_Pin
                          |IMU_INTERRUPT_Pin|FDCAN2_FAULT_Pin|FDCAN2_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Intro tasks
 */
void rgb_intro_task(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfSteps) {
	for (uint8_t i = 0; i < numberOfSteps; i++) {
		rgb_cycle(rgb1);
		rgb_reverse_cycle(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off between movements
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}


void rgb_green_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes) {
	for (uint8_t i = 0; i < numberOfFlashes; i++) {
		rgb_set_green(rgb1);
		rgb_set_green(rgb2);

		HAL_Delay(dwellTime);
		rgb_set_off(rgb1);
		rgb_set_off(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off to finish
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}


void rgb_white_blink(rgb_t *rgb1, rgb_t *rgb2, uint16_t dwellTime, uint8_t numberOfFlashes) {
	for (uint8_t i = 0; i < numberOfFlashes; i++) {
		rgb_set_white(rgb1);
		rgb_set_white(rgb2);

		HAL_Delay(dwellTime);
		rgb_set_off(rgb1);
		rgb_set_off(rgb2);
		HAL_Delay(dwellTime);
	}
	// Turn off to finish
	rgb_set_off(rgb1);
	rgb_set_off(rgb2);
}

/*
 * Button tasks
 */

void tmc2209_3_button_task(
	uint32_t period, // in ms
	rgb_t *rgb1,
	rgb_t *rgb2,
	button_t *topPB,
	button_t *midPB,
	button_t *botPB,
	tmc2209_t *tmc1,
	tmc2209_t *tmc2) {

	static uint32_t previousTick = 0;
	static uint32_t previousEventTick = 0;
	static uint32_t previousButtonPressTick = 0;

	uint32_t currentTick = HAL_GetTick();
	if(previousTick == 0){
		// If zero, it is the first time through
		previousTick = currentTick;
	}

	/*
	 * Top PB
	 */
	if(currentTick > (previousEventTick + period)) {
		previousEventTick = currentTick;
		button_read(topPB);
		if (topPB->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// Velocity Control
			tmc1->vactual += tmc1->acceleration ;
			tmc2209_set_VACTUAL(tmc1);
			tmc2->vactual += tmc1->acceleration ;
			tmc2209_set_VACTUAL(tmc2);

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_YELLOW) {
				rgb_set_red(rgb1);
				rgb_set_yellow(rgb2);
			} else {
				rgb_set_yellow(rgb1);
				rgb_set_red(rgb2);
			}
		}

		/*
		 * Mid PB
		 */
		button_read(midPB);
		if (midPB->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// Velocity Control
			if (tmc1->mode == TMC2209_VELOCITY_CONTROL) {
				tmc1->vactual -= tmc1->acceleration ;
				tmc2209_set_VACTUAL(tmc1);
			}
			if(tmc2->mode == TMC2209_VELOCITY_CONTROL) {
				tmc2->vactual -= tmc2->acceleration ;
				tmc2209_set_VACTUAL(tmc2);
			}

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_BLUE) {
				rgb_set_green(rgb1);
				rgb_set_blue(rgb2);
			} else {
				rgb_set_blue(rgb1);
				rgb_set_green(rgb2);
			}
		}


		/*
		 * Bot PB - Braking
		 */

		button_read(botPB);
		if (botPB->currentState == BUTTON_PRESSED) {
			previousButtonPressTick = currentTick;
			// VACTUAL control
			if((tmc1->vactual < (5* tmc1->acceleration)) && tmc1->vactual > (-5 * tmc1->acceleration)) {
				tmc1->vactual = 0x000000;
			} else {
				if (tmc1->vactual > 0) {
					tmc1->vactual -= 5*tmc1->acceleration;
				} else {
					tmc1->vactual += 5*tmc1->acceleration;
				}
			}
			tmc2209_set_VACTUAL(tmc1);


			// VACTUAL control
			if((tmc2->vactual < (5*tmc2->acceleration)) && (tmc2->vactual > (-5 * tmc2->acceleration ))) {
				tmc2->vactual = 0x000000;
			} else {
				if (tmc2->vactual > 0) {
					tmc2->vactual -= 5*tmc2->acceleration;
				} else {
					tmc2->vactual += 5*tmc2->acceleration;
				}
			}
			tmc2209_set_VACTUAL(tmc2);

			// Toggle LEDs to signal change
			if(rgb1->currentColor == RGB_VIOLET) {
				rgb_set_turquoise(rgb1);
				rgb_set_violet(rgb2);
			} else {
				rgb_set_violet(rgb1);
				rgb_set_turquoise(rgb2);
			}

		}

	}
	// If there is not button activity, turn off LEDs
	if(currentTick > (previousButtonPressTick + TASK_RGB_TIMEOUT)) {
		rgb_set_off(rgb1);
		rgb_set_off(rgb2);
	}


}

void tmc2209_icm20608_3_button_task(
		uint32_t period,
		rgb_t *rgb1,
		rgb_t *rgb2,
		button_t *topPB,
		button_t *midPB,
		button_t *botPB,
		icm20608_t *icm,
		tmc2209_t *tmc1,
		tmc2209_t *tmc2) {

	// Top PB
	button_read(topPB);
	while (topPB->currentState == topPB->pressedState) {


		button_read(topPB);
	}

	// Middle PB
	button_read(midPB);
	while (midPB->currentState == midPB->pressedState) {


		button_read(topPB);
	}

	// Bottom PB
	button_read(botPB);
	while (botPB->currentState == botPB->pressedState) {


		button_read(botPB);
	}
}

 /*
  * Logging tasks
  */
void tmc2209_log_task(uint32_t period, tmc2209_t *tmc) {
	if(LOG_USB == true) {

	}

	if(LOG_UART ==	false) {

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


	}
}

void icm20608_log_task(uint32_t period, icm20608_t *icm) {
	if(LOG_USB == true) {
		//ToDo: this
		return;
	}

	if(LOG_UART ==	false) {
		//ToDo: this
		return;
	}
}


/*
 * Program task(s)
 */
void bb3_loop(
		uint32_t period,
		rgb_t *rgb1, rgb_t *rgb2,
		button_t *topPB, button_t *midPB, button_t *botPB,
		icm20608_t *imu,
		tmc2209_t *rightWheel, tmc2209_t *leftWheel
 	 ) {
	// ToDo; This
	return;

}

/*
 * Heartbeat(s)
 */
void heartbeat_task(
	led_t *led1,
	uint32_t led1Period,
	led_t *led2,
	uint32_t led2Period) {
	static uint32_t led1PreviousEvent = 0;
	static uint32_t led2PreviousEvent = 0;

	uint32_t currentTick = HAL_GetTick();

	// If it's the first run, set PreviousEvent variable to current tick
	if(led1PreviousEvent == 0) {
		led1PreviousEvent = currentTick;
		led2PreviousEvent = currentTick;
	}

	// Toggle system LEDs every period
	if (currentTick > (led1PreviousEvent + led1Period)) {
		led1PreviousEvent = currentTick;
		led_toggle(led1);
	}
	if (currentTick > (led2PreviousEvent + led2Period)) {
		led2PreviousEvent = currentTick;
		led_toggle(led2);
	}

}





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

#ifdef  USE_FULL_ASSERT
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
