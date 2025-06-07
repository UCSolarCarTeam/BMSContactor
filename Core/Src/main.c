/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include <MessageReader.h>
#include <Gatekeeper.h>
#include <switchEnums.h>
#include <changeSwitch.h>
#include <makingCANMessage.h>
#include <SendingCANMessage.h>
#include <string.h>  // Include this header for memcpy

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTACTOR_TYPE COMMON
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// initializing variables for receiving data
CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8]; // CAN data max size is 8 bytes
volatile uint8_t messageFlag = 0; // flag for if the CAN message is meant for the MBMS
CAN_Message received_message;

// initializing variables for sending data
uint32_t state_status;
uint8_t TxData[8];
uint8_t heartData[8];
uint16_t heartbeat;

// initializing buffer for DMA
uint16_t rawValues[2];
uint32_t heartbeat_extendedIDE = 0x200 + CONTACTOR_TYPE;

SwitchInfo_t contactor =
	{
		.GPIO_Port = Contactor_ON_Output_Pin,
		.GPIO_Pin = Contactor_ON_Output_GPIO_Port,
		.GPIO_Port_Sense = Contactor_Aux_Input_GPIO_Port,
		.GPIO_Pin_Sense = Contactor_Aux_Input_Pin,
		.GPIO_State = GPIO_PIN_RESET, // All pins except the common should start off as open. Reset = 0
		.Switch_State = OPEN, // All pins except the common should start off as open.
	    .switchError = false,
		.BPSError = false,
//		.Delay = 8000*250, // NEEDS A DELAY OF ABOUT A 1/4 OF A SECOND
		.Delay = 250, // NEEDS A DELAY OF ABOUT A 1/4 OF A SECOND
		.WhichContactor = CONTACTOR_TYPE, // we need to define this separately on each board!!!!!!!!! **
		.extendedID = 0x210 + CONTACTOR_TYPE, // sets our extendedID to 0x200 + the number corresponding to each contactor (i.e motor will be 0x200 + 1 = 0x201) **
//		.resistance = 6.6, // WILL CHANGE BASED ON CONACTOR **
		.isContactor = 1,
		.lineCurrentAmpsPerADCVoltage = 50  // WILL CHANGE BASED ON CONACTOR ** can be 100!!! or 30!!!
	};
SwitchInfo_t precharger;


void checkState(){
	// check our contactor
	contactor.Switch_State = HAL_GPIO_ReadPin(contactor.GPIO_Port_Sense, contactor.GPIO_Pin_Sense);
	if (contactor.Switch_State == CLOSED){
		contactor.GPIO_State = GPIO_PIN_SET; // set the pin
	}else{
		contactor.GPIO_State = GPIO_PIN_RESET; // ensure it's reset
	}
	// if we're not common, check the same thing for our contactor
	if (contactor.WhichContactor != COMMON){
		precharger.Switch_State = HAL_GPIO_ReadPin(precharger.GPIO_Port_Sense, precharger.GPIO_Pin_Sense);

		if (precharger.Switch_State == CLOSED){
			precharger.GPIO_State = GPIO_PIN_SET; // set the pin
		}else{
			precharger.GPIO_State = GPIO_PIN_RESET; // ensure it's reset
		}
	}


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Callback: timer has reset
  if (htim->Instance == TIM16)
  {
	  // how do we get resistance to convert voltage to current? V = IR
	  //LED Pin = turn on if contactor closed vice versa

	  // this is sent every 10 milliseconds
	  // add heartbeat!
	  	heartbeat++;
	  	if (heartbeat >= 65535){ // 2^16, changed it from 65536 to 65535
	  		heartbeat = 0;
	  	}

	  	// Store the 16-bit heartbeat into two bytes
	  	heartData[0] = (heartbeat >> 8) & 0xFF;  // High byte (bits 8-15)
	  	heartData[1] = heartbeat & 0xFF;         // Low byte (bits 0-7)

	  	// send heartbeat
	  	SendingCANMessage(heartbeat_extendedIDE, heartData, 2);

		// implement timer interrurpt (enable timer on ioc with new timer)
		// if no message is sent after 65 milliseconds (limit timer 16 can track), it will send a CAN message
		// send a CAN message!
		// we want to send the current state of the contactor and precharger
		state_status = makingCANMessage();
		// the payload we're sending
		TxData[0] = (state_status >> 24) & 0xFF;
		TxData[1] = (state_status >> 16) & 0xFF;
		TxData[2] = (state_status >> 8) & 0xFF;
		TxData[3] = state_status & 0xFF;

		// send the message
		SendingCANMessage(contactor.extendedID, TxData, 4);
  }
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  if(contactor.WhichContactor != COMMON){
	SwitchInfo_t precharger =
		{
			.GPIO_Port = PRECHARGE_ON_Output_GPIO_Port,
			.GPIO_Pin = PRECHARGE_ON_Output_Pin,
			.GPIO_Port_Sense = PRECHARGE_CURRENT_ADC_GPIO_Port,
			.GPIO_Pin_Sense = PRECHARGE_CURRENT_ADC_Pin,
			.Switch_State = OPEN, // All pins except the common should start off as open.
			.switchError = false,
	//		.Delay = 3000, // DOESN'T NEED A DELAY
			.resistance = 0.005, // WILL CHANGE BASED ON CONACTOR ** IT COULD 0.3!!!
			.threshold = 1, // WILL CHANGE BASED ON CONACTOR **
			.isContactor = 0,
			.derivative_threshold = 1 // WILL CHANGE BASED ON CONACTOR **
		};
  }
  checkState();

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);

  //start the timer
  HAL_TIM_Base_Start_IT(&htim16);
  //
  //  // start ADC+DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
  uint8_t msg[] = "Received\r\n";
  uint8_t msg2[] = "Message Not Received\r\n";

  uint32_t testID = 0x101;


  CAN_TxHeaderTypeDef canTxHeader;
	canTxHeader.IDE 	= CAN_ID_EXT;
	canTxHeader.RTR 	= CAN_RTR_DATA;
	canTxHeader.ExtId  	= testID;
	canTxHeader.DLC 	= 8;
  uint8_t counter = 0;
  uint32_t mailbox;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// WHEN YOU'RE INTIALIZING CHECK THE STATE!!!!!!!!!
		checkState();
		//	if (messageFlag == 1){
				received_message.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
				received_message.dlc = rx_header.DLC;
				received_message.is_extended = (rx_header.IDE == CAN_ID_EXT);
				received_message.is_rtr = (rx_header.RTR == CAN_RTR_REMOTE);
		//		memcpy(
				received_message.data[0] = 0b00000000;
				received_message.data[1] = 0b00000000;
				received_message.data[2] = 0b00000000;
				received_message.data[3] = 0b00000000;

				HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
				Gatekeeper(&received_message);
				messageFlag = 0;
		//	} else{
		//			HAL_UART_Transmit(&huart2, msg2, strlen(msg2), HAL_MAX_DELAY);
		//		}
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filterConfig_1;
  filterConfig_1.FilterBank = 0;                         // First filter bank
  filterConfig_1.FilterMode = CAN_FILTERMODE_IDMASK;     // Mask mode
  filterConfig_1.FilterScale = CAN_FILTERSCALE_32BIT;    // 32-bit scale
//  filterConfig_1.FilterIdHigh = 0x0000;				// filter id is the part we want to mask and sets the IDE bit to true so we can accept extended IDs
  filterConfig_1.FilterIdHigh = (0x101 >> 13) & 0xffff;				// filter id is the part we want to mask and sets the IDE bit to true so we can accept extended IDs

  filterConfig_1.FilterIdLow = ((0x101 & 0x1fff) << 3) | (1 << 2);
  filterConfig_1.FilterMaskIdHigh = 0;
  filterConfig_1.FilterMaskIdLow = 0;		
  filterConfig_1.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Put accepted msgs in FIFO 0
  filterConfig_1.FilterActivation = ENABLE;              // Enable the filter
  if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig_1) != HAL_OK) {
	Error_Handler();
  }

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 19;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Contactor_Aux_Input_Pin|PRECHARGE_ON_Output_Pin|Contactor_ON_Output_Pin|PRECHARGE_Sense_On_Output_Pin
                          |CAN1_Mode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Contactor_Aux_Input_Pin PRECHARGE_ON_Output_Pin Contactor_ON_Output_Pin PRECHARGE_Sense_On_Output_Pin
                           CAN1_Mode_Pin */
  GPIO_InitStruct.Pin = Contactor_Aux_Input_Pin|PRECHARGE_ON_Output_Pin|Contactor_ON_Output_Pin|PRECHARGE_Sense_On_Output_Pin
                          |CAN1_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIAG_N_Input_Pin */
  GPIO_InitStruct.Pin = DIAG_N_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIAG_N_Input_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, 0, &rx_header, rx_data);
	if (rx_header.ExtId == 0x101){
	// we need to have it so it does the parsing part here and then set a flag
		// make the message with the data we're going to give to the gatekeeper task:
		received_message.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
		received_message.dlc = rx_header.DLC;
		received_message.is_extended = (rx_header.IDE == CAN_ID_EXT);
		received_message.is_rtr = (rx_header.RTR == CAN_RTR_REMOTE);
		memcpy(received_message.data, rx_data, rx_header.DLC);
		// message data will look like 00000 - 11111, anything outside of that range is rubbish
		if (*received_message.data > -1 && *received_message.data < 32){
			//set flag
			messageFlag = 1;

		} // else, we received rubbish data, ignore it
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
