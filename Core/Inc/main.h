/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern CAN_HandleTypeDef hcan1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Contactor_Aux_Input_Pin GPIO_PIN_1
#define Contactor_Aux_Input_GPIO_Port GPIOA
#define PRECHARGE_ON_Output_Pin GPIO_PIN_4
#define PRECHARGE_ON_Output_GPIO_Port GPIOA
#define Contactor_ON_Output_Pin GPIO_PIN_5
#define Contactor_ON_Output_GPIO_Port GPIOA
#define DIAG_N_Input_Pin GPIO_PIN_6
#define DIAG_N_Input_GPIO_Port GPIOA
#define PRECHARGE_Sense_On_Output_Pin GPIO_PIN_7
#define PRECHARGE_Sense_On_Output_GPIO_Port GPIOA
#define CAN1_Mode_Pin GPIO_PIN_8
#define CAN1_Mode_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
