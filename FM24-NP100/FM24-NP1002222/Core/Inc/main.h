/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm4.h"
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SIZE_BUFFER_ADC 128
#define SIZE_UART_RX 4

//#define START_TX "STAR"
//#define STOP_TX "STOP"
//#define TEST_TX "TEST"
//#define RESET_TX "RESE"
//#define RAMP1_TX "RAM1"
//#define RAMP2_TX "RAM2"
//#define AMPL_TX "AM"


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ADC1_2_Dual_Init(void);
void DMA1_Channel1_IRQHandler(void);
void DMA2_Channel3_IRQHandler(void);
void TIM8_UP_IRQHandler(void);
void TIM8_Init(void);
void TIM2_IRQHandler(void);
void TIM2_Init(void);
void DAC1_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void send_data_via_uart(uint8_t* data, uint16_t length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

struct message_ADC
{
	uint32_t preamble;
	uint32_t BUFF[SIZE_BUFFER_ADC*50];  //was 57 (for 14)
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
