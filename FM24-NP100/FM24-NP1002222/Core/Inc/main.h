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
#define SYS_CLOCK     72000000   // 72MHz
#define SIZE_BUFFER_ADC 128
#define SIZE_BUFFER_DAC 128
#define TIM2_ARR        500  //for DAC
#define TIM8_ARR        125  //for ADC
#define ADC_PER_DAC     TIM2_ARR/TIM8_ARR //#define ADC_PERIODS_PER_DAC_PERIOD
#define FREQ_DAC        SYS_CLOCK/TIM2_ARR
#define FREQ_ADC        SYS_CLOCK/TIM8_ARR
#define MAX_DAC_PERIODS  4 
#define MAX_ADC_PERIODS  MAX_DAC_PERIODS*ADC_PER_DAC
#define UART_BAUD_RATE 3000000
#define SIZE_UART_RX 4


enum
{
  START_COMMAND    = 1,
  STOP_COMMAND     = 2,
  RESET_COMMAND    = 3,
  TEST_COMMAND     = 4,
  RAMP1_COMMAND    = 5,
  RAMP2_COMMAND    = 6,
  AMPL_COMMAND     = 7
};

struct message_ADC
{
	uint32_t preamble;
	uint32_t BUFF[SIZE_BUFFER_ADC*MAX_ADC_PERIODS];  
};

struct flags
{
	uint32_t en_adc_dac : 1;
	uint32_t rx : 1;
	uint32_t data_adc_collect : 1;
	uint32_t adc_start : 1;
};


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
void TIM3_Init(void);
void TIM3_IRQHandler(void);
void DAC1_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Opamp_Start(OPAMP_TypeDef* opamp);
void Make_Ramp(uint8_t ramp, uint16_t ampl);
void Collect_ADC_Complete(struct flags flags_temp);
void Enable_DAC_ADC(struct flags flags_temp);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
