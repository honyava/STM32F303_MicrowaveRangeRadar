/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
//extern uint8_t flag_dma_half;
extern volatile uint32_t flag_dma_complete;
extern volatile uint8_t flag_tx;
extern volatile uint8_t flag_rx;
extern volatile uint32_t flag_dac;
extern volatile uint32_t flag_dac_count;
extern volatile uint8_t period_number_DAC;
extern volatile uint32_t flag_trans;
extern volatile uint32_t BUFF_ADC1_2[SIZE_BUFFER_ADC];
extern uint8_t UART_command[SIZE_UART_RX];
extern volatile uint8_t firstByteWait;

extern struct message_ADC message_ADC12;

uint16_t timeOut = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(firstByteWait != 0)
	{
		timeOut = 0; 
	}
  else 
	{
    timeOut++;
    if( timeOut >= 50 )
		{
      HAL_UART_AbortReceive_IT(&huart1);
      firstByteWait = 1; 
      timeOut = 0;
      HAL_UART_Receive_IT(&huart1, UART_command, 1);
    }
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void DMA1_Channel1_IRQHandler(void) // for ADC1_2 (dual)
{
	if(READ_BIT(DMA1->ISR, DMA_ISR_HTIF1)) // half transfer complete
	{
		//DMA1->IFCR |= DMA_IFCR_CGIF1;
		SET_BIT(DMA1->IFCR, DMA_IFCR_CHTIF1_Msk); // Resetting the flag of interrupt
		if ((flag_dac == 1) && (flag_dac_count <= period_number_DAC))
		{
			for(uint32_t i = 0; i < SIZE_BUFFER_ADC/2; i++)
			{
				message_ADC12.BUFF[i + (SIZE_BUFFER_ADC*flag_dma_complete)] = BUFF_ADC1_2[i];
			}
			//flag_tx = 0;
		}
	}	
	if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF1)) // transfer complete
	{
	//	DMA1->IFCR |= DMA_IFCR_CGIF1;
		SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF1_Msk); // Resetting the flag of interrupt
		if ((flag_dac == 1) && (flag_dac_count <= period_number_DAC))
		{		
			for(uint32_t i = SIZE_BUFFER_ADC/2; i < SIZE_BUFFER_ADC; i++)
			{
				message_ADC12.BUFF[i + (SIZE_BUFFER_ADC*flag_dma_complete)] = BUFF_ADC1_2[i];
			}
			flag_dma_complete++;
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
	}
}

void DMA2_Channel3_IRQHandler(void) // for DAC1
{
	if(READ_BIT(DMA2->ISR, DMA_ISR_TCIF3)) // transfer complete
	{
		SET_BIT(DMA2->IFCR, DMA_IFCR_CGIF3_Msk);
		//SET_BIT(DMA2->IFCR, DMA_IFCR_CTCIF3_Msk); // Resetting the flag of interrupt
		if (READ_BIT(TIM8->CR1, TIM_CR1_CEN_Msk))
		{  
			flag_dac = 1;
			flag_dac_count++;
      if(flag_dac_count > period_number_DAC)
      {
        CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN_Msk); // TIM8 disable
        flag_trans = 1;
        flag_dac = 0;
        //flag_dac_count = 0;
      }        
		}
		else
		{
			flag_dac = 0;
			flag_dac_count = 0;
			//flag_dma_complete = 0;
		}
	}
}

void TIM8_UP_IRQHandler(void) // for ADC1_2 (dual)
{
	if(READ_BIT(TIM8->SR, TIM_SR_UIF)) // check the flag of interrupt
	{
		TIM8->SR &= ~TIM_SR_UIF; // Resetting the flag of interrupt
	}
}

void TIM2_IRQHandler(void) // for DAC1
{
	if(READ_BIT(TIM2->SR, TIM_SR_UIF)) // check the flag of interrupt
	{
		TIM2->SR &= ~TIM_SR_UIF; // Resetting the flag of interrupt
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		flag_tx = 0;
//		flag_dma_complete = 0;
//		flag_trans = 0;
	}     
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		if (firstByteWait != 0)
		{	
			flag_rx = 1;
			firstByteWait = 0;
			HAL_UART_Receive_IT(&huart1, UART_command + 1, SIZE_UART_RX - 1);
		}
	}
}


/* USER CODE END 1 */
