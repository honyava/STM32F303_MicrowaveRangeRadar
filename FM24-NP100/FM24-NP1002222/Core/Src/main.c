/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;
OPAMP_HandleTypeDef hopamp4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

enum {
  START    = 1,
  STOP     = 2,
  RESET_C  = 3,
  TEST     = 4,
  RAMP1    = 5,
  RAMP2    = 6,
  AMPL     = 7
};

const uint16_t Triangle_DAC[128] = {0,	64,	128,	192,	256,	320,	384,	448,	512,	576,	640,	704,	768,
	832,	896,	960,	1024,	1088,	1152,	1216,	1280,	1344,	1408,	1472,	1536,	1600,	1664,	1728,	1792,	1856,
	1920,	1984,	2048,	2111,	2175,	2239,	2303,	2367,	2431,	2495,	2559,	2623,	2687,	2751,	2815,	2879,	2943,
	3007,	3071,	3135,	3199,	3263,	3327,	3391,	3455,	3519,	3583,	3647,	3711,	3775,	3839,	3903,	3967,	4031,
	4095,	4031,	3967,	3903,	3839,	3775,	3711,	3647,	3583,	3519,	3455,	3391,	3327,	3263,	3199,	3135,	3071,
	3007,	2943,	2879,	2815,	2751,	2687,	2623,	2559,	2495,	2431,	2367,	2303,	2239,	2175,	2111,	2048,	1984,
	1920,	1856,	1792,	1728,	1664,	1600,	1536,	1472,	1408,	1344,	1280,	1216,	1152,	1088,	1024,	960,	896,
	832,	768,	704,	640,	576,	512,	448,	384,	320,	256,	192,	128,	64
	};

//const uint16_t Triangle_DAC[128] = {0,	32,	64,	96,	128,	160,	192,	224,	256,	288,	320,	352,	384,
//	416,	448,	480,	512,	544,	576,	608,	640,	672,	704,	736,	768,	800,	832,	864,	896,	928,	
//	960,	992,	1024,	1056,	1088,	1120,	1152,	1184,	1216,	1248,	1280,	1312,	1344,	1376,	1408,	1440,	1472,	
//	1504,	1536,	1568,	1600,	1632,	1664,	1696,	1728,	1760,	1792,	1824,	1856,	1888,	1920,	1952,	1984,	2016,
//	2048,	2079,	2111,	2143,	2175,	2207,	2239,	2271,	2303,	2335,	2367,	2399,	2431,	2463,	2495,	2527,	2559,
//	2591,	2623,	2655,	2687,	2719,	2751,	2783,	2815,	2847,	2879,	2911,	2943,	2975,	3007,	3039,	3071,	3103,	
//	3135,	3167,	3199,	3231,	3263,	3295,	3327,	3359,	3391,	3423,	3455,	3487,	3519,	3551,	3583,	3615,	3647,
//	3679,	3711,	3743,	3775,	3807,	3839,	3871,	3903,	3935,	3967,	3999,	4031,	4063
//	};
uint16_t Triangle_DAC3[128] = {0,};
	
volatile uint32_t BUFF_ADC1_2[SIZE_BUFFER_ADC] = {0,};
volatile uint32_t flag_dma_complete = 0;
volatile uint32_t flag_dac = 0;	
volatile uint32_t flag_dac_count = 0;	
volatile uint32_t flag_tx = 0;
volatile uint32_t flag_rx = 0;
volatile uint32_t flag_trans = 0;
volatile uint32_t flag_adc = 0;
volatile uint8_t period_number_DAC = 0;
	
const uint8_t start_byte = 0x01;	

volatile uint8_t period_number = 0;
volatile uint16_t message_size = 0;	
	
uint8_t UART_command[SIZE_UART_RX];
volatile uint8_t firstByteWait = 0;
volatile uint16_t k_ramp = 0;
volatile uint16_t Ampl = 500;

struct message_ADC message_ADC12 = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_OPAMP4_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_OPAMP4_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_OPAMP_Start(&hopamp4);
	
	ADC1_2_Dual_Init();
	DAC1_Init();
	TIM2_Init();
	TIM8_Init();
	TIM3_Init();
	
	firstByteWait = 1;
	HAL_UART_Receive_IT(&huart1, UART_command, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//UART_command_convert = UART_command[0] + (UART_command[1] << 8);
		if ((UART_command[0] == START) && (UART_command[1] != 0))  //&& (UART_command[1] <= 14)
		{
			HAL_UART_AbortReceive_IT(&huart1);
			period_number_DAC = UART_command[1];
			if (flag_trans == 1) // to do
			{
				flag_adc = 0;
				UART_command[0] = 0;
				UART_command[1] = 0;
				period_number = flag_dac_count - 1;
				message_size = SIZE_BUFFER_ADC*flag_dma_complete*4; //bytes

				message_ADC12.preamble = (start_byte) | (period_number << 8) | (message_size << 16);
				flag_dma_complete = 0;
				flag_trans = 0;
				flag_dac_count = 0;
				flag_tx = 1;
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)&message_ADC12,  message_size + 4);
			}
		}
		else if(UART_command[0] == STOP)
		{
			UART_command[0] = 0;
			CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN_Msk); // TIM8 disable
			//CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN_Msk); // TIM2 disable
		}
		else if(UART_command[0] == RESET_C)
		{
			HAL_NVIC_SystemReset();
		}
		else if(UART_command[0] == TEST)
		{
			UART_command[0] = 0; // make TEST 1 time
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)"TEST", 4);
		}
		else if(UART_command[0] == RAMP1)
		{
			UART_command[0] = 0; // make TEST 1 time
			UART_command[1] = 0;
			k_ramp = ((2*Ampl)/SIZE_BUFFER_ADC) + 1;
			for(uint16_t i = 0; i < SIZE_BUFFER_ADC; i++)
			{
				if(i < 65) Triangle_DAC3[i] = (0 + i*k_ramp);
				else Triangle_DAC3[i] = Triangle_DAC3[128 - i];
			}
			DMA2_Channel3->CMAR =(uint32_t)&Triangle_DAC3[0]; // Adress of buffer
			
		}
		else if(UART_command[0] == RAMP2)
		{
			UART_command[0] = 0; // make TEST 1 time
			UART_command[1] = 0;
			k_ramp = ((1*Ampl)/SIZE_BUFFER_ADC) + 1;
			for(uint16_t i = 0; i < SIZE_BUFFER_ADC; i++)
			{
				Triangle_DAC3[i] = i*k_ramp;	
			}
			DMA2_Channel3->CMAR =(uint32_t)&Triangle_DAC3[0]; // Adress of buffer
			
		}
		else if(UART_command[0] == AMPL)   //strncmp ((char*)UART_command, AMPL_TX, 2)
		{
			HAL_Delay(10);
			Ampl = (UART_command[2]) + (UART_command[3] << 8);
//      if (Ampl > 500) Ampl = 500;
			UART_command[0] = UART_command[1]; // make TEST 1 time
			UART_command[2] = 0;
			UART_command[3] = 0;
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO1;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO1;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief OPAMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP4_Init(void)
{

  /* USER CODE BEGIN OPAMP4_Init 0 */

  /* USER CODE END OPAMP4_Init 0 */

  /* USER CODE BEGIN OPAMP4_Init 1 */

  /* USER CODE END OPAMP4_Init 1 */
  hopamp4.Instance = OPAMP4;
  hopamp4.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp4.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO3;
  hopamp4.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp4.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP4_Init 2 */

  /* USER CODE END OPAMP4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 2);
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
