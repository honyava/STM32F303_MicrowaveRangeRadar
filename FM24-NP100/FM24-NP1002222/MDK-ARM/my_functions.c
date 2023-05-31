#include "main.h"
extern UART_HandleTypeDef huart1;

extern volatile uint16_t Triangle_DAC[SIZE_BUFFER_DAC];
extern volatile uint8_t period_number_dac;
extern uint8_t UART_command[SIZE_UART_RX];
extern volatile uint16_t count_dma_period;	
extern volatile uint8_t count_dac_period;	
extern volatile uint8_t flag_data_adc_collect;
extern struct flags flags;

void Rx_Complete(uint8_t flag_rx_temp)
{
	if (flag_rx_temp == 1)
	{
		HAL_Delay(1);
		flags.rx = 0;
		flags.adc_start = 1;
		period_number_dac = UART_command[1];
		SET_BIT(DMA2_Channel3->CCR, DMA_CCR_EN);	
		SET_BIT(TIM2->CR1, TIM_CR1_CEN_Msk); // TIM2 enable
	}
	return;
}
//void Collect_ADC_Complete(uint8_t flag_data_adc_collect_temp, struct message_ADC message_ADC12_temp)
//{
//	if (flag_data_adc_collect_temp == 1) 
//	{
//		
//		uint8_t period_number = period_number_dac;
//		uint8_t start_byte = 0x01;
//		uint16_t message_size = message_size = SIZE_BUFFER_ADC*count_dma_period*(sizeof(uint32_t) / sizeof(uint8_t)); //bytes;	
//		HAL_UART_AbortReceive_IT(&huart1);
//		flag_adc_start = 0;
//		UART_command[0] = 0;
//		UART_command[1] = 0;
//		message_ADC12_temp.preamble = (start_byte) | (period_number << 8) | (message_size << 16);
//		flag_data_adc_collect = 0;
//		count_dma_period = 0;
//		count_dac_period = 0;			
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)&message_ADC12_temp,  message_size + sizeof(message_ADC12_temp.preamble));
//		}
//	return;
//}


void Make_Ramp(uint8_t ramp, uint16_t ampl)
{
	if(ramp == RAMP1_COMMAND)
	{
		int k_ramp = ((2*ampl)/SIZE_BUFFER_DAC) + 1;
		for(uint16_t i = 0; i < SIZE_BUFFER_DAC; i++)
		{
			if(i < SIZE_BUFFER_DAC/2 + 1) Triangle_DAC[i] = (0 + i*k_ramp);
			else Triangle_DAC[i] = Triangle_DAC[SIZE_BUFFER_DAC - i];
		}
	}
	else // for RAMP2
	{
		int k_ramp = ((1*ampl)/SIZE_BUFFER_DAC) + 1;
		for(uint16_t i = 0; i < SIZE_BUFFER_DAC; i++)
		{
			Triangle_DAC[i] = i*k_ramp;	
		}
	}
	MODIFY_REG(DMA2_Channel3->CMAR, DMA_CMAR_MA, (uint32_t)(Triangle_DAC)); // Adress of buffer
}
