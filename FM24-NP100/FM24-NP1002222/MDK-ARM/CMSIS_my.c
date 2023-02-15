#include "main.h"

void ADC3_4_Dual_Init(void)
{
	SET_BIT(RCC->AHBENR,RCC_AHBENR_ADC34EN);
	//SET_BIT(ADC3->IER, ADC_IER_EOCIE); // Enable interrupt for EOC
	/////////Multi-mode
	ADC3->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; //Regular simultaneous mode only
	ADC->CCR |= ADC_CCR_DMA_1; //  DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2)
	ADC->CCR |= (1 << ADC_CCR_DDS_Pos); // Enable DMA
	ADC->CCR &= ~(1 << ADC_CCR_DELAY_Pos); // delay 5 cycles	
	
	
	
	CLEAR_BIT(ADC3->CFGR, ADC_CFGR_RES_0);
	CLEAR_BIT(ADC3->CFGR, ADC_CFGR_RES_1); // Resolution 12 bit
	//SET_BIT(ADC3->CR, ADC_CR_ADON); // A/D Converter ON 
/*mb*/	ADC3->SQR1 = 1; // 1 channel for first conversation
	CLEAR_BIT(ADC3->CFGR, ADC_CFGR_ALIGN); // Right alignment
	MODIFY_REG(ADC3->CFGR, ADC_CFGR_EXTSEL, 8 << ADC_CFGR_EXTSEL_Pos); // Externel event Timer 8 TRGO
	MODIFY_REG(ADC3->CFGR, ADC_CFGR_EXTEN, 1 << ADC_CFGR_EXTEN_Pos); //External TRG enable, Rising edge	
	SET_BIT(ADC3->CFGR, ADC_CFGR_DMAEN); // Enable DMA
}