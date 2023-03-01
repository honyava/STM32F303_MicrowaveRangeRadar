#include "main.h"

extern uint32_t volatile BUFF_ADC1_2[SIZE_BUFFER_ADC];

void ADC1_2_Dual_Init(void)
{
	SET_BIT(RCC->AHBENR,RCC_AHBENR_ADC12EN);
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/////////Multi-mode
	ADC12_COMMON->CCR |= ADC12_CCR_CKMODE_0;
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC12_COMMON->CCR |= ADC12_CCR_MULTI_1 | ADC12_CCR_MULTI_2; //Regular simultaneous mode only
	ADC12_COMMON->CCR |= (1 << ADC12_CCR_DMACFG_Pos); //  DMA Circular mode
	ADC12_COMMON->CCR |= ADC12_CCR_MDMA_1; // Enable DMA for 12 and 10 bit resolution
	ADC12_COMMON->CCR &= ~(1 << ADC12_CCR_DELAY_Pos); // delay 5 cycles	
	/////////// Setting ADC1
	CLEAR_BIT(ADC1->CFGR, ADC_CFGR_RES_0);
	CLEAR_BIT(ADC1->CFGR, ADC_CFGR_RES_1); // Resolution 12 bit
  ADC1->SQR1 |= (3 << ADC_SQR1_SQ1_Pos); // 3 channel for first conversation (PA2)
//	CLEAR_BIT(ADC1->CFGR, ADC_CFGR_ALIGN); // Right alignment
	SET_BIT(ADC1->CFGR, ADC_CFGR_ALIGN); // Left alignment
	//SET_BIT(ADC1->CFGR, ADC_CFGR_CONT);
	MODIFY_REG(ADC1->CFGR, ADC_CFGR_EXTSEL, 7 << ADC_CFGR_EXTSEL_Pos); // Externel event Timer 8 TRGO
	MODIFY_REG(ADC1->CFGR, ADC_CFGR_EXTEN, 1 << ADC_CFGR_EXTEN_Pos); //External TRG enable, Rising edge	
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP3, 1 << ADC_SMPR1_SMP3_Pos); // Channel 3, 15 cycles for conversation	
	SET_BIT(ADC1->CFGR, ADC_CFGR_DMAEN); // Enable DMA
	
	/////////// Setting ADC2
	CLEAR_BIT(ADC2->CFGR, ADC_CFGR_RES_0);
	CLEAR_BIT(ADC2->CFGR, ADC_CFGR_RES_1); // Resolution 12 bit
	ADC2->SQR1 |= (3 << ADC_SQR1_SQ1_Pos); // 3 channel for first conversation (PA6)
//	ADC2->CFGR |=  ADC_CFGR_EXTEN;
//	ADC2->CFGR |=  ADC_CFGR_EXTSEL;
//	CLEAR_BIT(ADC2->CFGR, ADC_CFGR_ALIGN); // Right alignment
	SET_BIT(ADC2->CFGR, ADC_CFGR_ALIGN); // Left alignment
	MODIFY_REG(ADC2->SMPR2, ADC_SMPR1_SMP3, 1 << ADC_SMPR1_SMP3_Pos); // Channel 3, 15 cycles for conversation
  ////////////// Dual Start
	ADC1->CR |= ADC_CR_ADCAL; //Calibration
	while ((ADC1->CR & ADC_CR_ADCAL) != 0); //wait end of calibration
	ADC2->CR |= ADC_CR_ADCAL; //Calibration
	while ((ADC2->CR & ADC_CR_ADCAL) != 0); //wait end of calibration		
	SET_BIT(ADC1->CR, ADC_CR_ADEN);    // Enable ADC
	SET_BIT(ADC1->CR, ADC_CR_ADSTART); // Starts conversion of regular channels
	SET_BIT(ADC2->CR, ADC_CR_ADEN);    // Enable ADC
	// Init DMA
	DMA1_Channel1->CPAR = (uint32_t)&ADC12_COMMON->CDR; // Adress of data
  DMA1_Channel1->CMAR =(uint32_t)&BUFF_ADC1_2[0]; // Adress of buffer
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE); // Interrupt enable, complete transfer
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_HTIE); // Interrupt enable, half transfer
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR); // perifheral to memmory
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC); // circual mode enable
	DMA1_Channel1->CCR &= ~(1 << DMA_CCR_PL_Pos); // Set low priority level
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_PINC); // disabled incrementing perephiral addres
	DMA1_Channel1->CCR |= (2 << DMA_CCR_PSIZE_Pos); //periphiral data size 16b (half-word)
	DMA1_Channel1->CCR |= (2 << DMA_CCR_MSIZE_Pos); //memmory data size 16b (half-word)
//	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_CHSEL); // enable channel 0
	DMA1_Channel1->CNDTR |= (SIZE_BUFFER_ADC << DMA_CNDTR_NDT_Pos);
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN); // Enable DMA
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_SetPriority(DMA1_Channel1_IRQn,1);
	/////
}

void TIM8_Init(void)
{
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN); //clock to TIM8 64MHz
	TIM8->SMCR &= ~ TIM_SMCR_SMS; 
	CLEAR_REG(TIM8->CR1);
	TIM8->PSC = 1;
	TIM8->ARR = 64; //1 MHz TIM8 then for ADC 0.5MHz
	TIM8->DIER |= TIM_DIER_UIE; //interrupt on
	TIM8->CR1 &= ~TIM_CR1_DIR_Msk; // straight count
	
//	MODIFY_REG(TIM8->CR2, TIM_CR2_MMS2, 2 << TIM_CR2_MMS2_Pos);
	MODIFY_REG(TIM8->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for ADC1
//	SET_BIT(TIM8->CR1, TIM_CR1_CEN_Msk); // TIM8 enable
//	NVIC_SetPriority(1, TIM8_UP_IRQn);
	NVIC_EnableIRQ(TIM8_UP_IRQn);
}



