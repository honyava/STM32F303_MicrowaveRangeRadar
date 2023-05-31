#include "main.h"

extern volatile uint32_t BUFF_ADC1_2[SIZE_BUFFER_ADC];
extern const uint16_t Triangle_DAC[128];

	
void ADC1_2_Dual_Init(void)
{
	SET_BIT(RCC->AHBENR, RCC_AHBENR_ADC12EN);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);

	/////////Multi-mode
	SET_BIT(ADC12_COMMON->CCR, ADC12_CCR_CKMODE_0);
	SET_BIT(RCC->CFGR2, RCC_CFGR2_ADCPRE12_DIV1);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_ADC12EN);
	SET_BIT(ADC12_COMMON->CCR, ADC12_CCR_MULTI_1 | ADC12_CCR_MULTI_2); //Regular simultaneous mode only
	SET_BIT(ADC12_COMMON->CCR, (1 << ADC12_CCR_DMACFG_Pos)); // DMA Circular mode
	SET_BIT(ADC12_COMMON->CCR, ADC12_CCR_MDMA_1); // Enable DMA for 12 and 10 bit resolution
	CLEAR_BIT(ADC12_COMMON->CCR, (1 << ADC12_CCR_DELAY_Pos)); // delay 5 cycles

/////////// Setting ADC1
	CLEAR_BIT(ADC1->CFGR, ADC_CFGR_RES_0);
	CLEAR_BIT(ADC1->CFGR, ADC_CFGR_RES_1); // Resolution 12 bit
	SET_BIT(ADC1->SQR1, (3 << ADC_SQR1_SQ1_Pos)); // 3 channel for first conversation (PA2)
	SET_BIT(ADC1->CFGR, ADC_CFGR_ALIGN); // Left alignment
	MODIFY_REG(ADC1->CFGR, ADC_CFGR_EXTSEL, 7 << ADC_CFGR_EXTSEL_Pos); // Externel event Timer 8 TRGO
	MODIFY_REG(ADC1->CFGR, ADC_CFGR_EXTEN, 1 << ADC_CFGR_EXTEN_Pos); //External TRG enable, Rising edge
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP3, 1 << ADC_SMPR1_SMP3_Pos); // Channel 3, 15 cycles for conversation
	SET_BIT(ADC1->CFGR, ADC_CFGR_DMAEN); // Enable DMA

/////////// Setting ADC2
	CLEAR_BIT(ADC2->CFGR, ADC_CFGR_RES_0);
	CLEAR_BIT(ADC2->CFGR, ADC_CFGR_RES_1); // Resolution 12 bit
	SET_BIT(ADC2->SQR1, (3 << ADC_SQR1_SQ1_Pos)); // 3 channel for first conversation (PA6)
	SET_BIT(ADC2->CFGR, ADC_CFGR_ALIGN); // Left alignment
	MODIFY_REG(ADC2->SMPR2, ADC_SMPR1_SMP3, 1 << ADC_SMPR1_SMP3_Pos); // Channel 3, 15 cycles for conversation

////////////// Dual Start
	SET_BIT(ADC1->CR, ADC_CR_ADCAL); //Calibration
	while ((ADC1->CR & ADC_CR_ADCAL) != 0); //wait end of calibration
	SET_BIT(ADC2->CR, ADC_CR_ADCAL); //Calibration
	while ((ADC2->CR & ADC_CR_ADCAL) != 0); //wait end of calibration
	SET_BIT(ADC1->CR, ADC_CR_ADEN);    // Enable ADC
	SET_BIT(ADC1->CR, ADC_CR_ADSTART); // Starts conversion of regular channels
	SET_BIT(ADC2->CR, ADC_CR_ADEN);    // Enable ADC

// Init DMA
	DMA1_Channel1->CPAR = (uint32_t)&ADC12_COMMON->CDR; // Adress of data
	DMA1_Channel1->CMAR = (uint32_t)BUFF_ADC1_2; // Adress of buffer
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE); // Interrupt enable, complete transfer
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_HTIE); // Interrupt enable, half transfer
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR); // perifheral to memmory
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC); // circual mode enable
	CLEAR_BIT(DMA1_Channel1->CCR, (1 << DMA_CCR_PL_Pos)); // Set low priority level
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_PINC); // disabled incrementing perephiral addres
	SET_BIT(DMA1_Channel1->CCR, (2 << DMA_CCR_PSIZE_Pos)); // peripheral data size 32b (word)
	SET_BIT(DMA1_Channel1->CCR, (2 << DMA_CCR_MSIZE_Pos)); // memory data size 32b (word)
	SET_BIT(DMA1_Channel1->CNDTR, (SIZE_BUFFER_ADC << DMA_CNDTR_NDT_Pos));
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN); // Enable DMA
}

void TIM8_Init(void)
{
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN); //clock to TIM8 72MHz
	CLEAR_BIT(TIM8->SMCR, TIM_SMCR_SMS);
	CLEAR_REG(TIM8->CR1);
	MODIFY_REG(TIM8->PSC, TIM_PSC_PSC, 0);
	MODIFY_REG(TIM8->ARR, TIM_ARR_ARR, (125 - 1)); //576 kHz TIM8 then for ADC 576kHz
	CLEAR_BIT(TIM8->CR1, TIM_CR1_DIR_Msk); // straight count
	MODIFY_REG(TIM8->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for ADC1
}

void DAC1_Init(void) // for T2 TSEL = 100     // DMA2 Channel 3
{
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_DAC1EN); // clock for DAC
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN); // clock for DMA2
	SET_BIT(DAC->CR, DAC_CR_EN1); // DAC channel1 enable
	SET_BIT(DAC->CR,DAC_CR_DMAEN1);
	MODIFY_REG(DAC->CR, DAC_CR_MAMP1, 11 << DAC_CR_MAMP1_Pos); // Unmask bits[11:0] of LFSR/ triangle amplitude equal to 4095
	MODIFY_REG(DAC->CR, DAC_CR_TSEL1, 4 << DAC_CR_TSEL1_Pos); // Timer 2 TRGO event
	SET_BIT(DAC->CR,DAC_CR_TEN1); // DAC channel1 trigger enable
	CLEAR_BIT(DAC->SWTRIGR, DAC_SWTRIGR_SWTRIG1); //DAC channel1 software trigger disabled
	
	// Init DMA
	DMA2_Channel3->CPAR = (uint32_t)&DAC->DHR12R1; // Adress of data
  DMA2_Channel3->CMAR =(uint32_t)&Triangle_DAC[0]; // Adress of buffer
	SET_BIT(DMA2_Channel3->CCR, DMA_CCR_TCIE); // Interrupt enable, complete transfer
	SET_BIT(DMA2_Channel3->CCR, DMA_CCR_DIR); // memmory to perifheral
	SET_BIT(DMA2_Channel3->CCR, DMA_CCR_CIRC); // circual mode enable
	DMA2_Channel3->CCR &= ~(1 << DMA_CCR_PL_Pos); // Set low priority level
	SET_BIT(DMA2_Channel3->CCR, DMA_CCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA2_Channel3->CCR, DMA_CCR_PINC); // disabled incrementing perephiral addres
	DMA2_Channel3->CCR |= (1 << DMA_CCR_PSIZE_Pos); //periphiral data size 16b (half-word)
	DMA2_Channel3->CCR |= (1 << DMA_CCR_MSIZE_Pos); //memmory data size 16b (half-word)
	DMA2_Channel3->CNDTR |= (128 << DMA_CNDTR_NDT_Pos);
//	SET_BIT(DMA2_Channel3->CCR, DMA_CCR_EN); // Enable DMA
	NVIC_SetPriority(DMA2_Channel3_IRQn,0);
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	
}

void TIM2_Init(void)
{
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN); //clock to TIM2 72MHz
	CLEAR_BIT(TIM2->SMCR, TIM_SMCR_SMS);
	CLEAR_REG(TIM2->CR1);
	MODIFY_REG(TIM2->PSC, TIM_PSC_PSC, 0);
	MODIFY_REG(TIM2->ARR, TIM_ARR_ARR, (500 - 1)); //144 kHz TIM2 then for DAC 144 kHz  144/128 = 1.125kHz
	SET_BIT(TIM2->DIER, TIM_DIER_UIE); // interrupt on
	CLEAR_BIT(TIM2->CR1, TIM_CR1_DIR_Msk); // straight count
	MODIFY_REG(TIM2->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for DAC1

	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN); //clock to TIM8 72MHz
	CLEAR_BIT(TIM8->SMCR, TIM_SMCR_SMS);
	CLEAR_REG(TIM8->CR1);
	MODIFY_REG(TIM8->PSC, TIM_PSC_PSC, 0);
	MODIFY_REG(TIM8->ARR, TIM_ARR_ARR, (125 - 1)); //576 kHz TIM8 then for ADC 576kHz
	CLEAR_BIT(TIM8->CR1, TIM_CR1_DIR_Msk); // straight count
	MODIFY_REG(TIM8->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for ADC1
	
}

void TIM3_Init(void)
{
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN); // clock to TIM3 72MHz
	CLEAR_BIT(TIM3->SMCR, TIM_SMCR_SMS);
	CLEAR_REG(TIM3->CR1);
	MODIFY_REG(TIM3->PSC, TIM_PSC_PSC, 1-1);
	MODIFY_REG(TIM3->ARR, TIM_ARR_ARR, (7200 - 1)); // 10 kHz TIM3
	SET_BIT(TIM3->DIER, TIM_DIER_UIE); // interrupt on
	CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR_Msk); // straight count
	MODIFY_REG(TIM3->CR2, TIM_CR2_MMS, (2 << TIM_CR2_MMS_Pos)); // Update Event for DAC1
	SET_BIT(TIM3->CR1, TIM_CR1_CEN_Msk); // TIM3 enable
	NVIC_SetPriority(TIM3_IRQn, 4);
	NVIC_EnableIRQ(TIM3_IRQn);
	
}


