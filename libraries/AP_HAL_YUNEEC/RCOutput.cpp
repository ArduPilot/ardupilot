#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"
#include <stm32f37x.h>
#include <stm32f37x_gpio.h>
#include <stm32f37x_tim.h>
#include <stm32f37x_rcc.h>

using namespace YUNEEC;

void YUNEECRCOutput::init(void* machtnichts) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

/*
 * If use ESC rail, we need power from ESC and max number of channels is 8
 */
#ifdef USE_ESC_RAIL
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	/* TIMs clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_TIM19, ENABLE);

	/* GPIOs clock enable */
    RCC->AHBENR |= (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC);

	/* GPIOs Configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* GPIOA Configuration: TIM2 CH1 (PA15)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB Configuration: TIM2 CH2 (PB3), TIM16 CH1 (PB4) and TIM17 CH1 (PB5) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIOC Configuration: TIM5 CH1 (PC0), TIM19 CH1 (PC10), TIM19 CH2 (PC11) and TIM19 CH3 (PC12) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM Channels to GPIOs */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_10);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource0, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period 			= 20000 - 1; 	  // Default 50 Hz
	TIM_TimeBaseStructure.TIM_Prescaler 		= SystemCoreClock / 1000000 - 1; // 1 MHZ
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM19, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse 			= 0x0000000;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low;

	/* PWM Mode configuration: TIM2 Channel1/2 */
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM Mode configuration: TIM5 Channel1 */
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	/* PWM Mode configuration: TIM16 Channel1 */
	TIM_OC1Init(TIM16, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

	/* PWM Mode configuration: TIM17 Channel1 */
	TIM_OC1Init(TIM17, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);

	/* PWM Mode configuration: TIM19 Channel1/2/3 */
	TIM_OC1Init(TIM19, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM19, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM19, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM19, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM19, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM19, TIM_OCPreload_Enable);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStructure.TIM_OSSRState 		= TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel 		= TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime 			= 0x00;
	TIM_BDTRInitStructure.TIM_Break 			= TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_AutomaticOutput 	= TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(TIM16, &TIM_BDTRInitStructure);
	TIM_BDTRConfig(TIM17, &TIM_BDTRInitStructure);

	/* Main Output Enable */
	TIM16->BDTR |= TIM_BDTR_MOE;
	TIM17->BDTR |= TIM_BDTR_MOE;

	/* TIM enable counter */
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM16->CR1 |= TIM_CR1_CEN;
    TIM17->CR1 |= TIM_CR1_CEN;
    TIM19->CR1 |= TIM_CR1_CEN;
/*
 * Default use SERVO rail, we have 5v servo rail and max number of channels is 6
 */
#else
	/* TIMs clock enable */
    RCC->APB1ENR |= (RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM12);

	/* GPIOs clock enable */
    RCC->AHBENR |= (RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC);

	/* GPIOs Configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* GPIOB Configuration: TIM12 CH1 (PB14), CH2 (PB15)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIOC Configuration: TIM3 CH1 (PC6), CH2 (PC7), CH3 (PC8), CH4 (PC9)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM Channels to GPIOs */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_9);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_9);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period 			= 20000 - 1; 	  // Default 50 Hz
	TIM_TimeBaseStructure.TIM_Prescaler 		= SystemCoreClock / 1000000 - 1; // 1 MHZ
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse 			= 0x0000000;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low;

	/* PWM Mode configuration: TIM3 Channel1/2/3/4 */
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM Mode configuration: TIM15 Channel1/2 */
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

	/* TIM enable counter */
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM12->CR1 |= TIM_CR1_CEN;
#endif
}

void YUNEECRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
	uint32_t period = 1000000 / freq_hz - 1;
#ifdef USE_ESC_RAIL
    if ((chmask & ( _BV(CH_1) | _BV(CH_5))) != 0) {
    	TIM2->ARR = period;
    }

    if ((chmask & ( _BV(CH_2) | _BV(CH_3) | _BV(CH_4))) != 0) {
		TIM19->ARR = period;
    }

    if ((chmask & ( _BV(CH_6))) != 0) {
		TIM16->ARR = period;
    }

    if ((chmask & ( _BV(CH_7))) != 0) {
		TIM17->ARR = period;
    }

    if ((chmask & ( _BV(CH_8))) != 0) {
		TIM5->ARR = period;
    }
#else
    if ((chmask & ( _BV(CH_1) | _BV(CH_2))) != 0) {
		TIM12->ARR = period;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_5) | _BV(CH_6))) != 0) {
		TIM3->ARR = period;
    }
#endif
}

uint16_t YUNEECRCOutput::get_freq(uint8_t ch) {
    uint32_t ARR_Value = 0;

#ifdef USE_ESC_RAIL
    if(ch == CH_1 || ch == CH_5)
    	ARR_Value = TIM2->ARR;
    else if(ch == CH_2 || ch == CH_3 || ch == CH_4)
    	ARR_Value = TIM19->ARR;
    else if(ch == CH_6)
    	ARR_Value = TIM16->ARR;
    else if(ch == CH_7)
    	ARR_Value = TIM17->ARR;
    else if(ch == CH_8)
    	ARR_Value = TIM5->ARR;
    else
    	return 0;
#else
    if(ch == CH_1 || ch == CH_2)
    	ARR_Value = TIM12->ARR;
    else if(ch == CH_3 || ch == CH_4 || ch == CH_5 || ch == CH_6)
    	ARR_Value = TIM3->ARR;
    else
    	return 0;
#endif
    return (1000000 / (ARR_Value + 1));
}

void YUNEECRCOutput::enable_ch(uint8_t ch) {
#ifdef USE_ESC_RAIL
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_2: TIM_CCxCmd(TIM19, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_3: TIM_CCxCmd(TIM19, TIM_Channel_2, TIM_CCx_Enable); break;
		case CH_4: TIM_CCxCmd(TIM19, TIM_Channel_3, TIM_CCx_Enable); break;
        case CH_5: TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_6: TIM_CCxCmd(TIM16, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_7: TIM_CCxCmd(TIM17, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_8: TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable); break;
        default:
            break;
    }
#else
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM12, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_2: TIM_CCxCmd(TIM12, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_3: TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable); break;
		case CH_4: TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_5: TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Enable); break;
        case CH_6: TIM_CCxCmd(TIM3, TIM_Channel_4, TIM_CCx_Enable); break;
        default:
            break;
    }
#endif
}




void YUNEECRCOutput::disable_ch(uint8_t ch) {
#ifdef USE_ESC_RAIL
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_2: TIM_CCxCmd(TIM19, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_3: TIM_CCxCmd(TIM19, TIM_Channel_2, TIM_CCx_Disable); break;
		case CH_4: TIM_CCxCmd(TIM19, TIM_Channel_3, TIM_CCx_Disable); break;
        case CH_5: TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_6: TIM_CCxCmd(TIM16, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_7: TIM_CCxCmd(TIM17, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_8: TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable); break;
        default:
            break;
    }
#else
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM12, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_2: TIM_CCxCmd(TIM12, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_3: TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable); break;
		case CH_4: TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_5: TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Disable); break;
        case CH_6: TIM_CCxCmd(TIM3, TIM_Channel_4, TIM_CCx_Disable); break;
        default:
            break;
    }
#endif
}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void YUNEECRCOutput::write(uint8_t ch, uint16_t period_us) {
    uint16_t pwm = constrain_period(period_us);

#ifdef USE_ESC_RAIL
    switch(ch){
		case CH_1: TIM2->CCR1 = (uint32_t)pwm; break;
		case CH_2: TIM19->CCR1 = pwm; break;
		case CH_3: TIM19->CCR2 = pwm; break;
		case CH_4: TIM19->CCR3 = pwm; break;
		case CH_5: TIM2->CCR2 = (uint32_t)pwm; break;
		case CH_6: TIM16->CCR1 = pwm; break;
		case CH_7: TIM17->CCR1 = pwm; break;
		case CH_8: TIM5->CCR1 = (uint32_t)pwm; break;
		default:
			break;
    }
#else
    switch(ch){
		case CH_1: TIM12->CCR1 = pwm; break;
		case CH_2: TIM12->CCR2 = pwm; break;
		case CH_3: TIM3->CCR1 = pwm; break;
		case CH_4: TIM3->CCR2 = pwm; break;
		case CH_5: TIM3->CCR3 = pwm; break;
		case CH_6: TIM3->CCR4 = pwm; break;
		default:
			break;
    }
#endif
}

void YUNEECRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t YUNEECRCOutput::read(uint8_t ch) {
	uint16_t pwm = 0;

#ifdef USE_ESC_RAIL
    switch (ch) {
		case CH_1: pwm = (uint16_t)TIM2->CCR1; break;
		case CH_2: pwm = TIM19->CCR1; break;
		case CH_3: pwm = TIM19->CCR2; break;
		case CH_4: pwm = TIM19->CCR3; break;
		case CH_5: pwm = (uint16_t)TIM2->CCR2; break;
		case CH_6: pwm = TIM16->CCR1; break;
		case CH_7: pwm = TIM17->CCR1; break;
		case CH_8: pwm = (uint16_t)TIM5->CCR1; break;
		default:
			break;
    }
#else
    switch (ch) {
		case CH_1: pwm = TIM12->CCR1; break;
		case CH_2: pwm = TIM12->CCR2; break;
		case CH_3: pwm = TIM3->CCR1; break;
		case CH_4: pwm = TIM3->CCR2; break;
		case CH_5: pwm = TIM3->CCR3; break;
		case CH_6: pwm = TIM3->CCR4; break;
		default:
			break;
    }
#endif

    return pwm;
}

void YUNEECRCOutput::read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

#endif
