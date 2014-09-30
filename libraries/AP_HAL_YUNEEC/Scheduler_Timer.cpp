#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC)

#include "Scheduler.h"
#include <stdint.h>
#include <core_cm4.h>
#include <core_cmFunc.h>
#include <stm32f37x.h>
#include <stm32f37x_rcc.h>
#include <stm32f37x_tim.h>
#include <stm32f37x_misc.h>

#include <stm32f37x_gpio.h>
#include <utility/pinmap_typedef.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

extern "C"
{
	static voidFuncPtr timer6_callback = NULL;
	static voidFuncPtr timer14_failsafe = NULL;
	static volatile uint32_t timer_micros_counter = 0;
	static volatile uint32_t timer_millis_counter = 0;

	void TIM14_IRQHandler(void) {
		timer_micros_counter += 1000;
		timer_millis_counter += 1;

		if (timer14_failsafe != NULL) {
			timer14_failsafe();
		}

		TIM14->SR &= (uint16_t)~TIM_IT_Update;
	}

	void TIM6_DAC1_IRQHandler(void) {
	    if(timer6_callback)
	    	timer6_callback();

		TIM6->SR &= (uint16_t)~TIM_IT_Update;
	}
}

void YUNEECTimer::init() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// TIM6 clock enable
    RCC->APB1ENR |= (RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7 | RCC_APB1Periph_TIM14);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    /* Reset the ARR Preload Bit */
    TIM6->CR1 &= ~((uint16_t)TIM_CR1_ARPE);
    TIM7->CR1 &= ~((uint16_t)TIM_CR1_ARPE);
    TIM14->CR1 &= ~((uint16_t)TIM_CR1_ARPE);

	// Configure two bits for preemption priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the TIM6 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	// TIM Interrupts enable
    TIM6->DIER |= TIM_IT_Update;
    TIM7->DIER &= (uint16_t)~TIM_IT_Update;
    TIM14->DIER |= TIM_IT_Update;

	// TIM6 enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM7->CR1 |= TIM_CR1_CEN;
    TIM14->CR1 |= TIM_CR1_CEN;

}

uint32_t YUNEECTimer::micros() {
    return (TIM14->CNT + timer_micros_counter);
}

uint32_t YUNEECTimer::millis() {
    return timer_millis_counter;
}

/* Delay for the given number of microseconds */
void YUNEECTimer::delay_microseconds(uint16_t us)
{
	TIM7->ARR = us;

	while (!((TIM7->SR) & (uint16_t)0x0001));

	TIM7->ARR = 0;
	TIM7->SR &= (uint16_t)~TIM_IT_Update;
}

void YUNEECTimer::attachInterrupt(voidFuncPtr callback, voidFuncPtr failsafe) {
	timer6_callback = callback;
	timer14_failsafe = failsafe;
}

#endif
