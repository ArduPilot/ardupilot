/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @brief Generic board initialization routines.
 *
 * By default, we bring up all Maple boards to 72MHz, clocked off the
 * PLL, driven by the 8MHz external crystal. AHB and APB2 are clocked
 * at 72MHz.  APB1 is clocked at 36MHz.
 */

#include "boards.h"
#include "systick.h"
#include "gpio_hal.h"
#include "timer.h"
#include "adc.h"
#include <usb.h>

static void setupFlash(void);
static void setupClocks(void);
static void setupNVIC(void);
static void setupADC(void);
static void setupTimers(void);
static void enableFPU(void);
static void usb_init(void);

void init(void) {
    enableFPU();
    setupFlash();
    setupClocks();
    setupNVIC();
    systick_init(SYSTICK_RELOAD_VAL);
    gpio_init_all();
    afio_init();
    setupADC();
    setupTimers();

    SystemInit();
    SystemCoreClockUpdate();

    boardInit();
    usb_init();
}

void usb_init(void){
    usb_attr_t usb_attr;


    usb_open();

    usb_default_attr(&usb_attr);
	usb_attr.preempt_prio = 3;
	usb_attr.sub_prio = 0;
	usb_attr.use_present_pin = 1;
    usb_attr.present_port = _GPIOC;
    usb_attr.present_pin = 5;
    usb_ioctl(I_USB_SETATTR, &usb_attr);
}

void enableFPU(void){
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));	// set CP10 and CP11 Full Access
#endif
}
/* You could farm this out to the files in boards/ if e.g. it takes
 * too long to test on Maple Native (all those FSMC pins...). */
//bool boardUsesPin(uint8 pin) {
//    for (int i = 0; i < BOARD_NR_USED_PINS; i++) {
//        if (pin == boardUsedPins[i]) {
//            return true;
//        }
//    }
//    return false;
//}

static void setupFlash(void) {

}

/*
 * Clock setup.  Note that some of this only takes effect if we're
 * running bare metal and the bootloader hasn't done it for us
 * already.
 *
 * If you change this function, you MUST change the file-level Doxygen
 * comment above.
 */
static void setupClocks() {

}

static void setupNVIC() {
	/* 4 bit preemption,  0 bit subpriority */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);
}

static void adcDefaultConfig(const adc_dev *dev) {
    adc_init(dev);
    adc_enable(dev);
}

static void setupADC() {
    adc_foreach(adcDefaultConfig);
}

static void timerDefaultConfig(timer_dev*);

static void setupTimers() {
    timer_foreach(timerDefaultConfig);
}

//static void adcDefaultConfig(const adc_dev *dev) {
//}

static void timerDefaultConfig(timer_dev *dev) {

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
    timer_reset(dev);
    timer_pause(dev);

    dev->regs->CR1 = TIMER_CR1_ARPE;
    dev->regs->PSC = 1;
    dev->regs->SR = 0;
    dev->regs->DIER = 0;
    dev->regs->EGR = TIMER_EGR_UG;

    switch (dev->type) {
    case TIMER_ADVANCED:
        dev->regs->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF;
        // fall-through
    case TIMER_GENERAL:
		/* set period to 490 Hz as default */   
		/* SystemCoreClock is set to 168 MHz for STM32F4xx devices */
		if (dev->regs == TIM1 || dev->regs == TIM8 || dev->regs == TIM9 || dev->regs == TIM10 || dev->regs == TIM11)
		{
			/* Timer clock: 168 Mhz */
			/* 
			 * The objective is to generate PWM signal at 490 Hz 
			 * The lowest possible prescaler is 5
			 * Period = (SystemCoreClock / (490 * 6)) - 1 = 57141
			 */			
			TIM_TimeBaseStructure.TIM_Prescaler = 5;
			TIM_TimeBaseStructure.TIM_Period = 57141;				
		}
		else 
		{
			/* Timer clock: 84 Mhz */
			/* 
			 * The objective is to generate PWM signal at 490 Hz 
			 * The lowest possible prescaler is 2
			 * Period = (SystemCoreClock / 2 / (490 * 3)) - 1 = 57141
			 */
		        uint32_t period = (2000000UL / 50) - 1; // 50 Hz
		        uint32_t prescaler =  (uint16_t) ((SystemCoreClock /2) / 2000000) - 1; //2MHz 0.5us ticks

		        TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
			TIM_TimeBaseStructure.TIM_Period = period;
		}

		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(dev->regs, &TIM_TimeBaseStructure);
	
        for (int channel = 1; channel <= 4; channel++) {
        
			switch (channel)
			{
				case 1:
					TIM_SelectOCxM(dev->regs, TIM_Channel_1, TIM_OCMode_PWM1);
					TIM_OC1PreloadConfig(dev->regs, TIM_OCPreload_Enable);		
					break;
				case 2:
					TIM_SelectOCxM(dev->regs, TIM_Channel_2, TIM_OCMode_PWM1);
					TIM_OC2PreloadConfig(dev->regs, TIM_OCPreload_Enable);					
					break;
				case 3:
					TIM_SelectOCxM(dev->regs, TIM_Channel_3, TIM_OCMode_PWM1);
					TIM_OC3PreloadConfig(dev->regs, TIM_OCPreload_Enable);					
					break;
				case 4:
					TIM_SelectOCxM(dev->regs, TIM_Channel_4, TIM_OCMode_PWM1);
					TIM_OC4PreloadConfig(dev->regs, TIM_OCPreload_Enable);					
					break;
			}
                
        }
        // fall-through
    case TIMER_BASIC:
        break;
    }

    timer_resume(dev);
}
