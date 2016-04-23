/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file   timer.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  New-style timer interface
 */

#include "timer.h"



/* Just like the corresponding DIER bits:
 * [0] = Update handler;
 * [1,2,3,4] = capture/compare 1,2,3,4 handlers, respectively;
 * [5] = COM;
 * [6] = TRG;
 * [7] = BRK. */
#define NR_ADV_HANDLERS                 8
/* Update, capture/compare 1,2,3,4; <junk>; trigger. */
#define NR_GEN_HANDLERS                 7
/* Update only. */
#define NR_BAS_HANDLERS                 1


static timer_dev timer1 = {
    .regs         = TIM1,
    .clk	      = RCC_APB2Periph_TIM1,
    .type         = TIMER_ADVANCED,
    .handlers     = { [NR_ADV_HANDLERS - 1] = 0 },
};
/** Timer 1 device (advanced) */
timer_dev *TIMER1 = &timer1;

static timer_dev timer2 = {
    .regs         = TIM2,
    .clk    	  = RCC_APB1Periph_TIM2,
    .type         = TIMER_GENERAL,
    .handlers     = { [NR_GEN_HANDLERS - 1] = 0 },
};
/** Timer 2 device (general-purpose) */
timer_dev *TIMER2 = &timer2;

static timer_dev timer3 = {
    .regs         = TIM3,
    .clk	      = RCC_APB1Periph_TIM3,
    .type         = TIMER_GENERAL,
    .handlers     = { [NR_GEN_HANDLERS - 1] = 0 },
};
/** Timer 3 device (general-purpose) */
timer_dev *TIMER3 = &timer3;

static timer_dev timer4 = {
    .regs         = TIM4,
    .clk       	  = RCC_APB1Periph_TIM4,
    .type         = TIMER_GENERAL,
    .handlers     = { [NR_GEN_HANDLERS - 1] = 0 },
};
/** Timer 4 device (general-purpose) */
timer_dev *TIMER4 = &timer4;

static timer_dev timer5 = {
    .regs         = TIM5,
    .clk          = RCC_APB1Periph_TIM5,
    .type         = TIMER_GENERAL,
    .handlers     = { [NR_GEN_HANDLERS - 1] = 0 },
};
/** Timer 5 device (general-purpose) */
timer_dev *TIMER5 = &timer5;

static timer_dev timer6 = {
    .regs         = TIM6,
    .clk          = RCC_APB1Periph_TIM6,
    .type         = TIMER_BASIC,
    .handlers     = { [NR_BAS_HANDLERS - 1] = 0 },
};
/** Timer 6 device (basic) */
timer_dev *TIMER6 = &timer6;

static timer_dev timer7 = {
    .regs         = TIM7,
    .clk          = RCC_APB1Periph_TIM7,
    .type         = TIMER_BASIC,
    .handlers     = { [NR_BAS_HANDLERS - 1] = 0 },
};
/** Timer 7 device (basic) */
timer_dev *TIMER7 = &timer7;

static timer_dev timer8 = {
    .regs         = TIM8,
    .clk          = RCC_APB2Periph_TIM8,
    .type         = TIMER_ADVANCED,
    .handlers     = { [NR_ADV_HANDLERS - 1] = 0 },
};
/** Timer 8 device (advanced) */
timer_dev *TIMER8 = &timer8;

static timer_dev timer12 = {
    .regs         = TIM12,
    .clk          = RCC_APB1Periph_TIM12,
    .type         = TIMER_GENERAL,
    .handlers     = { [NR_GEN_HANDLERS - 1] = 0 },
};
/** Timer 12 device (general-purpose) */
timer_dev *TIMER12 = &timer12;

/*
 * Convenience routines
 */

static void disable_channel(timer_dev *dev, uint8_t channel);
static void pwm_mode(timer_dev *dev, uint8_t channel);
static void output_compare_mode(timer_dev *dev, uint8_t channel);

static inline void enable_irq(timer_dev *dev, uint8_t interrupt);

/**
 * Initialize a timer (enable timer clock)
 * @param dev Timer to initialize
 */
void timer_init(timer_dev *dev) {
	if (dev->regs == TIM1 || dev->regs == TIM8 || dev->regs == TIM9 || dev->regs == TIM10 || dev->regs == TIM11)
		RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
	else
		RCC_APB1PeriphClockCmd(dev->clk, ENABLE);
}

/**
 * Initialize a timer, and reset its register map.
 * @param dev Timer to initialize
 */
void timer_reset(timer_dev *dev) {
	if (dev->regs == TIM1 || dev->regs == TIM8 || dev->regs == TIM9 || dev->regs == TIM10 || dev->regs == TIM11)
		RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
	else
		RCC_APB1PeriphClockCmd(dev->clk, ENABLE);
	TIM_DeInit(dev->regs);		
}

/**
 * @brief Disable a timer.
 *
 * The timer will stop counting, all DMA requests and interrupts will
 * be disabled, and no state changes will be output.
 *
 * @param dev Timer to disable.
 */
void timer_disable(timer_dev *dev) {
    (dev->regs)->CR1 = 0;
    (dev->regs)->DIER = 0;
    switch (dev->type) {
    case TIMER_ADVANCED:        /* fall-through */
    case TIMER_GENERAL:
        (dev->regs)->CCER = 0;
        break;
    case TIMER_BASIC:
        break;
    }
}

/**
 * Sets the mode of an individual timer channel.
 *
 * Note that not all timers can be configured in every mode.  For
 * example, basic timers cannot be configured to output compare mode.
 * Be sure to use a timer which is appropriate for the mode you want.
 *
 * @param dev Timer whose channel mode to set
 * @param channel Relevant channel
 * @param mode New timer mode for channel
 */
void timer_set_mode(timer_dev *dev, uint8_t channel, timer_mode mode) {
    assert_param(channel > 0 && channel <= 4);

    /* TODO decide about the basic timers */
    assert_param(dev->type != TIMER_BASIC);
    if (dev->type == TIMER_BASIC)
        return;

    switch (mode) {
    case TIMER_DISABLED:
        disable_channel(dev, channel);
        break;
    case TIMER_PWM:
        pwm_mode(dev, channel);
        break;
    case TIMER_OUTPUT_COMPARE:
        output_compare_mode(dev, channel);
        break;
    }
}

/**
 * @brief Call a function on timer devices.
 * @param fn Function to call on each timer device.
 */
void timer_foreach(void (*fn)(timer_dev*)) {
    //fn(TIMER1);
    fn(TIMER2);
    fn(TIMER3);
    fn(TIMER4);
    //fn(TIMER5);
    //fn(TIMER6);
    fn(TIMER7);
    //fn(TIMER8);
}

/**
 * @brief Attach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to attach to; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @param handler Handler to attach to the given interrupt.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_attach_interrupt(timer_dev *dev,
                            uint8_t interrupt,
                            voidFuncPtr handler) {
    dev->handlers[interrupt] = handler;
    timer_enable_irq(dev, interrupt);
    enable_irq(dev, interrupt);
}

/**
 * @brief Detach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to detach; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_detach_interrupt(timer_dev *dev, uint8_t interrupt) {
    timer_disable_irq(dev, interrupt);
    dev->handlers[interrupt] = NULL;
}

/*
 * IRQ handlers
 */

static inline void dispatch_adv_brk(timer_dev *dev);
static inline void dispatch_adv_up(timer_dev *dev);
static inline void dispatch_adv_trg_com(timer_dev *dev);
static inline void dispatch_adv_cc(timer_dev *dev);
static inline void dispatch_general(timer_dev *dev);
static inline void dispatch_basic(timer_dev *dev);
/*
void TIM1_BRK_TIM9_IRQHandler(void)
{
    dispatch_adv_brk(TIMER1);
}

void TIM1_UP_TIM10_IRQHandler(void) {
    dispatch_adv_up(TIMER1);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER1);
}

void TIM1_CC_IRQHandler(void) {
    dispatch_adv_cc(TIMER1);
}
*/
void TIM2_IRQHandler(void) {
    dispatch_general(TIMER2);
}

void TIM3_IRQHandler(void) {
    dispatch_general(TIMER3);
}

void TIM4_IRQHandler(void) {
    dispatch_general(TIMER4);
}
/*
void TIM5_IRQHandler(void) {
    dispatch_general(TIMER5);
}

void TIM6_DAC_IRQHandler(void) {
    dispatch_basic(TIMER6);
}
*/
void TIM7_IRQHandler(void) {
    dispatch_basic(TIMER7);
}
/*
void TIM8_BRK_TIM12_IRQHandler(void) {
    dispatch_adv_brk(TIMER8);
}

void TIM8_UP_TIM13_IRQHandler(void) {
    dispatch_adv_up(TIMER8);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER8);
}

void TIM8_CC_IRQHandler(void) {
    dispatch_adv_cc(TIMER8);
}
*/

          
  
/* Note: the following dispatch routines make use of the fact that
 * DIER interrupt enable bits and SR interrupt flags have common bit
 * positions.  Thus, ANDing DIER and SR lets us check if an interrupt
 * is enabled and if it has occurred simultaneously.
 */

/* A special-case dispatch routine for single-interrupt NVIC lines.
 * This function assumes that the interrupt corresponding to `iid' has
 * in fact occurred (i.e., it doesn't check DIER & SR). */
static inline void dispatch_single_irq(timer_dev *dev,
                                       timer_interrupt_id iid,
                                       uint32_t irq_mask) {

    if (dev->regs->DIER & dev->regs->SR & irq_mask) {
    void (*handler)(void) = dev->handlers[iid];
    if (handler) {
        handler();
        dev->regs->SR &= ~irq_mask;
    }
    }
}

/* For dispatch routines which service multiple interrupts. */
#define handle_irq(dier_sr, irq_mask, handlers, iid, handled_irq) do {  \
        if ((dier_sr) & (irq_mask)) {                                   \
            void (*__handler)(void) = (handlers)[iid];                  \
            if (__handler) {                                            \
                __handler();                                            \
                handled_irq |= (irq_mask);                              \
            }                                                           \
        }                                                               \
    } while (0)

static inline void dispatch_adv_brk(timer_dev *dev) {
    dispatch_single_irq(dev, TIMER_BREAK_INTERRUPT, TIMER_SR_BIF);
}

static inline void dispatch_adv_up(timer_dev *dev) {
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
}

static inline void dispatch_adv_trg_com(timer_dev *dev) {
    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    void (**hs)(void) = dev->handlers;
    uint32_t handled = 0; /* Logical OR of SR interrupt flags we end up
                         * handling.  We clear these.  User handlers
                         * must clear overcapture flags, to avoid
                         * wasting time in output mode. */

    handle_irq(dsr, TIMER_SR_TIF,   hs, TIMER_TRG_INTERRUPT, handled);
    handle_irq(dsr, TIMER_SR_COMIF, hs, TIMER_COM_INTERRUPT, handled);

    dev->regs->SR &= ~handled;
}

static inline void dispatch_adv_cc(timer_dev *dev) {
    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    void (**hs)(void) = dev->handlers;
    uint32_t handled = 0;

    handle_irq(dsr, TIMER_SR_CC4IF, hs, TIMER_CC4_INTERRUPT, handled);
    handle_irq(dsr, TIMER_SR_CC3IF, hs, TIMER_CC3_INTERRUPT, handled);
    handle_irq(dsr, TIMER_SR_CC2IF, hs, TIMER_CC2_INTERRUPT, handled);
    handle_irq(dsr, TIMER_SR_CC1IF, hs, TIMER_CC1_INTERRUPT, handled);

    dev->regs->SR &= ~handled;
}

static inline void dispatch_general(timer_dev *dev) {
    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    void (**hs)(void) = dev->handlers;
    uint32_t handled = 0;

    handle_irq(dsr, TIMER_SR_TIF,   hs, TIMER_TRG_INTERRUPT,    handled);
    handle_irq(dsr, TIMER_SR_CC4IF, hs, TIMER_CC4_INTERRUPT,    handled);
    handle_irq(dsr, TIMER_SR_CC3IF, hs, TIMER_CC3_INTERRUPT,    handled);
    handle_irq(dsr, TIMER_SR_CC2IF, hs, TIMER_CC2_INTERRUPT,    handled);
    handle_irq(dsr, TIMER_SR_CC1IF, hs, TIMER_CC1_INTERRUPT,    handled);
    handle_irq(dsr, TIMER_SR_UIF,   hs, TIMER_UPDATE_INTERRUPT, handled);

    dev->regs->SR &= ~handled;
}

static inline void dispatch_basic(timer_dev *dev) {
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
}

/*
 * Utilities
 */

static void disable_channel(timer_dev *dev, uint8_t channel) {
    timer_detach_interrupt(dev, channel);
    timer_cc_disable(dev, channel);
}

static void pwm_mode(timer_dev *dev, uint8_t channel) {
    timer_disable_irq(dev, channel);
   // timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
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
    
    timer_cc_enable(dev, channel);
}

static void output_compare_mode(timer_dev *dev, uint8_t channel) {
	// TODO
//    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
    timer_cc_enable(dev, channel);
}

static void enable_advanced_irq(timer_dev *dev, timer_interrupt_id id);
static void enable_nonmuxed_irq(timer_dev *dev);

static inline void enable_irq(timer_dev *dev, timer_interrupt_id iid) {
    if (dev->type == TIMER_ADVANCED) {
        enable_advanced_irq(dev, iid);
    } else {
        enable_nonmuxed_irq(dev);
    }
}

static void enable_advanced_irq(timer_dev *dev, timer_interrupt_id id) {
    uint8 is_timer1 = dev->clk == RCC_APB2Periph_TIM1;

    switch (id) {
    case TIMER_UPDATE_INTERRUPT:
        NVIC_EnableIRQ(is_timer1 ? TIM1_UP_TIM10_IRQn : TIM8_UP_TIM13_IRQn);
        break;
    case TIMER_CC1_INTERRUPT:
    case TIMER_CC2_INTERRUPT:
    case TIMER_CC3_INTERRUPT:
    case TIMER_CC4_INTERRUPT:
        NVIC_EnableIRQ(is_timer1 ? TIM1_CC_IRQn : TIM8_CC_IRQn);
        break;
    case TIMER_COM_INTERRUPT:
    case TIMER_TRG_INTERRUPT:
        NVIC_EnableIRQ(is_timer1 ? TIM1_TRG_COM_TIM11_IRQn : TIM8_TRG_COM_TIM14_IRQn);
        break;
    case TIMER_BREAK_INTERRUPT:
        NVIC_EnableIRQ(is_timer1 ? TIM1_BRK_TIM9_IRQn : TIM8_BRK_TIM12_IRQn);
        break;
    }
}

static void enable_nonmuxed_irq(timer_dev *dev) {
	if (dev->regs == TIM2)	NVIC_EnableIRQ(TIM2_IRQn);
	else if (dev->regs == TIM3) NVIC_EnableIRQ(TIM3_IRQn);
	else if (dev->regs == TIM4) NVIC_EnableIRQ(TIM4_IRQn);
	else if (dev->regs == TIM5) NVIC_EnableIRQ(TIM5_IRQn);
	else if (dev->regs == TIM6) NVIC_EnableIRQ(TIM6_DAC_IRQn);
	else if (dev->regs == TIM7) NVIC_EnableIRQ(TIM7_IRQn);
	else
	{
        assert_param(0);
    }
}
