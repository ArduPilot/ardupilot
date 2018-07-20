/******************************************************************************
 * The MIT License

(c) 2017 night_ghost@ykoctpa.ru
 
based on:

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

#pragma GCC optimize ("O2")

/**
 * @file   timer.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  New-style timer interface
 */

#include "timer.h"
#include "dma.h"
#include <string.h>
#include "nvic.h"

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


static Handler tim1_handlers[NR_ADV_HANDLERS] IN_CCM;
static Handler tim2_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim3_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim4_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim5_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim6_handlers[NR_BAS_HANDLERS] IN_CCM;
static Handler tim7_handlers[NR_BAS_HANDLERS] IN_CCM;
static Handler tim8_handlers[NR_ADV_HANDLERS] IN_CCM;
static Handler tim9_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim10_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim11_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim12_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim13_handlers[NR_GEN_HANDLERS] IN_CCM;
static Handler tim14_handlers[NR_GEN_HANDLERS] IN_CCM;

static timerState timer_state[14] IN_CCM;

const timer_dev timers[] = {
 { /** Timer 1 device (advanced) */

    .regs         = TIM1,
    .clk	  = RCC_APB2Periph_TIM1,
    .handlers     = tim1_handlers,
    .af           = GPIO_AF_TIM1,
    .type         = TIMER_ADVANCED,
    .n_handlers   = NR_ADV_HANDLERS,
    .bus          = 1,
    .id           = 1,
    .state        = &timer_state[0],
    .ch_dma = { // dma per channel: stream, channel
        { DMA2_STREAM1, 6 },
        { DMA2_STREAM2, 6 },
        { DMA2_STREAM6, 6 },
        { DMA2_STREAM4, 6 },
    },        
 },

 { /** Timer 2 device (general-purpose) */
    .regs         = TIM2,
    .clk    	  = RCC_APB1Periph_TIM2,
    .handlers     = tim2_handlers,
    .af           = GPIO_AF_TIM2,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 2,
    .state        = &timer_state[1],
    .ch_dma = { // dma per channel: stream, channel 
        { DMA1_STREAM5, 3 },
        { DMA1_STREAM6, 3 },
        { DMA1_STREAM1, 3 },
        { DMA1_STREAM7, 6 },
    },

 },

 { /** Timer 3 device (general-purpose) */
    .regs         = TIM3,
    .clk	  = RCC_APB1Periph_TIM3,
    .handlers     = tim3_handlers,
    .af           = GPIO_AF_TIM3,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 3,
    .state        = &timer_state[2],
    .ch_dma = { // dma per channel: stream, channel 
        { DMA1_STREAM4, 5 },
        { DMA1_STREAM5, 5 },
        { DMA1_STREAM7, 5 },
        { DMA1_STREAM2, 5 },
    },

 },

 { /** Timer 4 device (general-purpose) */
    .regs         = TIM4,
    .clk       	  = RCC_APB1Periph_TIM4,
    .handlers     = tim4_handlers,
    .af           = GPIO_AF_TIM4,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 4,
    .state        = &timer_state[3],
    .ch_dma = { // dma per channel: stream, channel 
        { DMA1_STREAM0, 2 },
        { DMA1_STREAM3, 2 },
        { DMA1_STREAM7, 2 },
        { -1, -1 },
    },

 },

 { /** Timer 5 device (general-purpose) */
    .regs         = TIM5,
    .clk          = RCC_APB1Periph_TIM5,
    .handlers     = tim5_handlers,
    .af           = GPIO_AF_TIM5,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 5,
    .state        = &timer_state[4],
    .ch_dma = { // dma per channel: stream, channel 
        { DMA1_STREAM2, 6 },
        { DMA1_STREAM4, 6 },
        { DMA1_STREAM0, 6 },
        { DMA1_STREAM1, 6 },
    },
 },

 { /** Timer 6 device (basic) */
    .regs         = TIM6,
    .clk          = RCC_APB1Periph_TIM6,
    .handlers     = tim6_handlers,
    .af           = 0,
    .type         = TIMER_BASIC,
    .n_handlers   = NR_BAS_HANDLERS,
    .bus          = 0,
    .id           = 6,
    .state        = &timer_state[5],
 },

 { /** Timer 7 device (basic) */
    .regs         = TIM7,
    .clk          = RCC_APB1Periph_TIM7,
    .handlers     = tim7_handlers,
    .af           = 0,
    .type         = TIMER_BASIC,
    .n_handlers   = NR_BAS_HANDLERS,
    .bus          = 0,
    .id           = 7,
    .state        = &timer_state[6],
 },

 {  /** Timer 8 device (advanced) */
    .regs         = TIM8,
    .clk          = RCC_APB2Periph_TIM8,
    .handlers     = tim8_handlers,
    .af           = GPIO_AF_TIM8,
    .type         = TIMER_ADVANCED,
    .n_handlers   = NR_ADV_HANDLERS,
    .bus          = 1,
    .id           = 8,
    .state        = &timer_state[7],
    .ch_dma = { // dma per channel: stream, channel 
        { DMA2_STREAM2, 7 },
        { DMA2_STREAM3, 7 },
        { DMA2_STREAM4, 7 },
        { DMA2_STREAM7, 7 },
    },
 },

// another timers don't has DMA

 {  /** Timer 9 device (general-purpose) */
    .regs         = TIM9,
    .clk          = RCC_APB2Periph_TIM9,
    .handlers     = tim9_handlers,
    .af           = GPIO_AF_TIM9,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 1,
    .id           = 9,
    .state        = &timer_state[8],
 },

 { /** Timer 10 device (general-purpose) */
    .regs         = TIM10,
    .clk          = RCC_APB2Periph_TIM10,
    .handlers     = tim10_handlers,
    .af           = GPIO_AF_TIM10,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 1,
    .id           = 10,
    .state        = &timer_state[9],
 },

 { /** Timer 11 device (general-purpose) */
    .regs         = TIM11,
    .clk          = RCC_APB2Periph_TIM11,
    .handlers     = tim11_handlers,
    .af           = GPIO_AF_TIM11,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 1,
    .id           = 11,
    .state        = &timer_state[10],
 },

 { /** Timer 12 device (general-purpose) */
    .regs         = TIM12,
    .clk          = RCC_APB1Periph_TIM12,
    .handlers     = tim12_handlers,
    .af           = GPIO_AF_TIM12,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 12,
    .state        = &timer_state[11],
 },

 { /** Timer 13 device (general-purpose) */
    .regs         = TIM13,
    .clk          = RCC_APB1Periph_TIM13,
    .handlers     = tim13_handlers,
    .af           = GPIO_AF_TIM13,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 13,
    .state        = &timer_state[12],
 },

 { /** Timer 14 device (general-purpose) */
    .regs         = TIM14,
    .clk          = RCC_APB1Periph_TIM14,
    .handlers     = tim14_handlers,
    .af           = GPIO_AF_TIM14,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 14,
    .state        = &timer_state[13],
 }
};


/*
 * Convenience routines
 */

static void disable_channel(const timer_dev *dev, uint8_t channel);
static void pwm_mode(const timer_dev *dev, uint8_t channel);
static void output_compare_mode(const timer_dev *dev, uint8_t channel);

static inline void enable_irq(const timer_dev *dev, uint8_t interrupt, uint8_t priority);


/**
 * Initialize a timer (enable timer clock)
 * @param dev Timer to initialize
 */
void timer_init(const timer_dev *dev) {

    if(dev->bus)
    	RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
    else
	RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

}

/**
 * Initialize a timer, and reset its register map.
 * @param dev Timer to initialize
 */
void timer_reset(const timer_dev *dev) {
    memset(dev->handlers, 0, dev->n_handlers * sizeof(Handler));
    memset(dev->state, 0, sizeof(*dev->state));

    if(dev->bus) {
    	RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
    	RCC_APB2PeriphResetCmd(dev->clk, ENABLE);
        RCC_APB2PeriphResetCmd(dev->clk, DISABLE);
    } else {
	RCC_APB1PeriphClockCmd(dev->clk, ENABLE);
    	RCC_APB1PeriphResetCmd(dev->clk, ENABLE);
        RCC_APB1PeriphResetCmd(dev->clk, DISABLE);
    }

}

/**
 * @brief Disable a timer.
 *
 * The timer will stop counting, all DMA requests and interrupts will
 * be disabled, and no state changes will be output.
 *
 * @param dev Timer to disable.
 */
void timer_disable(const timer_dev *dev) {
    dev->regs->CR1 = 0;
    dev->regs->DIER = 0;

    switch (dev->type) {
    case TIMER_ADVANCED:
    case TIMER_GENERAL:
        (dev->regs)->CCER = 0;
        break;
    case TIMER_BASIC:
        break;
    }
}


// initial configuration - set required frequency (in kHz) and period (in ticks) 
// returns real timers freq
uint32_t configTimeBase(const timer_dev *dev, uint16_t period, uint16_t khz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TypeDef *tim = dev->regs;

    timer_init(dev); // turn it on
    
    timer_pause(dev);

    dev->regs->CR1 = TIMER_CR1_ARPE;
    dev->regs->PSC = 1;
    dev->regs->SR = 0;
    dev->regs->DIER = 0;
    dev->regs->EGR = TIMER_EGR_UG;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.TIM_Period = (period - 1) & get_timer_mask(dev); // AKA TIMx_ARR
    uint32_t freq = (uint32_t)khz * 1000;
    uint16_t prescaler;
    uint32_t tf; // timer's frequency

    if (tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11) {            // 168MHz
        tf  = SystemCoreClock;
    } else {                                                                                    // 84 MHz
        tf  = SystemCoreClock / 2;
    }

    prescaler = ((tf + freq/4) / freq) - 1; // ==41 for 2MHz
    
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    freq = tf / prescaler; // real timer's frequency
    
    dev->state->freq = freq; // store

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    switch (dev->type) {
    case TIMER_ADVANCED:
        dev->regs->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF; //  break and dead-time register, enable output
        // fall-through
    case TIMER_GENERAL:
    case TIMER_BASIC:
        break;
    }

    timer_set_count(dev,0);    

    dev->state->freq_scale = (float)freq / (khz * 1000); // remember ratio for correction

    return freq;
}


static void disable_channel(const timer_dev *dev, timer_Channel channel) {
    timer_detach_interrupt(dev, channel);
    timer_cc_disable(dev, channel);
}

static void pwm_mode(const timer_dev *dev, timer_Channel channel) {
    timer_disable_irq(dev, channel);
    timer_oc_set_mode(dev, channel, BOARD_PWM_MODE, TIMER_OC_PE);
    timer_cc_enable(dev, channel);
}

static void output_compare_mode(const timer_dev *dev, timer_Channel channel) {
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
    timer_cc_enable(dev, channel);
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
void timer_set_mode(const timer_dev *dev, timer_Channel channel, timer_mode mode) {
    assert_param(channel > 0 && channel <= 4);

    /* TODO decide about the basic timers */
    if (!dev || dev->type == TIMER_BASIC)
        return;
        
    assert_param(dev->type != TIMER_BASIC);

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
void timer_foreach(void (*fn)(const timer_dev*)) {
    uint8_t i;
    for(i=0; i<(sizeof(timers)/sizeof(timer_dev)); i++){
        fn(&timers[i]);
    }
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
void timer_attach_interrupt(const timer_dev *dev, timer_interrupt_id interrupt, Handler handler, uint8_t priority) {
    if(interrupt>=dev->n_handlers) return;

    dev->handlers[interrupt] = handler;
    timer_enable_irq(dev, interrupt);
    enable_irq(dev, interrupt, priority);
}

// attach all timer's interrupts to one handler - for PWM/PPM input
void timer_attach_all_interrupts(const timer_dev *dev,  Handler handler) {
    uint16_t i;
    for(i=0; i < dev->n_handlers; i++) {
        dev->handlers[i] = handler;
    }
    // enabling by caller
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
void timer_detach_interrupt(const timer_dev *dev, timer_interrupt_id interrupt) {
    timer_disable_irq(dev, interrupt);
    if(interrupt>=dev->n_handlers) return;
    dev->handlers[interrupt] = 0;
}

/*
 * IRQ handlers
 */

static inline void dispatch_adv_brk(const timer_dev *dev);
static inline void dispatch_adv_up(const timer_dev *dev);
static inline void dispatch_adv_trg_com(const timer_dev *dev);
static inline void dispatch_adv_cc(const timer_dev *dev);
static inline void dispatch_general(const timer_dev *dev);
static inline void dispatch_general_h(const timer_dev *dev);
static inline void dispatch_basic(const timer_dev *dev);

void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);





void TIM1_BRK_TIM9_IRQHandler(void)
{
    dispatch_adv_brk(TIMER1);
    dispatch_general_h(TIMER9);

}

void TIM1_UP_TIM10_IRQHandler(void) {
    dispatch_adv_up(TIMER1);
    dispatch_general_h(TIMER10);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER1);
    dispatch_general_h(TIMER11);

}

void TIM1_CC_IRQHandler(void) {
    dispatch_adv_cc(TIMER1);
}

void TIM2_IRQHandler(void) {
    dispatch_general(TIMER2);
}

void TIM3_IRQHandler(void) {
    dispatch_general(TIMER3);
}

void TIM4_IRQHandler(void) {
    dispatch_general(TIMER4);
}

void TIM5_IRQHandler(void) {
    dispatch_general(TIMER5);
}

void TIM6_DAC_IRQHandler(void) {
    dispatch_basic(TIMER6);
}

void TIM7_IRQHandler(void) {
    dispatch_basic(TIMER7);
}

// used in PWM
void TIM8_BRK_TIM12_IRQHandler(void) { // used in PWM tim12
    dispatch_adv_brk(TIMER8);
    dispatch_general_h(TIMER12);
}

void TIM8_CC_IRQHandler(void) { // used in PWM tim8
    dispatch_adv_cc(TIMER8);
}


void TIM8_UP_TIM13_IRQHandler(void) { // not conflicts with PWM
    dispatch_adv_up(TIMER8);
    dispatch_general_h(TIMER13);

}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER8);
    dispatch_general_h(TIMER14);

}

  
/* Note: the following dispatch routines make use of the fact that
 * DIER interrupt enable bits and SR interrupt flags have common bit
 * positions.  Thus, ANDing DIER and SR lets us check if an interrupt
 * is enabled and if it has occurred simultaneously.
 */

static INLINE  void dispatch_single_irq(const timer_dev *dev,
                                       timer_interrupt_id iid,
                                       uint32_t irq_mask) {
    
    uint32_t dsr = dev->regs->DIER & dev->regs->SR & irq_mask;
    if (dsr) {

        Handler handler = (dev->handlers)[iid];
        if (handler) {
            revo_call_handler(handler, (uint32_t)dev->regs);
        }

        dev->regs->SR &= ~dsr;  // reset IRQ inspite of installed handler! @NG
        
    }
}

/* For dispatch routines which service multiple interrupts. */
static INLINE void handle_irq(const timer_dev *dev, uint32_t dier_sr, uint32_t irq_mask, uint32_t iid) {
    if (dier_sr & irq_mask) {                                 
        Handler handler = (dev->handlers)[iid];                
        if (handler) {                                          
            revo_call_handler(handler, (uint32_t)dev->regs);
        }
    }
}

static inline void dispatch_adv_brk(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif
    dispatch_single_irq(dev, TIMER_BREAK_INTERRUPT, TIMER_SR_BIF);
#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}

static inline void dispatch_adv_up(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}

static inline void dispatch_adv_trg_com(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
                            /* Logical OR of SR interrupt flags we end up
                             * handling.  We clear these.  User handlers
                             * must clear overcapture flags, to avoid
                             * wasting time in output mode. */

    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_COMIF, TIMER_COM_INTERRUPT);

    dev->regs->SR &= ~dsr;     // handled ALL enabled interrupts! AFTER ISR itself!

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}

static inline void dispatch_adv_cc(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    
    handle_irq(dev, dsr, TIMER_SR_CC4IF, TIMER_CC4_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC3IF, TIMER_CC3_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);

    dev->regs->SR &= ~dsr;      // handled ALL enabled interrupts! AFTER ISR itself!

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}

static inline void dispatch_general(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;


    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC4IF, TIMER_CC4_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC3IF, TIMER_CC3_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_UIF,   TIMER_UPDATE_INTERRUPT);

    dev->regs->SR &= ~dsr; // handled ALL enabled interrupts! AFTER ISR itself!

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif

}

static inline void dispatch_general_h(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;

    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_UIF,   TIMER_UPDATE_INTERRUPT);

    dev->regs->SR &= ~dsr; // handled ALL enabled interrupts! AFTER ISR itself!

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif

}


// don't count time of basic timer because it used only as timer scheduler's interrupt
static inline void dispatch_basic(const timer_dev *dev) {
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
}

/*
 * Utilities
 */

static void enable_advanced_irq(const timer_dev *dev, timer_interrupt_id id, uint8_t priority);

#define PRIO_DISABLE_FLAG 0x80

static inline void enable_irq(const timer_dev *dev, timer_interrupt_id iid, uint8_t priority) {

    IRQn_Type irq;

    switch(dev->id){
    case 1: /* 1 - advanced */          enable_advanced_irq(dev, iid, priority); return;
    case 2: irq=TIM2_IRQn; break;
    case 3: irq=TIM3_IRQn; break;
    case 4: irq=TIM4_IRQn; break;
    case 5: irq=TIM5_IRQn; break;
    case 6: irq=TIM6_DAC_IRQn;  break;
    case 7: irq=TIM7_IRQn; break;
    case 8: /* 8 - advanced */          enable_advanced_irq(dev, iid, priority); return;
    case 9: irq=TIM1_BRK_TIM9_IRQn; break;
    case 10: irq=TIM1_UP_TIM10_IRQn; break;
    case 11: irq=TIM1_TRG_COM_TIM11_IRQn; break;
    case 12: irq=TIM8_BRK_TIM12_IRQn; break;
    case 13: irq=TIM8_UP_TIM13_IRQn; break;
    case 14: irq=TIM8_TRG_COM_TIM14_IRQn; break;
    
    default: return;
    }

    if(priority  & PRIO_DISABLE_FLAG){
        NVIC_DisableIRQ(irq);
    }else {
        enable_nvic_irq(irq, priority);
    }

}

static void enable_advanced_irq(const timer_dev *dev, timer_interrupt_id id, uint8_t priority) {
    uint8_t is_timer1 = (dev->id == 1);

    IRQn_Type irq=0;
    
    switch (id) {
    case TIMER_UPDATE_INTERRUPT:
        irq = is_timer1 ? TIM1_UP_TIM10_IRQn : TIM8_UP_TIM13_IRQn;
        break;
    case TIMER_CC1_INTERRUPT:
    case TIMER_CC2_INTERRUPT:
    case TIMER_CC3_INTERRUPT:
    case TIMER_CC4_INTERRUPT:
        irq = is_timer1 ? TIM1_CC_IRQn : TIM8_CC_IRQn;
        break;
    case TIMER_COM_INTERRUPT:
    case TIMER_TRG_INTERRUPT:
        irq = is_timer1 ? TIM1_TRG_COM_TIM11_IRQn : TIM8_TRG_COM_TIM14_IRQn;
        break;
    case TIMER_BREAK_INTERRUPT:
        irq = is_timer1 ? TIM1_BRK_TIM9_IRQn : TIM8_BRK_TIM12_IRQn;
        break;
    }
    if(!irq) return;
    if(priority & PRIO_DISABLE_FLAG){
        NVIC_DisableIRQ(irq);
    }else {
        NVIC_EnableIRQ(irq);
        NVIC_SetPriority(irq,priority);
    }
}


void timer_enable_NVICirq(const timer_dev *dev, uint8_t interrupt, uint8_t priority) {
    enable_irq(dev,interrupt, priority);
}


void timer_disable_NVICirq(const timer_dev *dev, uint8_t interrupt){
    enable_irq(dev,interrupt, PRIO_DISABLE_FLAG);
}
