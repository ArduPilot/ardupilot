/******************************************************************************
 * The GPLv3 License
 *
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
    (c) 2017 night_ghost@ykoctpa.ru
 
 
also see https://github.com/mahowik/MahoRotorF4-Discovery/blob/master/src/drv/drv_pwm_fy90q.c
 
 */

#pragma GCC optimize ("O2")

#include "pwm_in.h"
#include <stdbool.h>
#include "hal_types.h"
#include "timer.h"
#include <systick.h>
#include "gpio_hal.h"
#include <boards.h>
#include "ring_buffer_pulse.h"

#define PPM_CHANNELS 2 // independent input pins


/**************** PWM INPUT **************************************/

// Forward declaration
static void pwmIRQHandler(uint32_t v/*TIM_TypeDef *tim */);

static void pwmInitializeInput(uint8_t ppmsum);

extern const struct TIM_Channel PWM_Channels[];


struct PPM_State PPM_Inputs[PPM_CHANNELS] IN_CCM;
static uint16_t num_ppm_channels = 1;


static void pwmIRQHandler(uint32_t v /* TIM_TypeDef *tim */){
        TIM_TypeDef * tim = (TIM_TypeDef *)v;
        uint8_t i;
        uint16_t val = 0;

	for (i = 0; i < num_ppm_channels; i++)  {

            const struct TIM_Channel *channel = &PWM_Channels[i]; 
	    struct PPM_State         *input   = &PPM_Inputs[i];

            const stm32_pin_info     *p       = &PIN_MAP[channel->pin];
            const timer_dev          *timer   = p->timer_device;

/*
struct PPM_State  {
    uint8_t state;          // 1 or 0
    uint16_t last_val;      // length 
    uint32_t last_pulse;    // time of edge
    volatile pulse_buffer pulses;   // ring buffer
    Pulse pulse_mem[PULSES_QUEUE_SIZE]; // memory
};
*/
	    if (timer->regs == tim && (TIM_GetITStatus(tim, 1<<(p->timer_channel & TIMER_CH_MASK)) == SET)) {

                val = timer_get_capture(timer, p->timer_channel);

	        input->last_pulse = systick_uptime();
        
                uint16_t time;

                if (val > input->last_val)  {
                    time = val - input->last_val;
                } else  {
                    time = ((0xFFFF - input->last_val) + val)+1;
                }
                input->last_val = val;

                if(time>0x7fff) time=0x7fff; // limit to 15 bit

                {
                    Pulse pl={
                        .length  = time, 
                        .state = input->state // we store last state, so state reflects input line
                    };

                    if(!pb_is_full(&input->pulses)){ // save pulse length and state 
                        pb_insert(&input->pulses, pl);
                    }
                }

                if (input->state == 0) { // rising edge
	            input->state = 1;
	            timer_cc_set_pol(timer, p->timer_channel, TIMER_POLARITY_RISING);
	        } else  {               // falling edge
	            input->state = 0;
	            timer_cc_set_pol(timer, p->timer_channel, TIMER_POLARITY_FALLING);
	        }
                
                if(input->handler) revo_call_handler(input->handler, i); // call callback on each edge, SBUS decoding requires only  1.4% of CPU (2.5 for full io_completion)
	    }
	}
}

static inline void pwmInitializeInput(uint8_t ppmsum){
    uint8_t i;
    uint8_t last_tim=99;

    for (i = 0; i < num_ppm_channels; i++)   {
        const struct TIM_Channel *channel = &PWM_Channels[i];

        const stm32_pin_info *p     = &PIN_MAP[channel->pin];
        const gpio_dev       *dev   = p->gpio_device;
        uint8_t               bit   = p->gpio_bit;
        const timer_dev      *timer = p->timer_device;
	
        gpio_set_mode(dev, bit, GPIO_AF_OUTPUT_OD_PU);
        gpio_set_af_mode(dev, bit, timer->af); // connect pin to timer 

        timer_pause(timer);

	if(last_tim != timer->id) {
            configTimeBase(timer, 0, 2000); // 2MHz

	    Revo_hal_handler h = { .isr = pwmIRQHandler };
            timer_attach_all_interrupts(timer, h.h); 
            timer_enable_NVICirq(timer, p->timer_channel, PWM_INT_PRIORITY); // almost highest - bit time is ~10uS only - ~1680 commands	

	    last_tim = timer->id;
        }
            
	// PWM input capture ************************************************************
        timer_ic_set_mode(timer, p->timer_channel, TIM_ICSelection_DirectTI | TIM_ICPSC_DIV1, 0);
        timer_cc_set_pol(timer,  p->timer_channel, TIMER_POLARITY_FALLING);

        timer_cc_enable( timer, p->timer_channel); // enable capture

        timer_resume(timer);

        // enable the CC interrupt request **********************************************
        timer_enable_irq(timer, p->timer_channel);
    }
}

void pwmInit(bool ppmsum) {
    uint8_t i;

    memset(PPM_Inputs, 0, sizeof(PPM_Inputs));

    for (i = 0; i < PPM_CHANNELS; i++){
        struct PPM_State   *input = &PPM_Inputs[i];
	input->state = 1;
        pb_init(&input->pulses, PULSES_QUEUE_SIZE, input->pulse_mem); // init ring buffer
    }

// TODO use parameters!
    num_ppm_channels=PPM_CHANNELS;

    pwmInitializeInput(ppmsum);
}


bool getPPM_Pulse(Pulse *p, uint8_t ch) {
    if(ch>PPM_CHANNELS) return false;
    
    volatile pulse_buffer *bp = &PPM_Inputs[ch].pulses;
    if(pb_is_empty(bp)) return false;

    *p = pb_remove(bp);
    return true;
}

uint16_t getPPM_count(uint8_t ch){
    volatile pulse_buffer *bp = &PPM_Inputs[ch].pulses;
    return pb_full_count(bp);
}
