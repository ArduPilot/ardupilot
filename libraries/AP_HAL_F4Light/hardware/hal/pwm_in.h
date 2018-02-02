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

#ifndef _PWM_IN_H_
#define _PWM_IN_H_

#include <hal.h>
#include "hal_types.h"
#include <stdbool.h>
#include <strings.h>
#include "timer.h"


#ifdef __cplusplus
  extern "C" {
#endif

typedef struct PULSE {
    unsigned int length:15;
    bool state:1;
} Pulse;

#define PULSES_QUEUE_SIZE (25*12*2) // 1 full frame by 25 bytes (12 bits, 2 measures per bit) each

#include "ring_buffer_pulse.h"


struct PPM_State  {
    Handler handler;
    uint32_t last_pulse;    // time of edge
    volatile pulse_buffer pulses;   // ring buffer
    uint16_t last_val;      // length 
    uint8_t state;          // 1 or 0
    Pulse pulse_mem[PULSES_QUEUE_SIZE]; // memory
};

extern struct PPM_State PPM_Inputs[];


struct TIM_Channel {
    uint8_t pin;       // pin number
}; 

extern const struct TIM_Channel PWM_Channels[];
#define PWM_CHANNELS (sizeof(PWM_Channels) / sizeof(struct TIM_Channel) )
 

void pwmInit(bool ppmsum);

bool getPPM_Pulse(Pulse *p, uint8_t ch);
uint16_t getPPM_count(uint8_t ch);

static inline void pwm_setHandler(Handler handler, uint8_t ch){
    PPM_Inputs[ch].handler = handler;
}

#ifdef __cplusplus
  }
#endif

#endif

