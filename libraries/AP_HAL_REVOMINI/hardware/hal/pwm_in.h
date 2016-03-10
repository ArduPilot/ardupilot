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
 *  @file pwm.h
 *
 *  @brief Arduino-compatible PWM interface.
 */

#ifndef _PWM_IN_H_
#define _PWM_IN_H_

#include "hal_types.h"
#include <stdbool.h>

#ifdef __cplusplus
  extern "C" {
#endif
 
/**
 * Set the PWM duty on the given pin.
 *
 * User code is expected to determine and honor the maximum value
 * (based on the configured period).
 *
 * @param pin PWM output pin
 * @param duty_cycle Duty cycle to set.
 */

void pwmInit(bool ppmsum);
uint16_t pwmRead(uint8_t channel);
void attachPWMCaptureCallback(void (*callback)(uint8_t state, uint16_t value));
extern uint8_t _is_ppmsum;


#ifdef __cplusplus
  }
#endif

#endif

