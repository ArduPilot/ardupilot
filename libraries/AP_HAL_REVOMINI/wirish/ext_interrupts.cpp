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
 *  @file ext_interrupts.c
 *
 *  @brief Wiring-like interface for external interrupts
 */

#include "ext_interrupts.h"

#include "gpio_hal.h"
#include "exti.h"

#include "boards.h"

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode);

/**
 * @brief Attach an interrupt handler to a pin, triggering on the given mode.
 * @param pin     Pin to attach an interrupt handler onto.
 * @param handler Function to call when the external interrupt is triggered.
 * @param mode    Trigger mode for the given interrupt.
 * @see ExtIntTriggerMode
 */
void attachInterrupt(uint8 pin, voidFuncPtr handler, ExtIntTriggerMode mode) {
    if (pin >= BOARD_NR_GPIO_PINS || !handler) {
        return;
    }

    exti_trigger_mode outMode = exti_out_mode(mode);

    exti_attach_interrupt((afio_exti_num)(PIN_MAP[pin].gpio_bit),
                          gpio_exti_port(PIN_MAP[pin].gpio_device),
                          handler,
                          outMode);
}

/**
 * @brief Disable any external interrupt attached to a pin.
 * @param pin Pin number to detach any interrupt from.
 */
void detachInterrupt(uint8 pin) {
    if (pin >= BOARD_NR_GPIO_PINS) {
        return;
    }

    exti_detach_interrupt((afio_exti_num)(PIN_MAP[pin].gpio_bit));
}

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode) {
    switch (mode) {
    case RISING:
        return EXTI_RISING;
    case FALLING:
        return EXTI_FALLING;
    case CHANGE:
        return EXTI_RISING_FALLING;
    }
    // Can't happen
    assert_param(0);
    return (exti_trigger_mode)0;
}
