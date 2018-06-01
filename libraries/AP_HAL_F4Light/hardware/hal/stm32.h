/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs, LLC.
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
 * @file stm32.h
 * @brief STM32 chip-specific definitions
 */

#ifndef _STM32_H_
#define _STM32_H_


#define STM32_NR_INTERRUPTS 60
#define NR_INTERRUPTS STM32_NR_INTERRUPTS

#if defined(MCU_STM32F103RB)
    /* e.g., LeafLabs Maple */

    #define STM32_NR_GPIO_PORTS          4
    #define STM32_DELAY_US_MULT         12
    #define STM32_SRAM_END              ((void*)0x20005000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F103ZE)
    /* e.g., LeafLabs Maple Native */

    #define STM32_NR_GPIO_PORTS          7
    #define STM32_DELAY_US_MULT         12
    #define STM32_SRAM_END              ((void*)0x20010000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F103CB)
    /* e.g., LeafLabs Maple Mini */

    /* This STM32_NR_GPIO_PORTS value is not, strictly speaking, true.
     * But only pins 0 and 1 exist, and they're used for OSC on the
     * Mini, so we'll live with this for now. */
    #define STM32_NR_GPIO_PORTS          3
    #define STM32_DELAY_US_MULT         12
    #define STM32_SRAM_END              ((void*)0x20005000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F103RE)
    /* e.g., LeafLabs Maple RET6 edition */

    #define STM32_NR_GPIO_PORTS          4
    #define STM32_DELAY_US_MULT         12
    #define STM32_SRAM_END              ((void*)0x20010000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F103VE)
    /* e.g., LeafLabs Maple Native */

    #define STM32_NR_GPIO_PORTS          5
    #define STM32_DELAY_US_MULT         12
    #define STM32_SRAM_END              ((void*)0x20010000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F205VE)
    #define STM32_TICKS_PER_US          (SystemCoreClock / 1000000)
    #define STM32_NR_GPIO_PORTS          5
    #define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)
    #define STM32_SRAM_END              ((void*)0x20010000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F406VG)
    #define STM32_TICKS_PER_US          (SystemCoreClock / 1000000)
    #define STM32_NR_GPIO_PORTS          5
    #define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)
    #define STM32_SRAM_END              ((void*)0x20020000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#elif defined(MCU_STM32F407VG) || defined(stm32f407vg) || defined(mcu_stm32f405rg) || defined(MCU_STM32F405RG)
    #define STM32_TICKS_PER_US          (SystemCoreClock / 1000000)
    #define STM32_NR_GPIO_PORTS          5
    #define STM32_DELAY_US_MULT         (STM32_TICKS_PER_US/3)
    #define STM32_SRAM_END              ((void*)0x20020000)
    #define STM32_CCM_END               ((void*)0x10010000)

    #define NR_GPIO_PORTS               STM32_NR_GPIO_PORTS
    #define DELAY_US_MULT               STM32_DELAY_US_MULT

#else

#error "No MCU type specified. Add something like -DMCU_STM32F103RB "   \
       "to your compiler arguments (probably in a Makefile)."

#endif

#endif  /* _STM32_H_ */
