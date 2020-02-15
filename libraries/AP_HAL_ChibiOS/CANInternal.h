/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_UAVCAN

#include <hal.h>
#include <ch.h>
/**
 * Debug output
 */
#ifndef UAVCAN_STM32_LOG
# if 0
#  define UAVCAN_STM32_LOG(fmt, ...)  syslog("uavcan_stm32: " fmt "\n", ##__VA_ARGS__)
# else
#  define UAVCAN_STM32_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */
# define UAVCAN_STM32_IRQ_HANDLER(id)  CH_IRQ_HANDLER(id)
# define UAVCAN_STM32_IRQ_PROLOGUE()    CH_IRQ_PROLOGUE()
# define UAVCAN_STM32_IRQ_EPILOGUE()    CH_IRQ_EPILOGUE()

/**
 * Priority mask for timer and CAN interrupts.
 */
# ifndef UAVCAN_STM32_IRQ_PRIORITY_MASK
#  if (CH_KERNEL_MAJOR == 2)
#   define UAVCAN_STM32_IRQ_PRIORITY_MASK  CORTEX_PRIORITY_MASK(CORTEX_MAX_KERNEL_PRIORITY)
#  else // ChibiOS 3+
#   define UAVCAN_STM32_IRQ_PRIORITY_MASK  CORTEX_MAX_KERNEL_PRIORITY
#  endif
# endif

/**
 * Glue macros
 */
#define UAVCAN_STM32_GLUE2_(A, B)       A##B
#define UAVCAN_STM32_GLUE2(A, B)        UAVCAN_STM32_GLUE2_(A, B)

#define UAVCAN_STM32_GLUE3_(A, B, C)    A##B##C
#define UAVCAN_STM32_GLUE3(A, B, C)     UAVCAN_STM32_GLUE3_(A, B, C)

namespace ChibiOS_CAN {

struct CriticalSectionLocker {
    CriticalSectionLocker()
    {
        chSysSuspend();
    }
    ~CriticalSectionLocker()
    {
        chSysEnable();
    }
};

namespace clock {
uint64_t getUtcUSecFromCanInterrupt();
}
}

#endif //HAL_WITH_UAVCAN
