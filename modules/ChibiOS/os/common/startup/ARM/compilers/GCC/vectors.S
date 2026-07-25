/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ARM/compilers/GCC/vectors.s
 * @brief   Interrupt vectors for ARM devices.
 *
 * @defgroup ARM_VECTORS ARM Exception Vectors
 * @{
 */

#if defined(__DOXYGEN__)
/**
 * @brief   Unhandled exceptions handler.
 * @details Any undefined exception vector points to this function by default.
 *          This function simply stops the system into an infinite loop.
 * @note    The default implementation is a weak symbol, the application
 *          can override the default implementation.
 *
 * @notapi
 */
void _unhandled_exception(void) {}
#endif

#if !defined(__DOXYGEN__)

                .section .vectors, "ax"
                .code   32
                .balign 4

/*
 * System entry points.
 */
                .global _start
_start:
                ldr     pc, _reset
                ldr     pc, _undefined
                ldr     pc, _swi
                ldr     pc, _prefetch
                ldr     pc, _abort
                nop
                ldr     pc, _irq
                ldr     pc, _fiq

_reset:
                .word   Boot_Handler
_undefined:
                .word   Und_Handler
_swi:
                .word   Swi_Handler
_prefetch:
                .word   Prefetch_Handler
_abort:
                .word   Abort_Handler
_fiq:
                .word   Fiq_Handler
_irq:
                .word   Irq_Handler

/*
 * Default exceptions handlers. The handlers are declared weak in order to be
 * replaced by the real handling code. Everything is defaulted to an infinite
 * loop.
 */
                .weak   Reset_Handler
Reset_Handler:
                .weak   Und_Handler
Und_Handler:
                .weak   Swi_Handler
Swi_Handler:
                .weak   Prefetch_Handler
Prefetch_Handler:
                .weak   Abort_Handler
Abort_Handler:
                .weak   Fiq_Handler
Fiq_Handler:
                .weak   Irq_Handler
Irq_Handler:
                .weak   _unhandled_exception
_unhandled_exception:
                b       _unhandled_exception
/*
 * Default boot handler. Jump to Reset_Handler.
 */
                .section .boot, "ax"
                .weak	Boot_Handler
Boot_Handler:
                b       Reset_Handler
#endif

/** @} */
