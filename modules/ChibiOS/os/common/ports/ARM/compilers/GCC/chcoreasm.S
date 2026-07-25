/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ARM/compilers/GCC/chcoreasm.S
 * @brief   ARM architecture port low level code.
 *
 * @addtogroup ARM_GCC_CORE
 * @{
 */

#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "armparams.h"

#define FALSE 0
#define TRUE 1

#if !defined(__DOXYGEN__)

/*
 * RTOS-specific context offset.
 */
#if defined(_CHIBIOS_RT_CONF_)
#define CONTEXT_OFFSET  12

#elif defined(_CHIBIOS_NIL_CONF_)
#define CONTEXT_OFFSET  0

#else
#error "invalid chconf.h"
#endif

                .set    MODE_USR, 0x10
                .set    MODE_FIQ, 0x11
                .set    MODE_IRQ, 0x12
                .set    MODE_SVC, 0x13
                .set    MODE_ABT, 0x17
                .set    MODE_UND, 0x1B
                .set    MODE_SYS, 0x1F

                .equ    I_BIT, 0x80
                .equ    F_BIT, 0x40

                .text


                .balign 16

                .code   32
                .global __port_switch_arm
__port_switch_arm:
                stmfd   sp!, {r4, r5, r6, r7, r8, r9, r10, r11, lr}
                str     sp, [r1, #CONTEXT_OFFSET]
                ldr     sp, [r0, #CONTEXT_OFFSET]
                ldmfd   sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}

/*
 * Common IRQ code. It expects a macro ARM_IRQ_VECTOR_REG with the address
 * of a register holding the address of the ISR to be invoked, the ISR
 * then returns in the common epilogue code where the context switch will
 * be performed, if required.
 * System stack frame structure after a context switch in the
 * interrupt handler:
 *
 * High +------------+
 *      |   LR_USR   | -+
 *      |     r12    |  |
 *      |     r3     |  |
 *      |     r2     |  | External context: IRQ handler frame
 *      |     r1     |  |
 *      |     r0     |  |
 *      |   LR_IRQ   |  |   (user code return address)
 *      |   PSR_USR  | -+   (user code status)
 *      |    ....    | <- chSchDoReschedule() stack frame, optimize it for space
 *      |     LR     | -+   (system code return address)
 *      |     r11    |  |
 *      |     r10    |  |
 *      |     r9     |  |
 *      |     r8     |  | Internal context: chSysSwitch() frame
 *      |     r7     |  |
 *      |     r6     |  |
 *      |     r5     |  |
 * SP-> |     r4     | -+
 * Low  +------------+
 */
                .balign 16
                .code   32
                .global Irq_Handler
Irq_Handler:
                stmfd   sp!, {r0-r3, r12, lr}
                ldr     r0, =ARM_IRQ_VECTOR_REG
                ldr     r0, [r0]
                ldr     lr, =_irq_ret_arm       // ISR return point.
                bx      r0                      // Calling the ISR.
_irq_ret_arm:
                cmp     r0, #0
                ldmfd   sp!, {r0-r3, r12, lr}
                subeqs  pc, lr, #4              // No reschedule, returns.

                // Now the frame is created in the system stack, the IRQ
                // stack is empty.
                msr     CPSR_c, #MODE_SYS | I_BIT
                stmfd   sp!, {r0-r3, r12, lr}
                msr     CPSR_c, #MODE_IRQ | I_BIT
                mrs     r0, SPSR
                mov     r1, lr
                msr     CPSR_c, #MODE_SYS | I_BIT
                stmfd   sp!, {r0, r1}           // Push R0=SPSR, R1=LR_IRQ.

                // Context switch.
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_lock
#endif
                bl      chSchDoPreemption
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_unlock
#endif

                // Re-establish the IRQ conditions again.
                ldmfd   sp!, {r0, r1}           // Pop R0=SPSR, R1=LR_IRQ.
                msr     CPSR_c, #MODE_IRQ | I_BIT
                msr     SPSR_fsxc, r0
                mov     lr, r1
                msr     CPSR_c, #MODE_SYS | I_BIT
                ldmfd   sp!, {r0-r3, r12, lr}
                msr     CPSR_c, #MODE_IRQ | I_BIT
                subs    pc, lr, #4

/*
 * Threads trampoline code.
 * NOTE: The threads always start in ARM mode and then switches to the
 * thread-function mode.
 */
                .balign 16
                .code   32
                .globl  __port_thread_start
__port_thread_start:
#if CH_DBG_SYSTEM_STATE_CHECK
                bl      __dbg_check_unlock
#endif
                msr     CPSR_c, #MODE_SYS
                mov     r0, r5
                mov     lr, pc
                bx      r4
                mov     r0, #0              /* MSG_OK */
                bl      chThdExit
.zombies:       b       .zombies

#endif /* !defined(__DOXYGEN__) */

/** @} */
