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
 * @file    chcore.c
 * @brief   AVR architecture port code.
 *
 * @addtogroup AVR_CORE
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/* Executing-in-ISR global flag.*/
bool __avr_in_isr;

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 *
 * @todo    Put into an asm module, use of naked attribute is problematic.
 */
#if !defined(__DOXYGEN__)
__attribute__((naked, weak))
#endif
void _port_switch(thread_t *ntp, thread_t *otp) {

  (void)ntp;
  (void)otp;

  asm volatile ("push    r2");
  asm volatile ("push    r3");
  asm volatile ("push    r4");
  asm volatile ("push    r5");
  asm volatile ("push    r6");
  asm volatile ("push    r7");
  asm volatile ("push    r8");
  asm volatile ("push    r9");
  asm volatile ("push    r10");
  asm volatile ("push    r11");
  asm volatile ("push    r12");
  asm volatile ("push    r13");
  asm volatile ("push    r14");
  asm volatile ("push    r15");
  asm volatile ("push    r16");
  asm volatile ("push    r17");
  asm volatile ("push    r28");
  asm volatile ("push    r29");

#if defined(__CHIBIOS_RT__)
  asm volatile ("movw    r30, r22");
  asm volatile ("in      r0, 0x3d");
  asm volatile ("std     Z+5, r0");
  asm volatile ("in      r0, 0x3e");
  asm volatile ("std     Z+6, r0");

  asm volatile ("movw    r30, r24");
  asm volatile ("ldd     r0, Z+5");
  asm volatile ("out     0x3d, r0");
  asm volatile ("ldd     r0, Z+6");
  asm volatile ("out     0x3e, r0");
#endif

#if defined(__CHIBIOS_NIL__)
  asm volatile ("movw    r30, r22");
  asm volatile ("in      r0, 0x3d");
  asm volatile ("std     Z+0, r0");
  asm volatile ("in      r0, 0x3e");
  asm volatile ("std     Z+1, r0");

  asm volatile ("movw    r30, r24");
  asm volatile ("ldd     r0, Z+0");
  asm volatile ("out     0x3d, r0");
  asm volatile ("ldd     r0, Z+1");
  asm volatile ("out     0x3e, r0");
#endif

  asm volatile ("pop     r29");
  asm volatile ("pop     r28");
  asm volatile ("pop     r17");
  asm volatile ("pop     r16");
  asm volatile ("pop     r15");
  asm volatile ("pop     r14");
  asm volatile ("pop     r13");
  asm volatile ("pop     r12");
  asm volatile ("pop     r11");
  asm volatile ("pop     r10");
  asm volatile ("pop     r9");
  asm volatile ("pop     r8");
  asm volatile ("pop     r7");
  asm volatile ("pop     r6");
  asm volatile ("pop     r5");
  asm volatile ("pop     r4");
  asm volatile ("pop     r3");
  asm volatile ("pop     r2");
  asm volatile ("ret");
}

/**
 * @brief   Start a thread by invoking its work function.
 * @details If the work function returns @p chThdExit() is automatically
 *          invoked.
 */
void _port_thread_start(void) {

  chSysUnlock();
  asm volatile ("movw    r24, r4");
  asm volatile ("movw    r30, r2");
  asm volatile ("icall");
  asm volatile ("call    chThdExit");  /* Used for avr5 Architecture. */
}

/** @} */
