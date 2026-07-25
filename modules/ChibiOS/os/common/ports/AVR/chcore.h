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
 * @file    chcore.h
 * @brief   AVR port macros and structures.
 *
 * @addtogroup AVR_CORE
 * @{
 */

#ifndef CHCORE_H
#define CHCORE_H

#include <avr/io.h>
#include <avr/interrupt.h>

extern bool __avr_in_isr;

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Port Capabilities and Constants
 * @{
 */
/**
 * @brief   This port supports a realtime counter.
 */
#define PORT_SUPPORTS_RT                FALSE

/**
 * @brief   Natural alignment constant.
 * @note    It is the minimum alignment for pointer-size variables.
 */
#define PORT_NATURAL_ALIGN              1U

/**
 * @brief   Stack alignment constant.
 * @note    It is the alignment required for the stack pointer.
 */
#define PORT_STACK_ALIGN                1U

/**
 * @brief   Working Areas alignment constant.
 * @note    It is the alignment to be enforced for thread working areas.
 */
#define PORT_WORKING_AREA_ALIGN         1U
/** @} */

/**
 * @name    Architecture and Compiler
 * @{
 */
/**
 * @brief   Macro defining an AVR architecture.
 */
#define PORT_ARCHITECTURE_AVR

/**
 * @brief   Macro defining the specific AVR architecture.
 */
#define PORT_ARCHITECTURE_AVR_MEGAAVR

/**
 * @brief   Name of the implemented architecture.
 */
#define PORT_ARCHITECTURE_NAME          "AVR"

/**
 * @brief   Name of the architecture variant.
 */
#define PORT_CORE_VARIANT_NAME          "MegaAVR"

/**
 * @brief   Compiler name and version.
 */
#if defined(__GNUC__) || defined(__DOXYGEN__)
#define PORT_COMPILER_NAME              "GCC " __VERSION__

#else
#error "unsupported compiler"
#endif

/**
 * @brief   Port-specific information string.
 */
#define PORT_INFO                       "None"
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Stack size for the system idle thread.
 * @details This size depends on the idle thread implementation, usually
 *          the idle thread should take no more space than those reserved
 *          by @p PORT_INT_REQUIRED_STACK.
 */
#if !defined(PORT_IDLE_THREAD_STACK_SIZE) || defined(__DOXYGEN__)
#define PORT_IDLE_THREAD_STACK_SIZE     8
#endif

/**
 * @brief   Per-thread stack overhead for interrupts servicing.
 * @details This constant is used in the calculation of the correct working
 *          area size.
 */
#if !defined(PORT_INT_REQUIRED_STACK) || defined(__DOXYGEN__)
#define PORT_INT_REQUIRED_STACK         32
#endif

/**
 * @brief   Enables a "wait for interrupt" instruction in the idle loop.
 */
#if !defined(PORT_AVR_WFI_SLEEP_IDLE) || defined(__DOXYGEN__)
#define PORT_AVR_WFI_SLEEP_IDLE      FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Type of stack and memory alignment enforcement.
 * @note    In this architecture the stack alignment is enforced to 8 bits.
 */
typedef uint8_t stkalign_t;

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during a
 *          preemption-capable interrupt handler.
 * @note    R2 and R13 are not saved because those are assumed to be immutable
 *          during the system life cycle.
 */
struct port_extctx {
  uint8_t       _next;
  uint8_t       r31;
  uint8_t       r30;
  uint8_t       r27;
  uint8_t       r26;
  uint8_t       r25;
  uint8_t       r24;
  uint8_t       r23;
  uint8_t       r22;
  uint8_t       r21;
  uint8_t       r20;
  uint8_t       r19;
  uint8_t       r18;
  uint8_t       sr;
  uint8_t       r1;
  uint8_t       r0;
#if defined(__AVR_3_BYTE_PC__)
  uint8_t       pcx;
#endif
  uint16_t      pc;
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switching.
 * @note    R2 and R13 are not saved because those are assumed to be immutable
 *          during the system life cycle.
 * @note    LR is stored in the caller context so it is not present in this
 *          structure.
 */
struct port_intctx {
  uint8_t       _next;
  uint8_t       r29;
  uint8_t       r28;
  uint8_t       r17;
  uint8_t       r16;
  uint8_t       r15;
  uint8_t       r14;
  uint8_t       r13;
  uint8_t       r12;
  uint8_t       r11;
  uint8_t       r10;
  uint8_t       r9;
  uint8_t       r8;
  uint8_t       r7;
  uint8_t       r6;
  uint8_t       r5;
  uint8_t       r4;
  uint8_t       r3;
  uint8_t       r2;
#if defined(__AVR_3_BYTE_PC__)
  uint8_t       pcx;
#endif
  uint8_t       pcl;
  uint8_t       pch;
};

/**
 * @brief   Platform dependent part of the @p thread_t structure.
 * @details This structure usually contains just the saved stack pointer
 *          defined as a pointer to a @p port_intctx structure.
 */
struct port_context {
  struct port_intctx *sp;
};

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p port_intctx structure.
 */
#if defined(__AVR_3_BYTE_PC__) || defined(__DOXYGEN__)
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) {                      \
  tp->ctx.sp = (struct port_intctx *)((uint8_t *)(wtop) -                   \
                                      sizeof(struct port_intctx));          \
  tp->ctx.sp->r2  = (uint8_t)(0xff & (uint16_t)pf);                         \
  tp->ctx.sp->r3  = (uint8_t)((uint16_t)(pf) >> 8);                         \
  tp->ctx.sp->r4  = (uint8_t)(0xff & (uint16_t)arg);                        \
  tp->ctx.sp->r5  = (uint8_t)((uint16_t)(arg) >> 8);                        \
  tp->ctx.sp->pcx = (uint8_t)0;                                             \
  tp->ctx.sp->pcl = (uint16_t)_port_thread_start >> 8;                      \
  tp->ctx.sp->pch = (uint8_t)(0xff & (uint16_t)_port_thread_start);         \
}
#else /* !__AVR_3_BYTE_PC__ */
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) {                      \
  tp->ctx.sp = (struct port_intctx *)((uint8_t *)(wtop) -                   \
                                      sizeof(struct port_intctx));          \
  tp->ctx.sp->r2  = (uint8_t)(0xff & (uint16_t)pf);                         \
  tp->ctx.sp->r3  = (uint8_t)((uint16_t)(pf) >> 8);                         \
  tp->ctx.sp->r4  = (uint8_t)(0xff & (uint16_t)arg);                        \
  tp->ctx.sp->r5  = (uint8_t)((uint16_t)(arg) >> 8);                        \
  tp->ctx.sp->pcl = (uint16_t)_port_thread_start >> 8;                      \
  tp->ctx.sp->pch = (uint8_t)(0xff & (uint16_t)_port_thread_start);         \
}
#endif /* !__AVR_3_BYTE_PC__ */

/**
 * @brief   Computes the thread working area global size.
 * @note    There is no need to perform alignments in this macro.
 */
#define PORT_WA_SIZE(n) ((sizeof(struct port_intctx) - 1) +                \
                         (sizeof(struct port_extctx) - 1) +                \
                         ((size_t)(n)) + ((size_t)(PORT_INT_REQUIRED_STACK)))

/**
 * @brief   Static working area allocation.
 * @details This macro is used to allocate a static thread working area
 *          aligned as both position and size.
 *
 * @param[in] s         the name to be assigned to the stack array
 * @param[in] n         the stack size to be assigned to the thread
 */
#define PORT_WORKING_AREA(s, n)                                             \
  stkalign_t s[THD_WORKING_AREA_SIZE(n) / sizeof (stkalign_t)]

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n) false

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n) false

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers
 *          enabled to invoke system APIs.
 * @note    This code tricks the compiler to save all the specified registers
 *          by "touching" them.
 */
#define PORT_IRQ_PROLOGUE() {                                               \
  asm ("" : : : "r18", "r19", "r20", "r21", "r22", "r23", "r24",            \
                "r25", "r26", "r27", "r30", "r31");                         \
  __avr_in_isr = true;                                                      \
}

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE() {                                               \
  __avr_in_isr = false;                                                     \
  __dbg_check_lock();                                                       \
  if (chSchIsPreemptionRequired())                                          \
    chSchDoPreemption();                                                    \
  __dbg_check_unlock();                                                     \
}

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#define PORT_IRQ_HANDLER(id) ISR(id)

/**
 * @brief   Fast IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#define PORT_FAST_IRQ_HANDLER(id) ISR(id)

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#define port_switch(ntp, otp) {                                             \
  _port_switch(ntp, otp);                                                   \
  asm volatile ("" : : : "memory");                                         \
}

/**
 * @brief   Returns a word representing a critical section status.
 *
 * @return              The critical section status.
 */
#define port_get_lock_status() port_get_irq_status()

/**
 * @brief   Determines if in a critical section.
 *
 * @param[in] sts       status word returned by @p port_get_lock_status()
 * @return              The current status.
 * @retval false        if running outside a critical section.
 * @retval true         if running within a critical section.
 */
#define port_is_locked(sts) (!port_irq_enabled(sts))


/**
 * @brief   Port-related initialization code.
 * @note    This function is empty in this port.
 */
#define port_init(oip) {                                                    \
  __avr_in_isr = true;                                                      \
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

#ifdef __cplusplus
extern "C" {
#endif
  void _port_switch(thread_t *ntp, thread_t *otp);
  void _port_thread_start(void);
#ifdef __cplusplus
}
#endif

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Returns a word encoding the current interrupts status.
 *
 * @return              The interrupts status.
 */
static inline syssts_t port_get_irq_status(void) {

  return SREG;
}

/**
 * @brief   Checks the interrupt status.
 *
 * @param[in] sts       the interrupt status word
 *
 * @return              The interrupt status.
 * @retval false        the word specified a disabled interrupts status.
 * @retval true         the word specified an enabled interrupts status.
 */
static inline bool port_irq_enabled(syssts_t sts) {

  return (bool)((sts & 0x80) != 0);
}

/**
 * @brief   Determines the current execution context.
 *
 * @return              The execution context.
 * @retval false        not running in ISR mode.
 * @retval true         running in ISR mode.
 */
static inline bool port_is_isr_context(void) {

  return __avr_in_isr;
}

/**
 * @brief   Kernel-lock action.
 * @details Usually this function just disables interrupts but may perform more
 *          actions.
 */
static inline void port_lock(void) {

  asm volatile ("cli" : : : "memory");
}

/**
 * @brief   Kernel-unlock action.
 * @details Usually this function just enables interrupts but may perform more
 *          actions.
 */
static inline void port_unlock(void) {

  asm volatile ("sei" : : : "memory");
}

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @details This function is invoked before invoking I-class APIs from
 *          interrupt handlers. The implementation is architecture dependent,
 *          in its simplest form it is void.
 * @note    This function is empty in this port.
 */
static inline void port_lock_from_isr(void) {

}

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @details This function is invoked after invoking I-class APIs from interrupt
 *          handlers. The implementation is architecture dependent, in its
 *          simplest form it is void.
 * @note    This function is empty in this port.
 */
static inline void port_unlock_from_isr(void) {

}

/**
 * @brief   Disables all the interrupt sources.
 * @note    Of course non-maskable interrupt sources are not included.
 */
static inline void port_disable(void) {

  asm volatile ("cli" : : : "memory");
}

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 * @note    Interrupt sources above kernel level remains enabled.
 */
static inline void port_suspend(void) {

  asm volatile ("cli" : : : "memory");
}

/**
 * @brief   Enables all the interrupt sources.
 */
static inline void port_enable(void) {

  asm volatile ("sei" : : : "memory");
}

/**
 * @brief   Enters an architecture-dependent IRQ-waiting mode.
 * @details The function is meant to return when an interrupt becomes pending.
 *          The simplest implementation is an empty function or macro but this
 *          would not take advantage of architecture-specific power saving
 *          modes.
 */
static inline void port_wait_for_interrupt(void) {

#if PORT_AVR_WFI_SLEEP_IDLE
  asm volatile ("sleep" : : : "memory");
#endif
}

/**
 * @brief   Returns the current value of the realtime counter.
 *
 * @return              The realtime counter value.
 */
static inline rtcnt_t port_rt_get_counter_value(void) {

  return 0;
}

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module late inclusions.                                                   */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

#if CH_CFG_ST_TIMEDELTA > 0
#include "chcore_timer.h"
#endif /* CH_CFG_ST_TIMEDELTA > 0 */

#endif /* !defined(_FROM_ASM_) */

#endif /* CHCORE_H */

/** @} */
