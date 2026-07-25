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
 * @file    PPC/chcore.h
 * @brief   Power e200 port macros and structures.
 *
 * @addtogroup PPC_CORE
 * @{
 */

#ifndef CHCORE_H
#define CHCORE_H

#if defined(__ghs__) && !defined(_FROM_ASM_)
#include <ppc_ghs.h>
#endif

#include "intc.h"

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
#define PORT_NATURAL_ALIGN              sizeof (void *)

/**
 * @brief   Stack alignment constant.
 * @note    It is the alignment required for the stack pointer.
 */
#define PORT_STACK_ALIGN                sizeof (stkalign_t)

/**
 * @brief   Working Areas alignment constant.
 * @note    It is the alignment to be enforced for thread working areas.
 */
#define PORT_WORKING_AREA_ALIGN         sizeof (stkalign_t)
/** @} */

/**
 * @name    Architecture and Compiler
 * @{
 */
/**
 * @brief   Macro defining an PPC architecture.
 */
#define PORT_ARCHITECTURE_PPC

/**
 * @brief   Macro defining the specific PPC architecture.
 */
#define PORT_ARCHITECTURE_PPC_E200

/**
 * @brief   Name of the implemented architecture.
 */
#define PORT_ARCHITECTURE_NAME          "Power Architecture e200"

/**
 * @brief   Compiler name and version.
 */
#if (defined(__GNUC__) && !defined(__ghs__)) || defined(__DOXYGEN__)
#define PORT_COMPILER_NAME              "GCC " __VERSION__

#elif defined(__MWERKS__)
#define PORT_COMPILER_NAME              "CW"

#elif defined(__ghs__)
#define PORT_COMPILER_NAME              "GHS"

#else
#error "unsupported compiler"
#endif
/** @} */

/**
 * @name    E200 core variants
 * @{
 */
#define PPC_VARIANT_e200z0              200
#define PPC_VARIANT_e200z2              202
#define PPC_VARIANT_e200z3              203
#define PPC_VARIANT_e200z4              204
/** @} */

/* Inclusion of the PPC implementation specific parameters.*/
#include "ppcparams.h"
#include "vectors.h"

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Stack size for the system idle thread.
 * @details This size depends on the idle thread implementation, usually
 *          the idle thread should take no more space than those reserved
 *          by @p PORT_INT_REQUIRED_STACK.
 * @note    In this port it is set to 32 because the idle thread does have
 *          a stack frame when compiling without optimizations. You may
 *          reduce this value to zero when compiling with optimizations.
 */
#if !defined(PORT_IDLE_THREAD_STACK_SIZE) || defined(__DOXYGEN__)
#define PORT_IDLE_THREAD_STACK_SIZE     32
#endif

/**
 * @brief   Per-thread stack overhead for interrupts servicing.
 * @details This constant is used in the calculation of the correct working
 *          area size.
 * @note    In this port this value is conservatively is set to 256 because
 *          there is no separate interrupts stack (yet).
 */
#if !defined(PORT_INT_REQUIRED_STACK) || defined(__DOXYGEN__)
#define PORT_INT_REQUIRED_STACK         256
#endif

/**
 * @brief   Use VLE instruction set.
 * @note    This parameter is usually set in the Makefile.
 */
#if !defined(PPC_USE_VLE) || defined(__DOXYGEN__)
#define PPC_USE_VLE                     TRUE
#endif

/**
 * @brief   Enables the use of the @p WFI instruction.
 */
#if !defined(PPC_ENABLE_WFI_IDLE) || defined(__DOXYGEN__)
#define PPC_ENABLE_WFI_IDLE             FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if PPC_USE_VLE && !PPC_SUPPORTS_VLE
#error "the selected MCU does not support VLE instructions set"
#endif

#if !PPC_USE_VLE && !PPC_SUPPORTS_BOOKE
#error "the selected MCU does not support BookE instructions set"
#endif

/**
 * @brief   Name of the architecture variant.
 */
#if (PPC_VARIANT == PPC_VARIANT_e200z0) || defined(__DOXYGEN__)

#if !defined(CH_CUSTOMER_LIC_PORT_E200Z0)
#error "CH_CUSTOMER_LIC_PORT_E200Z0 not defined"
#endif

#if CH_CUSTOMER_LIC_PORT_E200Z0 == FALSE
#error "ChibiOS Power e200z0 port not licensed"
#endif

#define PORT_CORE_VARIANT_NAME          "e200z0"

#elif PPC_VARIANT == PPC_VARIANT_e200z2

#if !defined(CH_CUSTOMER_LIC_PORT_E200Z2)
#error "CH_CUSTOMER_LIC_PORT_E200Z2 not defined"
#endif

#if CH_CUSTOMER_LIC_PORT_E200Z2 == FALSE
#error "ChibiOS Power e200z2 port not licensed"
#endif

#define PORT_CORE_VARIANT_NAME          "e200z2"

#elif PPC_VARIANT == PPC_VARIANT_e200z3

#if !defined(CH_CUSTOMER_LIC_PORT_E200Z3)
#error "CH_CUSTOMER_LIC_PORT_E200Z3 not defined"
#endif

#if CH_CUSTOMER_LIC_PORT_E200Z3 == FALSE
#error "ChibiOS Power e200z3 port not licensed"
#endif

#define PORT_CORE_VARIANT_NAME          "e200z3"

#elif PPC_VARIANT == PPC_VARIANT_e200z4

#if !defined(CH_CUSTOMER_LIC_PORT_E200Z4)
#error "CH_CUSTOMER_LIC_PORT_E200Z4 not defined"
#endif

#if CH_CUSTOMER_LIC_PORT_E200Z4 == FALSE
#error "ChibiOS Power e200z4 port not licensed"
#endif

#define PORT_CORE_VARIANT_NAME          "e200z4"

#else
#error "unknown or unsupported PowerPC variant specified"
#endif

/**
 * @brief   Port-specific information string.
 */
#if PPC_USE_VLE
#define PORT_INFO                       "VLE mode"
#else
#define PORT_INFO                       "Book-E mode"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Type of stack and memory alignment enforcement.
 * @note    In this architecture the stack alignment is enforced to 64 bits.
 */
typedef uint64_t stkalign_t;

/**
 * @brief   Generic PPC register.
 */
typedef void *regppc_t;

/**
 * @brief   Mandatory part of a stack frame.
 */
struct port_eabi_frame {
  uint32_t      slink;          /**< Stack back link.                       */
  uint32_t      shole;          /**< Stack hole for LR storage.             */
};

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during a
 *          preemption-capable interrupt handler.
 * @note    R2 and R13 are not saved because those are assumed to be immutable
 *          during the system life cycle.
 */
struct port_extctx {
  struct port_eabi_frame frame;
  /* Start of the e_stmvsrrw frame (offset 8).*/
  regppc_t      pc;
  regppc_t      msr;
  /* Start of the e_stmvsprw frame (offset 16).*/
  regppc_t      cr;
  regppc_t      lr;
  regppc_t      ctr;
  regppc_t      xer;
  /* Start of the e_stmvgprw frame (offset 32).*/
  regppc_t      r0;
  regppc_t      r3;
  regppc_t      r4;
  regppc_t      r5;
  regppc_t      r6;
  regppc_t      r7;
  regppc_t      r8;
  regppc_t      r9;
  regppc_t      r10;
  regppc_t      r11;
  regppc_t      r12;
  regppc_t      padding;
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
  regppc_t      cr;                 /* Part of it is not volatile...        */
  regppc_t      r14;
  regppc_t      r15;
  regppc_t      r16;
  regppc_t      r17;
  regppc_t      r18;
  regppc_t      r19;
  regppc_t      r20;
  regppc_t      r21;
  regppc_t      r22;
  regppc_t      r23;
  regppc_t      r24;
  regppc_t      r25;
  regppc_t      r26;
  regppc_t      r27;
  regppc_t      r28;
  regppc_t      r29;
  regppc_t      r30;
  regppc_t      r31;
  regppc_t      padding;
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
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) {                      \
  uint8_t *sp = (uint8_t *)(wtop) - sizeof(struct port_eabi_frame);         \
  ((struct port_eabi_frame *)sp)->slink = 0;                                \
  ((struct port_eabi_frame *)sp)->shole = (uint32_t)_port_thread_start;     \
  (tp)->ctx.sp = (struct port_intctx *)(sp - sizeof(struct port_intctx));   \
  (tp)->ctx.sp->r31 = (regppc_t)(arg);                                      \
  (tp)->ctx.sp->r30 = (regppc_t)(pf);                                       \
}

/**
 * @brief   Computes the thread working area global size.
 * @note    There is no need to perform alignments in this macro.
 */
#define PORT_WA_SIZE(n) (sizeof(struct port_intctx) +                       \
                         sizeof(struct port_extctx) +                       \
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
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_PROLOGUE()

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE()

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#ifdef __cplusplus
#define PORT_IRQ_HANDLER(id) extern "C" void id(void)
#else
#define PORT_IRQ_HANDLER(id) void id(void)
#endif

/**
 * @brief   Fast IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#ifdef __cplusplus
#define PORT_FAST_IRQ_HANDLER(id) extern "C" void id(void)
#else
#define PORT_FAST_IRQ_HANDLER(id) void id(void)
#endif

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n)                                       \
  (((n) >= 0U) && ((n) < INTC_PRIORITY_LEVELS))

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n)                                \
    (((n) >= 0U) && ((n) < INTC_PRIORITY_LEVELS))

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
#if !CH_DBG_ENABLE_STACK_CHECK || defined(__DOXYGEN__)
#define port_switch(ntp, otp) _port_switch(ntp, otp)
#else
#define port_switch(ntp, otp) {                                             \
  register struct port_intctx *sp asm ("%r1");                              \
  if ((stkalign_t *)(void *)(sp - 1) < otp->wabase)                         \
    CH_CFG_STACK_OVERFLOW_HOOK(otp);                                        \
  _port_switch(ntp, otp);                                                   \
}
#endif

/**
 * @brief   Writes to a special register.
 *
 * @param[in] spr       special register number
 * @param[in] val       value to be written, must be an automatic variable
 */
#if !defined(__ghs__) || defined(__DOXYGEN__)
#define port_write_spr(spr, val)                                            \
  asm volatile ("mtspr   %[p0], %[p1]" : : [p0] "n" (spr), [p1] "r" (val))
#else
#define port_write_spr(spr, val)                                            \
  __MTSPR(spr, val);
#endif

/**
 * @brief   Reads a special register.
 *
 * @param[in] spr       special register number
 * @param[in] val       returned value, must be an automatic variable
 */
#if !defined(__ghs__) || defined(__DOXYGEN__)
#define port_read_spr(spr, val)                                             \
  asm volatile ("mfspr   %[p0], %[p1]" : [p0] "=r" (val) : [p1] "n" (spr))
#else
#define port_read_spr(spr, val)                                             \
  val = __MFSPR(spr)
#endif

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

extern void _IVOR4(void);
extern void _IVOR10(void);

/**
 * @brief   Kernel port layer initialization.
 * @details IVOR4 and IVOR10 initialization.
 */
static inline void port_init(os_instance_t *oip) {
  uint32_t n;
  unsigned i;

  (void)oip;

  /* Initializing the SPRG0 register to zero, it is required for interrupts
     handling.*/
  n = 0;
  port_write_spr(272, n);

#if PPC_SUPPORTS_IVORS
  {
    /* The CPU supports IVOR registers, the kernel requires IVOR4 and IVOR10
       and the initialization is performed here.*/
    port_write_spr(404, (uint32_t)_IVOR4);

#if PPC_SUPPORTS_DECREMENTER
    port_write_spr(410, (uint32_t)_IVOR10);
#endif
  }
#endif

  /* INTC initialization, software vector mode, 4 bytes vectors, starting
     at priority 0.*/
  INTC_BCR = 0;
  for (i = 0; i < PPC_CORE_NUMBER; i++) {
    INTC_CPR(i)   = 0;
    INTC_IACKR(i) = (uint32_t)_vectors;
  }
}

/**
 * @brief   Returns a word encoding the current interrupts status.
 *
 * @return              The interrupts status.
 */
static inline syssts_t port_get_irq_status(void) {
  uint32_t sts;

#if defined(__ghs__)
  sts = __GETSR();
#else
  asm volatile ("mfmsr   %[p0]" : [p0] "=r" (sts) :);
#endif

  return sts;
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

  return (bool)((sts & (1 << 15)) != 0);
}

/**
 * @brief   Determines the current execution context.
 *
 * @return              The execution context.
 * @retval false        not running in ISR mode.
 * @retval true         running in ISR mode.
 */
static inline bool port_is_isr_context(void) {
  uint32_t sprg0;

  /* The SPRG0 register is increased before entering interrupt handlers and
     decreased at the end.*/
  port_read_spr(272, sprg0);
  return (bool)(sprg0 > 0);
}

/**
 * @brief   Kernel-lock action.
 * @note    Implemented as global interrupt disable.
 */
static inline void port_lock(void) {

#if defined(__ghs__)
  __DI();
#else
  asm volatile ("wrteei  0" : : : "memory");
#endif
}

/**
 * @brief   Kernel-unlock action.
 * @note    Implemented as global interrupt enable.
 */
static inline void port_unlock(void) {

#if defined(__ghs__)
  __EI();
#else
  asm volatile("wrteei  1" : : : "memory");
#endif
}

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @note    Implementation not needed.
 */
static inline void port_lock_from_isr(void) {

}

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @note    Implementation not needed.
 */
static inline void port_unlock_from_isr(void) {

}

/**
 * @brief   Disables all the interrupt sources.
 * @note    Implemented as global interrupt disable.
 */
static inline void port_disable(void) {

#if defined(__ghs__)
  __DI();
#else
  asm volatile ("wrteei  0" : : : "memory");
#endif
}

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 * @note    Same as @p port_disable() in this port, there is no difference
 *          between the two states.
 */
static inline void port_suspend(void) {

#if defined(__ghs__)
  __DI();
#else
  asm volatile ("wrteei  0" : : : "memory");
#endif
}

/**
 * @brief   Enables all the interrupt sources.
 * @note    Implemented as global interrupt enable.
 */
static inline void port_enable(void) {

#if defined(__ghs__)
  __EI();
#else
  asm volatile ("wrteei  1" : : : "memory");
#endif
}

/**
 * @brief   Enters an architecture-dependent IRQ-waiting mode.
 * @details The function is meant to return when an interrupt becomes pending.
 *          The simplest implementation is an empty function or macro but this
 *          would not take advantage of architecture-specific power saving
 *          modes.
 * @note    Implemented as an inlined @p wait instruction.
 */
static inline void port_wait_for_interrupt(void) {

#if PPC_ENABLE_WFI_IDLE
  asm volatile ("wait" : : : "memory");
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
