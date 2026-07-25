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
 * @file    ARM/chcore.h
 * @brief   ARM7/9 architecture port macros and structures.
 *
 * @addtogroup ARM_CORE
 * @{
 */

#ifndef CHCORE_H
#define CHCORE_H

/* Inclusion of the ARM implementation specific parameters.*/
#include "armparams.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module because those intrinsic macros are not necessarily defined
   by the assembler too.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Compiler name and version.
 */
#if defined(__GNUC__) || defined(__DOXYGEN__)
#define PORT_COMPILER_NAME              "GCC " __VERSION__

#else
#error "unsupported compiler"
#endif

#endif /* !defined(_FROM_ASM_) */
/** @} */

/**
 * @name    Port Capabilities and Constants
 * @{
 */
/**
 * @brief   This port supports a realtime counter.
 */
#define PORT_SUPPORTS_RT                TRUE

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
 * @name    ARM variants
 * @{
 */
#define ARM_CORE_ARM7TDMI               7
#define ARM_CORE_ARM9                   9
#define ARM_CORE_CORTEX_A5              105
#define ARM_CORE_CORTEX_A7              107
#define ARM_CORE_CORTEX_A8              108
#define ARM_CORE_CORTEX_A9              109
/** @} */

/**
 * @name    Architecture
 * @{
 */
/**
 * @brief   Macro defining a generic ARM architecture.
 */
#define PORT_ARCHITECTURE_ARM

/* ARM core check.*/
#if (ARM_CORE == ARM_CORE_ARM7TDMI) || defined(__DOXYGEN__)
  #define PORT_ARCHITECTURE_ARM_ARM7
  #define PORT_ARCHITECTURE_NAME        "ARMv4T"
  #define PORT_CORE_VARIANT_NAME        "ARM7"

#elif ARM_CORE == ARM_CORE_ARM9
  #define PORT_ARCHITECTURE_ARM_ARM9
  #define PORT_ARCHITECTURE_NAME        "ARMv5T"
  #define PORT_CORE_VARIANT_NAME        "ARM9"

#elif ARM_CORE == ARM_CORE_CORTEX_A5
  #define PORT_ARCHITECTURE_ARM_CORTEXA5
  #define PORT_ARCHITECTURE_NAME        "ARMv7"
  #define PORT_CORE_VARIANT_NAME        "ARM Cortex-A5"

#elif ARM_CORE == ARM_CORE_CORTEX_A7
  #define PORT_ARCHITECTURE_ARM_CORTEXA5
  #define PORT_ARCHITECTURE_NAME        "ARMv7"
  #define PORT_CORE_VARIANT_NAME        "ARM Cortex-A7"

#elif ARM_CORE == ARM_CORE_CORTEX_A8
  #define PORT_ARCHITECTURE_ARM_CORTEXA8
  #define PORT_ARCHITECTURE_NAME        "ARMv7"
  #define PORT_CORE_VARIANT_NAME        "ARM Cortex-A8"

#elif ARM_CORE == ARM_CORE_CORTEX_A9
  #define PORT_ARCHITECTURE_ARM_CORTEXA9
  #define PORT_ARCHITECTURE_NAME        "ARMv7"
  #define PORT_CORE_VARIANT_NAME        "ARM Cortex-A9"

#else
  #error "unknown or unsupported ARM core"
#endif

#if defined(THUMB_PRESENT)
  #if defined(THUMB_NO_INTERWORKING)
    #define PORT_INFO                   "Pure THUMB mode"
  #else
    #define PORT_INFO                   "Interworking mode"
  #endif
#else
  #define PORT_INFO                     "Pure ARM mode"
#endif

#if ARM_CORE < 100
  #define ARM_CORE_CLASSIC              1
  #define ARM_CORE_CORTEX_A             0
#elif ARM_CORE < 200
  #define ARM_CORE_CLASSIC              0
  #define ARM_CORE_CORTEX_A             1
#else
  #error "unknown or unsupported ARM core"
#endif
/** @} */

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
 */
#if !defined(PORT_INT_REQUIRED_STACK) || defined(__DOXYGEN__)
#define PORT_INT_REQUIRED_STACK         32
#endif

/**
 * @brief   If enabled allows the idle thread to enter a low power mode.
 */
#ifndef ARM_ENABLE_WFI_IDLE
#define ARM_ENABLE_WFI_IDLE             FALSE
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
 * @brief   Generic ARM register.
 */
typedef void *regarm_t;

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during an
 *          interrupt handler.
 */
struct port_extctx {
  regarm_t              spsr_irq;
  regarm_t              lr_irq;
  regarm_t              r0;
  regarm_t              r1;
  regarm_t              r2;
  regarm_t              r3;
  regarm_t              r12;
  regarm_t              lr_usr;
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switch.
 */
struct port_intctx {
  regarm_t              r4;
  regarm_t              r5;
  regarm_t              r6;
  regarm_t              r7;
  regarm_t              r8;
  regarm_t              r9;
  regarm_t              r10;
  regarm_t              r11;
  regarm_t              lr;
};

/**
 * @brief   Platform dependent part of the @p thread_t structure.
 * @details In this port the structure just holds a pointer to the
 *          @p port_intctx structure representing the stack pointer
 *          at context switch time.
 */
struct port_context {
  struct port_intctx    *sp;
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Priority level verification macro.
 * @note    Not applicable in this architecture.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n) false

/**
 * @brief   Priority level verification macro.
 * @note    Not applicable in this architecture.
 */
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n) false

/**
 * @brief   Optimized thread function declaration macro.
 */
#define PORT_THD_FUNCTION(tname, arg) void tname(void *arg)

/**
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p port_intctx structure.
 */
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) {                      \
  (tp)->ctx.sp = (struct port_intctx *)((uint8_t *)(wtop) -                 \
                                        sizeof (struct port_intctx));       \
  (tp)->ctx.sp->r4 = (regarm_t)(pf);                                        \
  (tp)->ctx.sp->r5 = (regarm_t)(arg);                                       \
  (tp)->ctx.sp->lr = (regarm_t)(__port_thread_start);                       \
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
 * @brief   Priority level verification macro.
 * @todo    Add the required parameters to armparams.h.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n) false

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
#define PORT_IRQ_EPILOGUE() return chSchIsPreemptionRequired()

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#ifdef __cplusplus
#define PORT_IRQ_HANDLER(id) extern "C" bool id(void)
#else
#define PORT_IRQ_HANDLER(id) bool id(void)
#endif

/**
 * @brief   Fast IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#define PORT_FAST_IRQ_HANDLER(id)                                           \
  __attribute__((interrupt("FIQ"))) void id(void)

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 * @note    Implemented as inlined code for performance reasons.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#if defined(THUMB)

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
#define port_switch(ntp, otp) {                                             \
  register struct port_intctx *r13 asm ("r13");                             \
  if ((stkalign_t *)(r13 - 1) < otp->wabase)                                \
    CH_CFG_STACK_OVERFLOW_HOOK(otp);                                        \
  __port_switch_thumb(ntp, otp);                                             \
}
#else
#define port_switch(ntp, otp) __port_switch_thumb(ntp, otp)
#endif

#else /* !defined(THUMB) */

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
#define port_switch(ntp, otp) {                                             \
  register struct port_intctx *r13 asm ("r13");                             \
  if ((stkalign_t *)(r13 - 1) < otp->wabase)                                \
  CH_CFG_STACK_OVERFLOW_HOOK(otp);                                          \
  __port_switch_arm(ntp, otp);                                               \
}
#else
#define port_switch(ntp, otp) __port_switch_arm(ntp, otp)
#endif

#endif /* !defined(THUMB) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#if defined(THUMB_PRESENT)
  syssts_t __port_get_cpsr(void);
#endif
#if defined(THUMB)
  void __port_switch_thumb(thread_t *ntp, thread_t *otp);
#else
  void __port_switch_arm(thread_t *ntp, thread_t *otp);
#endif
  void __port_thread_start(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Port-related initialization code.
 */
static inline void port_init(os_instance_t *oip) {

  (void)oip;
}

/**
 * @brief   Returns a word encoding the current interrupts status.
 *
 * @return              The interrupts status.
 */
static inline syssts_t port_get_irq_status(void) {
  syssts_t sts;

#if defined(THUMB)
  sts = __port_get_cpsr();
#else
  __asm volatile ("mrs     %[p0], CPSR" : [p0] "=r" (sts) :);
#endif
  /*lint -save -e530 [9.1] Asm instruction not seen by lint.*/
  return sts;
  /*lint -restore*/
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

  return (sts & (syssts_t)0x80) == (syssts_t)0;
}

/**
 * @brief   Determines the current execution context.
 *
 * @return              The execution context.
 * @retval false        not running in ISR mode.
 * @retval true         running in ISR mode.
 */
static inline bool port_is_isr_context(void) {
  syssts_t sts;

#if defined(THUMB)
  sts = __port_get_cpsr();
#else
  __asm volatile ("mrs     %[p0], CPSR" : [p0] "=r" (sts) :);
#endif

  /*lint -save -e530 [9.1] Asm instruction not seen by lint.*/
  return (sts & (syssts_t)0x1F) == (syssts_t)0x12;
  /*lint -restore*/
}

/**
 * @brief   Kernel-lock action.
 * @details In this port it disables the IRQ sources and keeps FIQ sources
 *          enabled.
 */
static inline void port_lock(void) {

#if defined(THUMB)
  __asm volatile ("bl      __port_lock_thumb" : : : "r3", "lr", "memory");
#else
  __asm volatile ("msr     CPSR_c, #0x9F" : : : "memory");
#endif
}

/**
 * @brief   Kernel-unlock action.
 * @details In this port it enables both the IRQ and FIQ sources.
 */
static inline void port_unlock(void) {

#if defined(THUMB)
  __asm volatile ("bl      __port_unlock_thumb" : : : "r3", "lr", "memory");
#else
  __asm volatile ("msr     CPSR_c, #0x1F" : : : "memory");
#endif
}

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @note    Empty in this port.
 */
static inline void port_lock_from_isr(void) {

}

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @note    Empty in this port.
 */
static inline void port_unlock_from_isr(void) {

}

/**
 * @brief   Disables all the interrupt sources.
 * @details In this port it disables both the IRQ and FIQ sources.
 * @note    Implements a workaround for spurious interrupts taken from the NXP
 *          LPC214x datasheet.
 */
static inline void port_disable(void) {

#if defined(THUMB)
  __asm volatile ("bl      __port_disable_thumb" : : : "r3", "lr", "memory");
#else
  __asm volatile ("mrs     r3, CPSR                       \n\t"
                  "orr     r3, #0x80                      \n\t"
                  "msr     CPSR_c, r3                     \n\t"
                  "orr     r3, #0x40                      \n\t"
                  "msr     CPSR_c, r3" : : : "r3", "memory");
#endif
}

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 * @note    Interrupt sources above kernel level remains enabled.
 * @note    In this port it disables the IRQ sources and enables the
 *          FIQ sources.
 */
static inline void port_suspend(void) {

#if defined(THUMB)
  __asm volatile ("bl      __port_suspend_thumb" : : : "r3", "lr", "memory");
#else
  __asm volatile ("msr     CPSR_c, #0x9F" : : : "memory");
#endif
}

/**
 * @brief   Enables all the interrupt sources.
 * @note    In this port it enables both the IRQ and FIQ sources.
 */
static inline void port_enable(void) {

#if defined(THUMB)
  __asm volatile ("bl      __port_enable_thumb" : : : "r3", "lr", "memory");
#else
  __asm volatile ("msr     CPSR_c, #0x1F" : : : "memory");
#endif
}

/**
 * @brief   Returns the current value of the realtime counter.
 *
 * @return              The realtime counter value.
 */
static inline rtcnt_t port_rt_get_counter_value(void) {

#if ARM_CORE_CORTEX_A
  rtcnt_t cyc;

  __asm volatile("mrc p15, 0, %[p0], c9, c13, 0" : [p0] "=r" (cyc) :);

  return cyc;
#else
  return 0;
#endif
}

/**
 * @brief   Enters an architecture-dependent IRQ-waiting mode.
 * @details The function is meant to return when an interrupt becomes pending.
 *          The simplest implementation is an empty function or macro but this
 *          would not take advantage of architecture-specific power saving
 *          modes.
 * @note    Implemented as an inlined @p WFI instruction.
 */
static inline void port_wait_for_interrupt(void) {

#if ARM_ENABLE_WFI_IDLE == TRUE
  ARM_WFI_IMPL;
#endif
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
