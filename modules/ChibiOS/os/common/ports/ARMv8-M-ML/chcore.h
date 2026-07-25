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
 * @file    ARMv8-M-ML/chcore.h
 * @brief   ARMv8-M MainLine port macros and structures.
 *
 * @addtogroup ARMV8M_ML_CORE
 * @{
 */

#ifndef CHCORE_H
#define CHCORE_H

/* Inclusion of the Cortex-Mx implementation specific parameters.*/
#include "cmparams.h"

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

#elif defined(__ICCARM__)
#define PORT_COMPILER_NAME              "IAR"

#elif defined(__CC_ARM)
#define PORT_COMPILER_NAME              "RVCT"

#else
#error "unsupported compiler"
#endif

#endif /* !defined(_FROM_ASM_) */
/** @} */

/**
 * @name    Priority Ranges
 * @{
 */
/**
 * @brief   Disabled value for BASEPRI register.
 */
#define CORTEX_BASEPRI_DISABLED         0U

/**
 * @brief   Total priority levels.
 */
#define CORTEX_PRIORITY_LEVELS          (1U << CORTEX_PRIORITY_BITS)

/**
 * @brief   Minimum priority level.
 * @details This minimum priority level is calculated from the number of
 *          priority bits supported by the specific Cortex-Mx implementation.
 */
#define CORTEX_MINIMUM_PRIORITY         (CORTEX_PRIORITY_LEVELS - 1)

/**
 * @brief   Maximum priority level.
 * @details The maximum allowed priority level is always zero.
 */
#define CORTEX_MAXIMUM_PRIORITY         0U

/**
 * @brief   PendSV priority level.
 * @note    This priority is enforced to be equal to
 *          @p CORTEX_MAX_KERNEL_PRIORITY, this handler always have the
 *          highest priority that cannot preempt the kernel.
 */
#define CORTEX_PRIORITY_PENDSV          CORTEX_MAX_KERNEL_PRIORITY

/**
 * @brief   Priority level to priority mask conversion macro.
 */
#define CORTEX_PRIO_MASK(n)                                                 \
  ((n) << (8U - (unsigned)CORTEX_PRIORITY_BITS))
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Stack size for the system idle thread.
 * @details This size depends on the idle thread implementation, usually
 *          the idle thread should take no more space than those reserved
 *          by @p PORT_INT_REQUIRED_STACK.
 * @note    In this port it is set to 16 because the idle thread does have
 *          a stack frame when compiling without optimizations. You may
 *          reduce this value to zero when compiling with optimizations.
 */
#if !defined(PORT_IDLE_THREAD_STACK_SIZE) || defined(__DOXYGEN__)
#define PORT_IDLE_THREAD_STACK_SIZE     16
#endif

/**
 * @brief   Per-thread stack overhead for interrupts servicing.
 * @details This constant is used in the calculation of the correct working
 *          area size.
 * @note    In this port this value is conservatively set to 64 because the
 *          function @p chSchDoPreemption() can have a stack frame, especially
 *          with compiler optimizations disabled. The value can be reduced
 *          when compiler optimizations are enabled.
 */
#if !defined(PORT_INT_REQUIRED_STACK) || defined(__DOXYGEN__)
#define PORT_INT_REQUIRED_STACK         64
#endif

/**
 * @brief   Enables the use of the WFI instruction in the idle thread loop.
 */
#if !defined(CORTEX_ENABLE_WFI_IDLE)
#define CORTEX_ENABLE_WFI_IDLE          FALSE
#endif

/**
 * @brief   FPU support in context switch.
 * @details Activating this option activates the FPU support in the kernel.
 */
#if !defined(CORTEX_USE_FPU)
#define CORTEX_USE_FPU                  CORTEX_HAS_FPU
#elif (CORTEX_USE_FPU == TRUE) && (CORTEX_HAS_FPU == FALSE)
/* This setting requires an FPU presence check in case it is externally
   redefined.*/
#error "the selected core does not have an FPU"
#endif

/**
 * @brief   Simplified priority handling flag.
 * @details Activating this option makes the Kernel work in compact mode.
 *          In compact mode interrupts are disabled globally instead of
 *          raising the priority mask to some intermediate level.
 */
#if !defined(CORTEX_SIMPLIFIED_PRIORITY)
#define CORTEX_SIMPLIFIED_PRIORITY      FALSE
#endif

/**
 * @brief   SVCALL handler priority.
 * @note    The default SVCALL handler priority is defaulted to
 *          @p CORTEX_MAXIMUM_PRIORITY+1, this reserves the
 *          @p CORTEX_MAXIMUM_PRIORITY priority level as fast interrupts
 *          priority level.
 */
#if !defined(CORTEX_PRIORITY_SVCALL)
#define CORTEX_PRIORITY_SVCALL          (CORTEX_MAXIMUM_PRIORITY + 1U)
#elif !PORT_IRQ_IS_VALID_PRIORITY(CORTEX_PRIORITY_SVCALL)
/* If it is externally redefined then better perform a validity check on it.*/
#error "invalid priority level specified for CORTEX_PRIORITY_SVCALL"
#endif

/**
 * @brief   NVIC PRIGROUP initialization expression.
 * @details The default assigns all available priority bits as preemption
 *          priority with no sub-priority.
 */
#if !defined(CORTEX_PRIGROUP_INIT) || defined(__DOXYGEN__)
#define CORTEX_PRIGROUP_INIT            (7 - CORTEX_PRIORITY_BITS)
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

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
#define PORT_STACK_ALIGN                32U

/**
 * @brief   Working Areas alignment constant.
 * @note    It is the alignment to be enforced for thread working areas.
 */
#define PORT_WORKING_AREA_ALIGN         32U
/** @} */

/**
 * @name    Architecture
 * @{
 */
/**
 * @brief   Macro defining the specific ARM architecture.
 */
#define PORT_ARCHITECTURE_ARM_V8M_MAINLINE

/**
 * @brief   Name of the implemented architecture.
 */
#define PORT_ARCHITECTURE_NAME          "ARMv8-M Mainline"

/**
 * @brief   Macro defining a generic ARM architecture.
 */
#define PORT_ARCHITECTURE_ARM

#if (CORTEX_MODEL == 33) || defined(__DOXYGEN__)

  #if !defined(CH_CUSTOMER_LIC_PORT_CM33)
    #error "CH_CUSTOMER_LIC_PORT_CM33 not defined"
  #endif

  #if CH_CUSTOMER_LIC_PORT_CM33 == FALSE
    #error "ChibiOS Cortex-M33 port not licensed"
  #endif

  /**
   * @brief   Name of the architecture variant.
   */
  #define PORT_CORE_VARIANT_NAME        "Cortex-M33"


#else
  #error "unknown ARMv8-M variant"
#endif

/**
 * @brief   Port-specific information string.
 */
#if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) || defined(__DOXYGEN__)
  #define PORT_INFO                     "Advanced kernel mode"
#else
  #define PORT_INFO                     "Compact kernel mode"
#endif
/** @} */

#if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) || defined(__DOXYGEN__)
  /**
   * @brief   Maximum usable priority for normal ISRs.
   */
  #define CORTEX_MAX_KERNEL_PRIORITY    (CORTEX_PRIORITY_SVCALL + 1U)

  /**
   * @brief   BASEPRI level within kernel lock.
   */
  #define CORTEX_BASEPRI_KERNEL                                             \
    CORTEX_PRIO_MASK(CORTEX_MAX_KERNEL_PRIORITY)

#else
  #define CORTEX_MAX_KERNEL_PRIORITY    0U
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)

/**
 * @brief   Interrupt saved context.
 * @details This structure represents the stack frame saved during a
 *          preemption-capable interrupt handler.
 * @note    It is implemented to match the Cortex-Mx exception context.
 */
struct port_extctx {
  uint32_t              r0;
  uint32_t              r1;
  uint32_t              r2;
  uint32_t              r3;
  uint32_t              r12;
  uint32_t              lr_thd;
  uint32_t              pc;
  uint32_t              xpsr;
#if (CORTEX_USE_FPU == TRUE) || defined(__DOXYGEN__)
  uint32_t              s0;
  uint32_t              s1;
  uint32_t              s2;
  uint32_t              s3;
  uint32_t              s4;
  uint32_t              s5;
  uint32_t              s6;
  uint32_t              s7;
  uint32_t              s8;
  uint32_t              s9;
  uint32_t              s10;
  uint32_t              s11;
  uint32_t              s12;
  uint32_t              s13;
  uint32_t              s14;
  uint32_t              s15;
  uint32_t              fpscr;
  uint32_t              reserved;
#endif /* CORTEX_USE_FPU == TRUE */
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switch.
 */
struct port_intctx {
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || defined(__DOXYGEN__)
  uint32_t              splim;
#endif
#if (CORTEX_USE_FPU == TRUE) || defined(__DOXYGEN__)
  uint32_t              s16;
  uint32_t              s17;
  uint32_t              s18;
  uint32_t              s19;
  uint32_t              s20;
  uint32_t              s21;
  uint32_t              s22;
  uint32_t              s23;
  uint32_t              s24;
  uint32_t              s25;
  uint32_t              s26;
  uint32_t              s27;
  uint32_t              s28;
  uint32_t              s29;
  uint32_t              s30;
  uint32_t              s31;
#endif /* CORTEX_USE_FPU == TRUE */
  uint32_t              r4;
  uint32_t              r5;
  uint32_t              r6;
  uint32_t              r7;
  uint32_t              r8;
  uint32_t              r9;
  uint32_t              r10;
  uint32_t              r11;
  uint32_t              lr;
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

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_PRIORITY(n)                                       \
  (((n) >= 0U) && ((n) < CORTEX_PRIORITY_LEVELS))

/**
 * @brief   Priority level verification macro.
 */
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n)                                \
  (((n) >= CORTEX_MAX_KERNEL_PRIORITY) && ((n) < CORTEX_PRIORITY_LEVELS))

/**
 * @brief   Optimized thread function declaration macro.
 */
#define PORT_THD_FUNCTION(tname, arg) void tname(void *arg)

/**
 * @brief   Initialization of stack check part of thread context.
 */
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || defined(__DOXYGEN__)
  #define PORT_SETUP_CONTEXT_SPLIM(tp, wbase)                               \
    (tp)->ctx.sp->splim = (uint32_t)(wbase)
#else
  #define PORT_SETUP_CONTEXT_SPLIM(tp, wbase)
#endif

/**
 * @brief   Initialization of FPU part of thread context.
 */
#if (CORTEX_USE_FPU == TRUE) || defined(__DOXYGEN__)
  #define PORT_SETUP_CONTEXT_FPU(tp)
#else
  #define PORT_SETUP_CONTEXT_FPU(tp)
#endif

/**
 * @brief   Initialization of MPU part of thread context.
 */
#define PORT_SETUP_CONTEXT_MPU(tp)

/**
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p port_intctx structure.
 */
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) do {                   \
  (tp)->ctx.sp = (struct port_intctx *)(void *)                             \
                   ((uint8_t *)(wtop) - sizeof (struct port_intctx));       \
  (tp)->ctx.sp->r4 = (uint32_t)(pf);                                        \
  (tp)->ctx.sp->r5 = (uint32_t)(arg);                                       \
  (tp)->ctx.sp->lr = (uint32_t)__port_thread_start;                         \
  PORT_SETUP_CONTEXT_SPLIM(tp, wbase);                                      \
  PORT_SETUP_CONTEXT_FPU(tp);                                               \
  PORT_SETUP_CONTEXT_MPU(tp);                                               \
} while (false)

/**
 * @brief   Computes the thread working area global size.
 * @note    There is no need to perform alignments in this macro.
 */
#define PORT_WA_SIZE(n) (sizeof (struct port_intctx) +                      \
                         sizeof (struct port_extctx) +                      \
                         (size_t)(n) +                                      \
                         (size_t)PORT_INT_REQUIRED_STACK)

/**
 * @brief   Static working area allocation.
 * @details This macro is used to allocate a static thread working area
 *          aligned as both position and size.
 *
 * @param[in] s         the name to be assigned to the stack array
 * @param[in] n         the stack size to be assigned to the thread
 */
#define PORT_WORKING_AREA(s, n)                                             \
  ALIGNED_VAR(32) stkalign_t s[THD_WORKING_AREA_SIZE(n) / sizeof (stkalign_t)]

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
#define PORT_IRQ_EPILOGUE() __port_irq_epilogue()

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
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#if (CH_DBG_ENABLE_STACK_CHECK == FALSE) || defined(__DOXYGEN__)
  #define port_switch(ntp, otp) __port_switch(ntp, otp)

#else
  #define port_switch(ntp, otp) do {                                        \
    struct port_intctx *r13 = (struct port_intctx *)__get_PSP();            \
    if ((stkalign_t *)(void *)(r13 - 1) < (otp)->wabase) {                  \
      chSysHalt("stack overflow");                                          \
    }                                                                       \
    __port_switch(ntp, otp);                                                \
  } while (false)
#endif

/**
 * @brief   Returns a word representing a critical section status.
 *
 * @return              The critical section status.
 */
#define port_get_lock_status() __port_get_irq_status()

/**
 * @brief   Determines if in a critical section.
 *
 * @param[in] sts       status word returned by @p port_get_lock_status()
 * @return              The current status.
 * @retval false        if running outside a critical section.
 * @retval true         if running within a critical section.
 */
#define port_is_locked(sts) !__port_irq_enabled(sts)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

#ifdef __cplusplus
extern "C" {
#endif
  void port_init(os_instance_t *oip);
  void __port_irq_epilogue(void);
  void __port_switch(thread_t *ntp, thread_t *otp);
  void __port_thread_start(void);
  void __port_switch_from_isr(void);
  void __port_exit_from_isr(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

 /**
  * @brief   Returns a word encoding the current interrupts status.
  *
  * @return              The interrupts status.
  */
 __STATIC_FORCEINLINE syssts_t __port_get_irq_status(void) {
   syssts_t sts;

 #if CORTEX_SIMPLIFIED_PRIORITY == FALSE
   sts = (syssts_t)__get_BASEPRI();
 #else /* CORTEX_SIMPLIFIED_PRIORITY */
   sts = (syssts_t)__get_PRIMASK();
 #endif /* CORTEX_SIMPLIFIED_PRIORITY */
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
 __STATIC_FORCEINLINE bool __port_irq_enabled(syssts_t sts) {

 #if CORTEX_SIMPLIFIED_PRIORITY == FALSE
   return sts == (syssts_t)CORTEX_BASEPRI_DISABLED;
 #else /* CORTEX_SIMPLIFIED_PRIORITY */
   return (sts & (syssts_t)1) == (syssts_t)0;
 #endif /* CORTEX_SIMPLIFIED_PRIORITY */
 }

 /**
  * @brief   Determines the current execution context.
  *
  * @return              The execution context.
  * @retval false        not running in ISR mode.
  * @retval true         running in ISR mode.
  */
 __STATIC_FORCEINLINE bool port_is_isr_context(void) {

   return (bool)((__get_IPSR() & 0x1FFU) != 0U);
 }

 /**
  * @brief   Kernel-lock action.
  * @details In this port this function raises the base priority to kernel
  *          level.
  */
 __STATIC_FORCEINLINE void port_lock(void) {

 #if CORTEX_SIMPLIFIED_PRIORITY == FALSE
 #if defined(__CM7_REV)
 #if __CM7_REV <= 1
   __disable_irq();
 #endif
 #endif
   __set_BASEPRI(CORTEX_BASEPRI_KERNEL);
 #if defined(__CM7_REV)
 #if __CM7_REV <= 1
   __enable_irq();
 #endif
 #endif
 #else /* CORTEX_SIMPLIFIED_PRIORITY */
   __disable_irq();
 #endif /* CORTEX_SIMPLIFIED_PRIORITY */
 }

 /**
  * @brief   Kernel-unlock action.
  * @details In this port this function lowers the base priority to user
  *          level.
  */
 __STATIC_FORCEINLINE void port_unlock(void) {

 #if CORTEX_SIMPLIFIED_PRIORITY == FALSE
   __set_BASEPRI(CORTEX_BASEPRI_DISABLED);
 #else /* CORTEX_SIMPLIFIED_PRIORITY */
   __enable_irq();
 #endif /* CORTEX_SIMPLIFIED_PRIORITY */
 }

 /**
  * @brief   Kernel-lock action from an interrupt handler.
  * @details In this port this function raises the base priority to kernel
  *          level.
  * @note    Same as @p port_lock() in this port.
  */
 __STATIC_FORCEINLINE void port_lock_from_isr(void) {

   port_lock();
 }

 /**
  * @brief   Kernel-unlock action from an interrupt handler.
  * @details In this port this function lowers the base priority to user
  *          level.
  * @note    Same as @p port_unlock() in this port.
  */
 __STATIC_FORCEINLINE void port_unlock_from_isr(void) {

   port_unlock();
 }

 /**
  * @brief   Disables all the interrupt sources.
  * @note    In this port it disables all the interrupt sources by raising
  *          the priority mask to level 0.
  */
 __STATIC_FORCEINLINE void port_disable(void) {

   __disable_irq();
 }

 /**
  * @brief   Disables the interrupt sources below kernel-level priority.
  * @note    Interrupt sources above kernel level remains enabled.
  * @note    In this port it raises/lowers the base priority to kernel level.
  */
 __STATIC_FORCEINLINE void port_suspend(void) {

 #if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) || defined(__DOXYGEN__)
   __set_BASEPRI(CORTEX_BASEPRI_KERNEL);
   __enable_irq();
 #else
   __disable_irq();
 #endif
 }

 /**
  * @brief   Enables all the interrupt sources.
  * @note    In this port it lowers the base priority to user level.
  */
 __STATIC_FORCEINLINE void port_enable(void) {

 #if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) || defined(__DOXYGEN__)
   __set_BASEPRI(CORTEX_BASEPRI_DISABLED);
 #endif
   __enable_irq();
 }

 /**
  * @brief   Enters an architecture-dependent IRQ-waiting mode.
  * @details The function is meant to return when an interrupt becomes pending.
  *          The simplest implementation is an empty function or macro but this
  *          would not take advantage of architecture-specific power saving
  *          modes.
  * @note    Implemented as an inlined @p WFI instruction.
  */
 __STATIC_FORCEINLINE void port_wait_for_interrupt(void) {

 #if CORTEX_ENABLE_WFI_IDLE == TRUE
   __WFI();
 #endif
 }

 /**
  * @brief   Returns the current value of the realtime counter.
  *
  * @return              The realtime counter value.
  */
 __STATIC_FORCEINLINE rtcnt_t port_rt_get_counter_value(void) {

   return DWT->CYCCNT;
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
