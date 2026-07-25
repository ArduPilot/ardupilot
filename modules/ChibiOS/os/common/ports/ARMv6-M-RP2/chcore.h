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
 * @file    ARMv6-M-RP2/chcore.h
 * @brief   ARMv6-M-RP2 port macros and structures.
 *
 * @addtogroup ARMV6M_RP2_CORE
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
 * @name    Port Capabilities and Constants
 * @{
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
#define PORT_WORKING_AREA_ALIGN         PORT_STACK_ALIGN

/**
 * @brief   Number of cores supported.
 */
#define PORT_CORES_NUMBER               2

/**
 * @brief   Port-related fields added to the OS instance structure.
 */
#define PORT_INSTANCE_EXTRA_FIELDS
/** @} */

/**
 * @brief   IPC messages
 * @{
 */
#define PORT_FIFO_RESCHEDULE_MESSAGE    0xFFFFFFFFU
#define PORT_FIFO_PANIC_MESSAGE         0xFFFFFFFEU
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
#define CORTEX_MINIMUM_PRIORITY         (CORTEX_PRIORITY_LEVELS - 1U)

/**
 * @brief   Maximum priority level.
 * @details The maximum allowed priority level is always zero.
 */
#define CORTEX_MAXIMUM_PRIORITY         0U

/**
 * @brief   PendSV priority level.
 * @note    This priority is enforced to be equal to @p 0,
 *          this handler always has the highest priority that cannot preempt
 *          the kernel.
 */
#define CORTEX_PRIORITY_PENDSV          0U

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
#if !defined(PORT_IDLE_THREAD_STACK_SIZE)
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
#if !defined(PORT_INT_REQUIRED_STACK)
#define PORT_INT_REQUIRED_STACK         64
#endif

/**
 * @brief   Enables the use of the WFI instruction in the idle thread loop.
 */
#if !defined(CORTEX_ENABLE_WFI_IDLE)
#define CORTEX_ENABLE_WFI_IDLE          FALSE
#endif

/**
 * @brief   Alternate preemption method.
 * @details Activating this option will make the Kernel use the PendSV
 *          handler for preemption instead of the NMI handler.
 */
#if !defined(CORTEX_ALTERNATE_SWITCH)
#define CORTEX_ALTERNATE_SWITCH         FALSE
#endif

/**
 * @brief   Spinlock to be used by the port layer.
 */
#if !defined(PORT_SPINLOCK_NUMBER)
#define PORT_SPINLOCK_NUMBER            31
#endif

/**
 * @brief   Preferential BSS section for core zero.
 */
#if !defined(PORT_CORE0_BSS_SECTION)
#define PORT_CORE0_BSS_SECTION          CC_SECTION(".ram4_clear.core0")
#endif

/**
 * @brief   Preferential BSS section for core one.
 */
#if !defined(PORT_CORE1_BSS_SECTION)
#define PORT_CORE1_BSS_SECTION          CC_SECTION(".ram5_clear.core1")
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(CH_CUSTOMER_LIC_PORT_CM0)
  #error "CH_CUSTOMER_LIC_PORT_CM0 not defined"
#endif

#if CH_CUSTOMER_LIC_PORT_CM0 == FALSE
  #error "ChibiOS Cortex-M0 port not licensed"
#endif

#if (PORT_SPINLOCK_NUMBER < 0) || (PORT_SPINLOCK_NUMBER > 31)
  #error "invalid PORT_SPINLOCK_NUMBER value"
#endif

/* Handling a GCC problem impacting ARMv6-M.*/
#if defined(__GNUC__) && !defined(PORT_IGNORE_GCC_VERSION_CHECK)
  #if ( __GNUC__ > 5 ) && ( __GNUC__ < 10 )
    #define GCC_VERSION ( __GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ )
    #if ( __GNUC__ == 7 ) && ( GCC_VERSION >= 70500 )
    #elif ( __GNUC__ == 8 ) && ( GCC_VERSION >= 80400 )
    #elif ( __GNUC__ == 9 ) && ( GCC_VERSION >= 90300 )
    #else
      #warning "This compiler has a know problem with Cortex-M0, see GCC bugs: 88167, 88656."
    #endif
  #endif
#endif

/**
 * @name    Architecture
 * @{
 */
/**
 * @brief   Macro defining a generic ARM architecture.
 */
#define PORT_ARCHITECTURE_ARM

/**
 * @brief   Macro defining the specific ARM architecture.
 */
#define PORT_ARCHITECTURE_ARM_V6M

/**
 * @brief   Name of the implemented architecture.
 */
#define PORT_ARCHITECTURE_NAME          "ARMv6-M"

/**
 * @brief   Name of the architecture variant.
 */
#define PORT_CORE_VARIANT_NAME          "Cortex-M0+"

/**
 * @brief   Port-specific information string.
 */
#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
  #if (CORTEX_ALTERNATE_SWITCH == FALSE) || defined(__DOXYGEN__)
    #define PORT_INFO                   "Preemption through NMI (SMP)"
  #else
    #define PORT_INFO                   "Preemption through PendSV (SMP)"
  #endif
#else
  #if (CORTEX_ALTERNATE_SWITCH == FALSE) || defined(__DOXYGEN__)
    #define PORT_INFO                     "Preemption through NMI"
  #else
    #define PORT_INFO                     "Preemption through PendSV"
  #endif
#endif
/** @} */

/**
 * @brief   Maximum usable priority for normal ISRs.
 */
#if (CORTEX_ALTERNATE_SWITCH == TRUE) || defined(__DOXYGEN__)
  #define CORTEX_MAX_KERNEL_PRIORITY    1
#else
  #define CORTEX_MAX_KERNEL_PRIORITY    0
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
  uint32_t      r0;
  uint32_t      r1;
  uint32_t      r2;
  uint32_t      r3;
  uint32_t      r12;
  uint32_t      lr_thd;
  uint32_t      pc;
  uint32_t      xpsr;
};

/**
 * @brief   System saved context.
 * @details This structure represents the inner stack frame during a context
 *          switch.
 */
struct port_intctx {
  uint32_t      r8;
  uint32_t      r9;
  uint32_t      r10;
  uint32_t      r11;
  uint32_t      r4;
  uint32_t      r5;
  uint32_t      r6;
  uint32_t      r7;
  uint32_t      lr;
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
 * @brief   Platform dependent part of the @p chThdCreateI() API.
 * @details This code usually setup the context switching frame represented
 *          by an @p port_intctx structure.
 */
#define PORT_SETUP_CONTEXT(tp, wbase, wtop, pf, arg) do {                   \
  (tp)->ctx.sp = (struct port_intctx *)((uint8_t *)(wtop) -                 \
                                        sizeof (struct port_intctx));       \
  (tp)->ctx.sp->r4 = (uint32_t)(pf);                                        \
  (tp)->ctx.sp->r5 = (uint32_t)(arg);                                       \
  (tp)->ctx.sp->lr = (uint32_t)__port_thread_start;                         \
} while (false)

/**
 * @brief   Computes the thread working area global size.
 * @note    There is no need to perform alignments in this macro.
 */
#define PORT_WA_SIZE(n) (sizeof (struct port_intctx) +                      \
                         sizeof (struct port_extctx) +                      \
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
#if defined(__GNUC__) || defined(__DOXYGEN__)
  #define PORT_IRQ_PROLOGUE()                                               \
    uint32_t _saved_lr = (uint32_t)__builtin_return_address(0)
#elif defined(__ICCARM__)
  #define PORT_IRQ_PROLOGUE()                                               \
    uint32_t _saved_lr = (uint32_t)__get_LR()
#elif defined(__CC_ARM)
  #define PORT_IRQ_PROLOGUE()                                               \
    uint32_t _saved_lr = (uint32_t)__return_address()
#endif

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE() __port_irq_epilogue(_saved_lr)

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

/**
 * @brief   Panic notification.
 * @note    It is sent without polling for FIFO space because the other side
 *          could be unable to empty the FIFO after a catastrophic error.
 */
#define PORT_SYSTEM_HALT_HOOK() do {                                        \
    SIO->FIFO_WR = PORT_FIFO_PANIC_MESSAGE;                                 \
  } while (false)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void port_init(os_instance_t *oip);
  void __port_irq_epilogue(uint32_t lr);
  void __port_switch(thread_t *ntp, thread_t *otp);
  void __port_thread_start(void);
  void __port_switch_from_isr(void);
  void __port_exit_from_isr(void);
#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
  void __port_spinlock_take(void);
  void __port_spinlock_release_inline(void);
#endif /* CH_CFG_SMP_MODE == TRUE */
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Triggers an inter-core notification.
 *
 * @param[in] oip       pointer to the @p os_instance_t structure
 */
__STATIC_INLINE void port_notify_instance(os_instance_t *oip) {

  (void)oip;

  /* Sending a reschedule order to the other core if there is space in
     the FIFO.*/
  if ((SIO->FIFO_ST & SIO_FIFO_ST_RDY) != 0U) {
    SIO->FIFO_WR = PORT_FIFO_RESCHEDULE_MESSAGE;
  }
}

/**
 * @brief   Takes the kernel spinlock.
 */
__STATIC_INLINE void port_spinlock_take(void) {

  while (SIO->SPINLOCK[PORT_SPINLOCK_NUMBER] == 0U) {
  }
  __DMB();
}

/**
 * @brief   Releases the kernel spinlock.
 */
__STATIC_INLINE void port_spinlock_release(void) {

  __DMB();
  SIO->SPINLOCK[PORT_SPINLOCK_NUMBER] = (uint32_t)SIO;
}
#endif /* CH_CFG_SMP_MODE == TRUE */

/**
 * @brief   Returns a word encoding the current interrupts status.
 *
 * @return              The interrupts status.
 */
 __STATIC_INLINE syssts_t __port_get_irq_status(void) {

  return (syssts_t)__get_PRIMASK();
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
__STATIC_INLINE bool __port_irq_enabled(syssts_t sts) {

  return (sts & (syssts_t)1) == (syssts_t)0;
}

/**
 * @brief   Determines the current execution context.
 *
 * @return              The execution context.
 * @retval false        not running in ISR mode.
 * @retval true         running in ISR mode.
 */
__STATIC_INLINE bool port_is_isr_context(void) {

  return (bool)((__get_IPSR() & 0x1FFU) != 0U);
}

#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Kernel-lock action.
 * @details In this port this function disables interrupts globally.
 */
__STATIC_INLINE void port_lock(void) {

  __disable_irq();
  port_spinlock_take();
}

/**
 * @brief   Kernel-unlock action.
 * @details In this port this function enables interrupts globally.
 */
__STATIC_INLINE void port_unlock(void) {

  port_spinlock_release();
  __enable_irq();
}
#else /* CH_CFG_SMP_MODE == FALSE */
__STATIC_INLINE void port_lock(void) {

  __disable_irq();
}

__STATIC_INLINE void port_unlock(void) {

  __enable_irq();
}
#endif /* CH_CFG_SMP_MODE == FALSE */

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @details In this port this function disables interrupts globally.
 * @note    Same as @p port_lock() in this port.
 */
__STATIC_INLINE void port_lock_from_isr(void) {

  port_lock();
}

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @details In this port this function enables interrupts globally.
 * @note    Same as @p port_lock() in this port.
 */
__STATIC_INLINE void port_unlock_from_isr(void) {

  port_unlock();
}

/**
 * @brief   Disables all the interrupt sources.
 */
__STATIC_INLINE void port_disable(void) {

  __disable_irq();
}

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 */
__STATIC_INLINE void port_suspend(void) {

  __disable_irq();
}

/**
 * @brief   Enables all the interrupt sources.
 */
__STATIC_INLINE void port_enable(void) {

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
__STATIC_INLINE void port_wait_for_interrupt(void) {

#if CORTEX_ENABLE_WFI_IDLE == TRUE
  __WFI();
#endif
}

/**
 * @brief   Returns the current value of the realtime counter.
 * @note    In this port the RT counter is the same counter used for
 *          system time in tick-less mode, resolution is always 1uS.
 *
 * @return              The realtime counter value.
 */
__STATIC_FORCEINLINE rtcnt_t port_rt_get_counter_value(void) {

  return TIMER->TIMERAWL;
}

/**
 * @brief   Returns a core index.
 * @return              The core identifier from 0 to @p PORT_CORES_NUMBER - 1.
 */
__STATIC_INLINE core_id_t port_get_core_id(void) {

  return SIO->CPUID;
}

#endif /* !defined(_FROM_ASM_) */

/*===========================================================================*/
/* Module late inclusions.                                                   */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

#if CH_CFG_ST_TIMEDELTA > 0
#include "chcore_timer_rp2.h"
#endif /* CH_CFG_ST_TIMEDELTA > 0 */

#endif /* !defined(_FROM_ASM_) */

#endif /* CHCORE_H */

/** @} */
