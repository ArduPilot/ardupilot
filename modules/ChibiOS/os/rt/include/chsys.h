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
 * @file    rt/include/chsys.h
 * @brief   System related macros and structures.
 *
 * @addtogroup system
 * @{
 */

#ifndef CHSYS_H
#define CHSYS_H

/*lint -sem(chSysHalt, r_no)*/

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Masks of executable integrity checks.
 * @{
 */
#define CH_INTEGRITY_RLIST                  1U
#define CH_INTEGRITY_VTLIST                 2U
#define CH_INTEGRITY_REGISTRY               4U
#define CH_INTEGRITY_PORT                   8U
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   Core zero memory affinity macro.
 * @note    The memory is meant to be reachable by both cores but
 *          preferred by core zero.
 * @note    Only uninitialized variables can be tagged with this attribute.
 */
#if defined(PORT_CORE0_BSS_SECTION) || defined(__DOXYGEN__)
#define CH_SYS_CORE0_MEMORY             PORT_CORE0_BSS_SECTION
#else
#define CH_SYS_CORE0_MEMORY             /* Default.*/
#endif

/**
 * @brief   Core one memory affinity macro.
 * @note    The memory is meant to be reachable by both cores but
 *          preferred by core one.
 * @note    Only uninitialized variables can be tagged with this attribute.
 */
#if defined(PORT_CORE1_BSS_SECTION) || defined(__DOXYGEN__)
#define CH_SYS_CORE1_MEMORY             PORT_CORE1_BSS_SECTION
#else
#define CH_SYS_CORE1_MEMORY             /* Default.*/
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Access to current core's instance structure.
 */
#if defined(PORT_INSTANCE_ACCESS)
  #define currcore                      PORT_INSTANCE_ACCESS
#else /* !defined(PORT_INSTANCE_ACCESS) */
  #if (PORT_CORES_NUMBER > 1) || defined(__DOXYGEN__)
    #define currcore                    ch_system.instances[port_get_core_id()]
  #else
    #define currcore                    (&ch0)
  #endif
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @name    ISRs abstraction macros
 * @{
 */
/**
 * @brief   Priority level validation macro.
 * @details This macro determines if the passed value is a valid priority
 *          level for the underlying architecture.
 *
 * @param[in] prio      the priority level
 * @return              Priority range result.
 * @retval false        if the priority is invalid or if the architecture
 *                      does not support priorities.
 * @retval true         if the priority is valid.
 */
#if defined(PORT_IRQ_IS_VALID_PRIORITY) || defined(__DOXYGEN__)
#define CH_IRQ_IS_VALID_PRIORITY(prio)                                      \
  PORT_IRQ_IS_VALID_PRIORITY(prio)
#else
#define CH_IRQ_IS_VALID_PRIORITY(prio) false
#endif

/**
 * @brief   Priority level validation macro.
 * @details This macro determines if the passed value is a valid priority
 *          level that cannot preempt the kernel critical zone.
 *
 * @param[in] prio      the priority level
 * @return              Priority range result.
 * @retval false        if the priority is invalid or if the architecture
 *                      does not support priorities.
 * @retval true         if the priority is valid.
 */
#if defined(PORT_IRQ_IS_VALID_KERNEL_PRIORITY) || defined(__DOXYGEN__)
#define CH_IRQ_IS_VALID_KERNEL_PRIORITY(prio)                               \
  PORT_IRQ_IS_VALID_KERNEL_PRIORITY(prio)
#else
#define CH_IRQ_IS_VALID_KERNEL_PRIORITY(prio) false
#endif

/**
 * @brief   IRQ handler enter code.
 * @note    Usually IRQ handlers functions are also declared naked.
 * @note    On some architectures this macro can be empty.
 *
 * @special
 */
#define CH_IRQ_PROLOGUE()                                                   \
  PORT_IRQ_PROLOGUE();                                                      \
  CH_CFG_IRQ_PROLOGUE_HOOK();                                               \
  __stats_increase_irq();                                                   \
  __trace_isr_enter(__func__);                                              \
  __dbg_check_enter_isr()

/**
 * @brief   IRQ handler exit code.
 * @note    Usually IRQ handlers function are also declared naked.
 * @note    This macro usually performs the final reschedule by using
 *          @p chSchIsPreemptionRequired() and @p chSchDoReschedule().
 *
 * @special
 */
#define CH_IRQ_EPILOGUE()                                                   \
  __dbg_check_leave_isr();                                                  \
  __trace_isr_leave(__func__);                                              \
  CH_CFG_IRQ_EPILOGUE_HOOK();                                               \
  PORT_IRQ_EPILOGUE()

/**
 * @brief   Standard normal IRQ handler declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 *
 * @special
 */
#define CH_IRQ_HANDLER(id) PORT_IRQ_HANDLER(id)
/** @} */

/**
 * @name    Fast ISRs abstraction macros
 * @{
 */
/**
 * @brief   Standard fast IRQ handler declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 * @note    Not all architectures support fast interrupts.
 *
 * @special
 */
#define CH_FAST_IRQ_HANDLER(id) PORT_FAST_IRQ_HANDLER(id)
/** @} */

/**
 * @name    Time conversion utilities for the realtime counter
 * @{
 */
/**
 * @brief   Seconds to realtime counter.
 * @details Converts from seconds to realtime counter cycles.
 * @note    The macro assumes that @p freq >= @p 1.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] sec       number of seconds
 * @return              The number of cycles.
 *
 * @api
 */
#define S2RTC(freq, sec) ((freq) * (sec))

/**
 * @brief   Milliseconds to realtime counter.
 * @details Converts from milliseconds to realtime counter cycles.
 * @note    The result is rounded upward to the next millisecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] msec      number of milliseconds
 * @return              The number of cycles.
 *
 * @api
 */
#define MS2RTC(freq, msec) (rtcnt_t)((((freq) + 999UL) / 1000UL) * (msec))

/**
 * @brief   Microseconds to realtime counter.
 * @details Converts from microseconds to realtime counter cycles.
 * @note    The result is rounded upward to the next microsecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] usec      number of microseconds
 * @return              The number of cycles.
 *
 * @api
 */
#define US2RTC(freq, usec) (rtcnt_t)((((freq) + 999999UL) / 1000000UL) * (usec))

/**
 * @brief   Realtime counter cycles to seconds.
 * @details Converts from realtime counter cycles number to seconds.
 * @note    The result is rounded up to the next second boundary.
 * @note    The macro assumes that @p freq >= @p 1.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] n         number of cycles
 * @return              The number of seconds.
 *
 * @api
 */
#define RTC2S(freq, n) ((((n) - 1UL) / (freq)) + 1UL)

/**
 * @brief   Realtime counter cycles to milliseconds.
 * @details Converts from realtime counter cycles number to milliseconds.
 * @note    The result is rounded up to the next millisecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] n         number of cycles
 * @return              The number of milliseconds.
 *
 * @api
 */
#define RTC2MS(freq, n) ((((n) - 1UL) / ((freq) / 1000UL)) + 1UL)

/**
 * @brief   Realtime counter cycles to microseconds.
 * @details Converts from realtime counter cycles number to microseconds.
 * @note    The result is rounded up to the next microsecond boundary.
 * @note    The macro assumes that @p freq >= @p 1000000.
 *
 * @param[in] freq      clock frequency, in Hz, of the realtime counter
 * @param[in] n         number of cycles
 * @return              The number of microseconds.
 *
 * @api
 */
#define RTC2US(freq, n) ((((n) - 1UL) / ((freq) / 1000000UL)) + 1UL)
/** @} */

/**
 * @brief   Returns the current value of the system real time counter.
 * @note    This function is only available if the port layer supports the
 *          option @p PORT_SUPPORTS_RT.
 *
 * @return              The value of the system realtime counter of
 *                      type rtcnt_t.
 *
 * @xclass
 */
#if (PORT_SUPPORTS_RT == TRUE) || defined(__DOXYGEN__)
#define chSysGetRealtimeCounterX() (rtcnt_t)port_rt_get_counter_value()
#endif

/**
 * @brief   Performs a context switch.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself or from within the port layer.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 *
 * @special
 */
#define chSysSwitch(ntp, otp) {                                             \
                                                                            \
  __trace_switch(ntp, otp);                                                 \
  __stats_ctxswc(ntp, otp);                                                 \
  CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp);                                     \
  port_switch(ntp, otp);                                                    \
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern ch_system_t ch_system;

extern const os_instance_config_t ch_core0_cfg;
extern os_instance_t ch0;

#if PORT_CORES_NUMBER > 1
extern const os_instance_config_t ch_core1_cfg;
extern os_instance_t ch1;
#endif

#endif /* !defined(__DOXYGEN__) */

#ifdef __cplusplus
extern "C" {
#endif
  void chSysWaitSystemState(system_state_t state);
  void chSysInit(void);
  thread_t *chSysGetIdleThreadX(void);
  bool chSysIntegrityCheckI(unsigned testmask);
  void chSysTimerHandlerI(void);
  syssts_t chSysGetStatusAndLockX(void);
  void chSysRestoreStatusX(syssts_t sts);
#if PORT_SUPPORTS_RT == TRUE
  bool chSysIsCounterWithinX(rtcnt_t cnt, rtcnt_t start, rtcnt_t end);
  void chSysPolledDelayX(rtcnt_t cycles);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Raises the system interrupt priority mask to the maximum level.
 * @details All the maskable interrupt sources are disabled regardless their
 *          hardware priority.
 * @note    Do not invoke this API from within a kernel lock.
 * @note    This API is no replacement for @p chSysLock()and @p chSysUnock()
 *          which could do more than just disable interrupts.
 *
 * @special
 */
static inline void chSysDisable(void) {

  port_disable();
  __dbg_check_disable();
}

/**
 * @brief   Raises the system interrupt priority mask to system level.
 * @details The interrupt sources that should not be able to preempt the kernel
 *          are disabled, interrupt sources with higher priority are still
 *          enabled.
 * @note    Do not invoke this API from within a kernel lock.
 * @note    This API is no replacement for @p chSysLock() which
 *          could do more than just disable interrupts.
 *
 * @special
 */
static inline void chSysSuspend(void) {

  port_suspend();
  __dbg_check_suspend();
}

/**
 * @brief   Lowers the system interrupt priority mask to user level.
 * @details All the interrupt sources are enabled.
 * @note    Do not invoke this API from within a kernel lock.
 * @note    This API is no replacement for @p chSysUnlock() which
 *          could do more than just enable interrupts.
 *
 * @special
 */
static inline void chSysEnable(void) {

  __dbg_check_enable();
  port_enable();
}

/**
 * @brief   Enters the kernel lock state.
 * @note    The exact behavior of this function is port-dependent and could
 *          not be limited to disabling interrupts.
 *
 * @special
 */
static inline void chSysLock(void) {

  port_lock();
  __stats_start_measure_crit_thd();
  __dbg_check_lock();
}

/**
 * @brief   Leaves the kernel lock state.
 * @note    The exact behavior of this function is port-dependent and could
 *          not be limited to enabling interrupts.
 *
 * @special
 */
static inline void chSysUnlock(void) {

  __dbg_check_unlock();
  __stats_stop_measure_crit_thd();

  /* The following condition can be triggered by the use of i-class functions
     in a critical section not followed by a chSchRescheduleS(), this means
     that the current thread has a lower priority than the next thread in
     the ready list.*/
  chDbgAssert((currcore->rlist.pqueue.next == &currcore->rlist.pqueue) ||
              (currcore->rlist.current->hdr.pqueue.prio >= currcore->rlist.pqueue.next->prio),
              "priority order violation");

  port_unlock();
}

/**
 * @brief   Enters the kernel lock state from within an interrupt handler.
 * @note    This API may do nothing on some architectures, it is required
 *          because on ports that support preemptable interrupt handlers
 *          it is required to raise the interrupt mask to the same level of
 *          the system mutual exclusion zone.<br>
 *          It is good practice to invoke this API before invoking any I-class
 *          syscall from an interrupt handler.
 * @note    The exact behavior of this function is port-dependent and could
 *          not be limited to disabling interrupts.
 * @note    This API must be invoked exclusively from interrupt handlers.
 *
 * @special
 */
static inline void chSysLockFromISR(void) {

  port_lock_from_isr();
  __stats_start_measure_crit_isr();
  __dbg_check_lock_from_isr();
}

/**
 * @brief   Leaves the kernel lock state from within an interrupt handler.
 *
 * @note    This API may do nothing on some architectures, it is required
 *          because on ports that support preemptable interrupt handlers
 *          it is required to raise the interrupt mask to the same level of
 *          the system mutual exclusion zone.<br>
 *          It is good practice to invoke this API after invoking any I-class
 *          syscall from an interrupt handler.
 * @note    The exact behavior of this function is port-dependent and could
 *          not be limited to enabling interrupts.
 * @note    This API must be invoked exclusively from interrupt handlers.
 *
 * @special
 */
static inline void chSysUnlockFromISR(void) {

  __dbg_check_unlock_from_isr();
  __stats_stop_measure_crit_isr();
  port_unlock_from_isr();
}

#if (CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Unconditionally enters the kernel lock state.
 * @note    Can be called without previous knowledge of the current lock state.
 *          The final state is "s-locked".
 * @note    This function is only available if the underlying port supports
 *          @p port_get_lock_status() and @p port_is_locked().
 *
 * @special
 */
static inline void chSysUnconditionalLock(void) {

  if (!port_is_locked(port_get_lock_status())) {
    chSysLock();
  }
}

/**
 * @brief   Unconditionally leaves the kernel lock state.
 * @note    Can be called without previous knowledge of the current lock state.
 *          The final state is "normal".
 * @note    This function is only available if the underlying port supports
 *          @p port_get_lock_status() and @p port_is_locked().
 *
 * @special
 */
static inline void chSysUnconditionalUnlock(void) {

  if (port_is_locked(port_get_lock_status())) {
    chSysUnlock();
  }
}
#endif /* CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE */

#if (CH_CFG_SMP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Notifies an OS instance to check for reschedule.
 * @details An OS instance is notified to check if a reschedule is required,
 *          the implementation is port-dependent.
 *
 * @param[in] oip       pointer to the instance to be notified
 */
static inline void chSysNotifyInstance(os_instance_t *oip) {

  port_notify_instance(oip);
}
#endif

#endif /* CHSYS_H */

/** @} */
