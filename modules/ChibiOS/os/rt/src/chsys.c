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
 * @file    rt/src/chsys.c
 * @brief   System related code.
 *
 * @addtogroup system
 * @details System related APIs and services:
 *          - Initialization.
 *          - Locks.
 *          - Interrupt Handling.
 *          - Power Management.
 *          - Abnormal Termination.
 *          - Realtime counter.
 *          .
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   System root object.
 */
ch_system_t ch_system;

/**
 * @brief   Core 0 OS instance.
 */
CH_SYS_CORE0_MEMORY os_instance_t ch0;

#if (CH_CFG_NO_IDLE_THREAD == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Working area for core 0 idle thread.
 */
static CH_SYS_CORE0_MEMORY THD_WORKING_AREA(ch_c0_idle_thread_wa,
                                            PORT_IDLE_THREAD_STACK_SIZE);
#endif

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
extern stkalign_t __main_thread_stack_base__, __main_thread_stack_end__;
#endif

/**
 * @brief   Core 0 OS instance configuration.
 */
const os_instance_config_t ch_core0_cfg = {
  .name             = "c0",
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
  .mainthread_base  = &__main_thread_stack_base__,
  .mainthread_end   = &__main_thread_stack_end__,
#elif CH_CFG_USE_DYNAMIC == TRUE
  .mainthread_base  = NULL,
  .mainthread_end   = NULL,
#endif
#if CH_CFG_NO_IDLE_THREAD == FALSE
  .idlethread_base  = THD_WORKING_AREA_BASE(ch_c0_idle_thread_wa),
  .idlethread_end   = THD_WORKING_AREA_END(ch_c0_idle_thread_wa)
#endif
};

#if (PORT_CORES_NUMBER > 1) || defined(__DOXYGEN__)
/**
 * @brief   Core 1 OS instance.
 */
CH_SYS_CORE1_MEMORY os_instance_t ch1;

#if (CH_CFG_NO_IDLE_THREAD == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Working area for core 1 idle thread.
 */
static CH_SYS_CORE1_MEMORY THD_WORKING_AREA(ch_c1_idle_thread_wa,
                                            PORT_IDLE_THREAD_STACK_SIZE);
#endif

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
extern stkalign_t __c1_main_thread_stack_base__, __c1_main_thread_stack_end__;
#endif

/**
 * @brief   Core 1 OS instance configuration.
 */
const os_instance_config_t ch_core1_cfg = {
  .name             = "c1",
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
  .mainthread_base  = &__c1_main_thread_stack_base__,
  .mainthread_end   = &__c1_main_thread_stack_end__,
#elif CH_CFG_USE_DYNAMIC == TRUE
  .mainthread_base  = NULL,
  .mainthread_end   = NULL,
#endif
#if CH_CFG_NO_IDLE_THREAD == FALSE
  .idlethread_base  = THD_WORKING_AREA_BASE(ch_c1_idle_thread_wa),
  .idlethread_end   = THD_WORKING_AREA_END(ch_c1_idle_thread_wa)
#endif
};
#endif /* PORT_CORES_NUMBER > 1 */

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
 * @brief   Waits for the system state to be equal to the specified one.
 * @note    Can be called before @p chSchObjectInit() in order to wait
 *          for system initialization by another core.
 *
 * @special
 */
void chSysWaitSystemState(system_state_t state) {

  while (ch_system.state != state) {
  }
}

/**
 * @brief   System initialization.
 * @details After executing this function the current instructions stream
 *          becomes the main thread.
 * @pre     Interrupts must disabled before invoking this function.
 * @post    The main thread is created with priority @p NORMALPRIO and
 *          interrupts are enabled.
 * @post    the system is in @p ch_sys_running state.
 *
 * @special
 */
void chSysInit(void) {
  unsigned i;

  /* System object initialization.*/
  ch_system.state = ch_sys_initializing;
  for (i = 0U; i < (unsigned)PORT_CORES_NUMBER; i++) {
    ch_system.instances[i] = NULL;
  }

#if CH_CFG_USE_TM == TRUE
  /* Time Measurement calibration.*/
  __tm_calibration_object_init(&ch_system.tmc);
#endif

#if (CH_CFG_USE_REGISTRY == TRUE) && (CH_CFG_SMP_MODE == TRUE)
  /* Registry initialization when SMP mode is enabled.*/
  __reg_object_init(&ch_system.reglist);
#endif

#if CH_CFG_SMP_MODE == TRUE
  /* RFCU initialization when SMP mode is enabled.*/
  __rfcu_object_init(&ch_system.rfcu);
#endif

  /* User system initialization hook.*/
  CH_CFG_SYSTEM_INIT_HOOK();

  /* OS library modules.*/
  __oslib_init();

  /* Initializing default OS instance.*/
  chInstanceObjectInit(&ch0, &ch_core0_cfg);

  /* It is alive now.*/
  ch_system.state = ch_sys_running;
  chSysUnlock();
}

/**
 * @brief   Halts the system.
 * @details This function is invoked by the operating system when an
 *          unrecoverable error is detected, for example because a programming
 *          error in the application code that triggers an assertion while
 *          in debug mode.
 * @note    Can be invoked from any system state.
 *
 * @param[in] reason        pointer to an error string
 *
 * @special
 */
void chSysHalt(const char *reason) {

  port_disable();

  /* Logging the event.*/
  __trace_halt(reason);

  /* Pointing to the passed message.*/
  currcore->dbg.panic_msg = reason;

  /* Halt hook code, usually empty.*/
  CH_CFG_SYSTEM_HALT_HOOK(reason);

#if defined(PORT_SYSTEM_HALT_HOOK)
  /* Port-related actions, this could include halting other instances
     via some inter-core messaging or other means.*/
  PORT_SYSTEM_HALT_HOOK();
#endif

  /* Entering the halted state.*/
  ch_system.state = ch_sys_halted;

  /* Harmless infinite loop.*/
  while (true) {
  }
}

/**
 * @brief   Returns a pointer to the idle thread.
 * @note    The reference counter of the idle thread is not incremented but
 *          it is not strictly required being the idle thread a static
 *          object.
 * @note    This function cannot be called from the idle thread itself,
 *          use @p chThdGetSelfX() in that case.
 *
 * @return              Pointer to the idle thread.
 *
 * @xclass
 */
thread_t *chSysGetIdleThreadX(void) {
  thread_t *tp = threadref(currcore->rlist.pqueue.prev);

  chDbgAssert(tp->hdr.pqueue.prio == IDLEPRIO, "not idle thread");

  return tp;
}

/**
 * @brief   System integrity check.
 * @details Performs an integrity check of the important ChibiOS/RT data
 *          structures.
 * @note    The appropriate action in case of failure is to halt the system
 *          before releasing the critical zone.
 * @note    If the system is corrupted then one possible outcome of this
 *          function is an exception caused by @p NULL or corrupted pointers
 *          in list elements. Exception vectors must be monitored as well.
 * @note    This function is not used internally, it is up to the
 *          application to define if and where to perform system
 *          checking.
 * @note    Performing all tests at once can be a slow operation and can
 *          degrade the system response time. It is suggested to execute
 *          one test at time and release the critical zone in between tests.
 *
 * @param[in] testmask  Each bit in this mask is associated to a test to be
 *                      performed.
 * @return              The test result.
 * @retval false        The test succeeded.
 * @retval true         Test failed.
 *
 * @iclass
 */
bool chSysIntegrityCheckI(unsigned testmask) {
  os_instance_t *oip = currcore;
  cnt_t n;

  chDbgCheckClassI();

  /* Ready List integrity check.*/
  if ((testmask & CH_INTEGRITY_RLIST) != 0U) {
    ch_priority_queue_t *pqp;

    /* Scanning the ready list forward.*/
    n = (cnt_t)0;
    pqp = oip->rlist.pqueue.next;
    while (pqp != &oip->rlist.pqueue) {
      n++;
      pqp = pqp->next;
    }

    /* Scanning the ready list backward.*/
    pqp = oip->rlist.pqueue.prev;
    while (pqp != &oip->rlist.pqueue) {
      n--;
      pqp = pqp->prev;
    }

    /* The number of elements must match.*/
    if (n != (cnt_t)0) {
      return true;
    }
  }

  /* Timers list integrity check.*/
  if ((testmask & CH_INTEGRITY_VTLIST) != 0U) {
    ch_delta_list_t *dlp;

    /* Scanning the timers list forward.*/
    n = (cnt_t)0;
    dlp = oip->vtlist.dlist.next;
    while (dlp != &oip->vtlist.dlist) {
      n++;
      dlp = dlp->next;
    }

    /* Scanning the timers list backward.*/
    dlp = oip->vtlist.dlist.prev;
    while (dlp != &oip->vtlist.dlist) {
      n--;
      dlp = dlp->prev;
    }

    /* The number of elements must match.*/
    if (n != (cnt_t)0) {
      return true;
    }
  }

#if CH_CFG_USE_REGISTRY == TRUE
  if ((testmask & CH_INTEGRITY_REGISTRY) != 0U) {
    ch_queue_t *qp, *rqp;

    /* Registry header, access to this list depends on the current
       kernel configuration.*/
    rqp = REG_HEADER(oip);

    /* Scanning the registry list forward.*/
    n = (cnt_t)0;
    qp = rqp->next;
    while (qp != rqp) {
      n++;
      qp = qp->next;
    }

    /* Scanning the registry list backward.*/
    qp = rqp->prev;
    while (qp != rqp) {
      n--;
      qp = qp->prev;
    }

    /* The number of elements must match.*/
    if (n != (cnt_t)0) {
      return true;
    }
  }
#endif /* CH_CFG_USE_REGISTRY == TRUE */

#if defined(PORT_INTEGRITY_CHECK)
  if ((testmask & CH_INTEGRITY_PORT) != 0U) {
    PORT_INTEGRITY_CHECK();
  }
#endif

  return false;
}

/**
 * @brief   Handles time ticks for round robin preemption and timer increments.
 * @details Decrements the remaining time quantum of the running thread
 *          and preempts it when the quantum is used up. Increments system
 *          time and manages the timers.
 * @note    The frequency of the timer determines the system tick granularity
 *          and, together with the @p CH_CFG_TIME_QUANTUM macro, the round robin
 *          interval.
 *
 * @iclass
 */
void chSysTimerHandlerI(void) {
#if (CH_CFG_TIME_QUANTUM > 0) || (CH_DBG_THREADS_PROFILING == TRUE)
  thread_t *currtp = chThdGetSelfX();
#endif

  chDbgCheckClassI();

#if CH_CFG_TIME_QUANTUM > 0
  /* Running thread has not used up quantum yet? */
  if (currtp->ticks > (tslices_t)0) {
    /* Decrement remaining quantum.*/
    currtp->ticks--;
  }
#endif
#if CH_DBG_THREADS_PROFILING == TRUE
  currtp->time++;
#endif
  chVTDoTickI();
  CH_CFG_SYSTEM_TICK_HOOK();
}

#if (CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Returns the execution status and enters a critical zone.
 * @details This functions enters into a critical zone and can be called
 *          from any context. Because its flexibility it is less efficient
 *          than @p chSysLock() which is preferable when the calling context
 *          is known.
 * @post    The system is in a critical zone.
 * @note    This function is only available if the underlying port supports
 *          @p port_get_lock_status() and @p port_is_locked().
 *
 * @return              The previous system status, the encoding of this
 *                      status word is architecture-dependent and opaque.
 *
 * @xclass
 */
syssts_t chSysGetStatusAndLockX(void) {

  syssts_t sts = port_get_lock_status();
  if (!port_is_locked(sts)) {
    if (port_is_isr_context()) {
      chSysLockFromISR();
    }
    else {
      chSysLock();
    }
  }
  return sts;
}

/**
 * @brief   Restores the specified execution status and leaves a critical zone.
 * @note    A call to @p chSchRescheduleS() is automatically performed
 *          if exiting the critical zone and if not in ISR context.
 * @note    This function is only available if the underlying port supports
 *          @p port_get_lock_status() and @p port_is_locked().
 *
 * @param[in] sts       the system status to be restored.
 *
 * @xclass
 */
void chSysRestoreStatusX(syssts_t sts) {

  if (!port_is_locked(sts)) {
    if (port_is_isr_context()) {
      chSysUnlockFromISR();
    }
    else {
      chSchRescheduleS();
      chSysUnlock();
    }
  }
}
#endif /* CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE */

#if (PORT_SUPPORTS_RT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Realtime window test.
 * @details This function verifies if the current realtime counter value
 *          lies within the specified range or not. The test takes care
 *          of the realtime counter wrapping to zero on overflow.
 * @note    When start==end then the function returns always false because a
 *          null time range is specified.
 * @note    This function is only available if the port layer supports the
 *          option @p PORT_SUPPORTS_RT.
 *
 * @param[in] cnt       the counter value to be tested
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @xclass
 */
bool chSysIsCounterWithinX(rtcnt_t cnt, rtcnt_t start, rtcnt_t end) {

  return (bool)(((rtcnt_t)cnt - (rtcnt_t)start) <
                ((rtcnt_t)end - (rtcnt_t)start));
}

/**
 * @brief   Polled delay.
 * @note    The real delay is always few cycles in excess of the specified
 *          value.
 * @note    This function is only available if the port layer supports the
 *          option @p PORT_SUPPORTS_RT.
 *
 * @param[in] cycles    number of cycles
 *
 * @xclass
 */
void chSysPolledDelayX(rtcnt_t cycles) {
  rtcnt_t start = chSysGetRealtimeCounterX();
  rtcnt_t end  = start + cycles;

  while (chSysIsCounterWithinX(chSysGetRealtimeCounterX(), start, end)) {
  }
}
#endif /* PORT_SUPPORTS_RT == TRUE */

/** @} */
