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
 * @file    nil/src/ch.c
 * @brief   Nil RTOS main source file.
 *
 * @addtogroup NIL_KERNEL
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   System data structures.
 */
os_instance_t nil;

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Retrieves the highest priority thread in the specified state and
 *          associated to the specified object.
 *
 * @param[in] state     thread state
 * @param[in] p         object pointer
 * @return              The pointer to the found thread.
 * @retval NULL         if the thread is not found.
 *
 * @notapi
 */
thread_t *nil_find_thread(tstate_t state, void *p) {
  thread_t *tp = nil.threads;

  while (tp < &nil.threads[CH_CFG_MAX_THREADS]) {
    /* Is this thread matching?*/
    if ((tp->state == state) && (tp->u1.p == p)) {
      return tp;
    }
    tp++;
  }
  return NULL;
}

/**
 * @brief   Puts in ready state all thread matching the specified status and
 *          associated object.
 *
 * @param[in] p         object pointer
 * @param[in] cnt       number of threads to be readied as a negative number,
 *                      non negative numbers are ignored
 * @param[in] msg       the wakeup message
 * @return              The number of readied threads.
 *
 * @notapi
 */
cnt_t nil_ready_all(void *p, cnt_t cnt, msg_t msg) {
  thread_t *tp = nil.threads;;

  while (cnt < (cnt_t)0) {

    chDbgAssert(tp < &nil.threads[CH_CFG_MAX_THREADS],
                "pointer out of range");

    /* Is this thread waiting on this queue?*/
    if ((tp->state == NIL_STATE_WTQUEUE) && (tp->u1.p == p)) {
      cnt++;
      (void) chSchReadyI(tp, msg);
    }
    tp++;
  }

  return cnt;
}

#if (CH_DBG_SYSTEM_STATE_CHECK == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Guard code for @p chSysDisable().
 *
 * @notapi
 */
void __dbg_check_disable(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#1");
  }
}

/**
 * @brief   Guard code for @p chSysSuspend().
 *
 * @notapi
 */
void __dbg_check_suspend(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#2");
  }
}

/**
 * @brief   Guard code for @p chSysEnable().
 *
 * @notapi
 */
void __dbg_check_enable(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#3");
  }
}

/**
 * @brief   Guard code for @p chSysLock().
 *
 * @notapi
 */
void __dbg_check_lock(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#4");
  }
  __dbg_enter_lock();
}

/**
 * @brief   Guard code for @p chSysUnlock().
 *
 * @notapi
 */
void __dbg_check_unlock(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt <= (cnt_t)0)) {
    chSysHalt("SV#5");
  }
  __dbg_leave_lock();
}

/**
 * @brief   Guard code for @p chSysLockFromIsr().
 *
 * @notapi
 */
void __dbg_check_lock_from_isr(void) {

  if ((nil.isr_cnt <= (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#6");
  }
  __dbg_enter_lock();
}

/**
 * @brief   Guard code for @p chSysUnlockFromIsr().
 *
 * @notapi
 */
void __dbg_check_unlock_from_isr(void) {

  if ((nil.isr_cnt <= (cnt_t)0) || (nil.lock_cnt <= (cnt_t)0)) {
    chSysHalt("SV#7");
  }
  __dbg_leave_lock();
}

/**
 * @brief   Guard code for @p CH_IRQ_PROLOGUE().
 *
 * @notapi
 */
void __dbg_check_enter_isr(void) {

  port_lock_from_isr();
  if ((nil.isr_cnt < (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#8");
  }
  nil.isr_cnt++;
  port_unlock_from_isr();
}

/**
 * @brief   Guard code for @p CH_IRQ_EPILOGUE().
 *
 * @notapi
 */
void __dbg_check_leave_isr(void) {

  port_lock_from_isr();
  if ((nil.isr_cnt <= (cnt_t)0) || (nil.lock_cnt != (cnt_t)0)) {
    chSysHalt("SV#9");
  }
  nil.isr_cnt--;
  port_unlock_from_isr();
}

/**
 * @brief   I-class functions context check.
 * @details Verifies that the system is in an appropriate state for invoking
 *          an I-class API function. A panic is generated if the state is
 *          not compatible.
 *
 * @api
 */
void chDbgCheckClassI(void) {

  if ((nil.isr_cnt < (cnt_t)0) || (nil.lock_cnt <= (cnt_t)0)) {
    chSysHalt("SV#10");
  }
}

/**
 * @brief   S-class functions context check.
 * @details Verifies that the system is in an appropriate state for invoking
 *          an S-class API function. A panic is generated if the state is
 *          not compatible.
 *
 * @api
 */
void chDbgCheckClassS(void) {

  if ((nil.isr_cnt != (cnt_t)0) || (nil.lock_cnt <= (cnt_t)0)) {
    chSysHalt("SV#11");
  }
}
#endif /* CH_DBG_SYSTEM_STATE_CHECK == TRUE */

/**
 * @brief   Initializes the kernel.
 * @details Initializes the kernel structures, the current instructions flow
 *          becomes the idle thread upon return. The idle thread must not
 *          invoke any kernel primitive able to change state to not runnable.
 * @note    This function assumes that the @p nil global variable has been
 *          zeroed by the runtime environment. If this is not the case then
 *          make sure to clear it before calling this function.
 *
 * @special
 */
void chSysInit(void) {
  const thread_descriptor_t *tdp;

  /* Optional library modules.*/
  __oslib_init();

  /* Architecture layer initialization.*/
  port_init(&nil);

  /* System initialization hook.*/
  CH_CFG_SYSTEM_INIT_HOOK();

  /* Making idle the current thread, this may change after rescheduling.*/
  nil.next = nil.current = &nil.threads[CH_CFG_MAX_THREADS];
  nil.current->state = NIL_STATE_READY;

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
  /* The idle thread is a special case because its stack is set up by the
     runtime environment.*/
  nil.threads[CH_CFG_MAX_THREADS].wabase = THD_IDLE_BASE;
#endif

  /* Interrupts partially enabled. It is equivalent to entering the
     kernel critical zone.*/
  chSysSuspend();
#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
  nil.lock_cnt = (cnt_t)1;
#endif

#if CH_CFG_AUTOSTART_THREADS == TRUE
  /* Iterates through the list of threads to be auto-started.*/
  tdp = nil_thd_configs;
  do {
    (void) chThdCreateI(tdp);
    tdp++;
  } while (tdp->funcp != NULL);
#endif

  /* Starting the dance.*/
  chSchRescheduleS();
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

#if NIL_DBG_ENABLED
  nil.dbg_panic_msg = reason;
#else
  (void)reason;
#endif

  /* Halt hook code, usually empty.*/
  CH_CFG_SYSTEM_HALT_HOOK(reason);

  /* Harmless infinite loop.*/
  while (true) {
  }
}

/**
 * @brief   Time management handler.
 * @note    This handler has to be invoked by a periodic ISR in order to
 *          reschedule the waiting threads.
 *
 * @iclass
 */
void chSysTimerHandlerI(void) {

  chDbgCheckClassI();

#if CH_CFG_ST_TIMEDELTA == 0
  thread_t *tp = &nil.threads[0];
  nil.systime++;
  do {
    /* Is the thread in a wait state with timeout?.*/
    if (tp->timeout > (sysinterval_t)0) {

      chDbgAssert(!NIL_THD_IS_READY(tp), "is ready");

      /* Did the timer reach zero?*/
      if (--tp->timeout == (sysinterval_t)0) {
        /* Timeout on queues/semaphores requires a special handling because
           the counter must be incremented.*/
        /*lint -save -e9013 [15.7] There is no else because it is not needed.*/
        if (NIL_THD_IS_WTQUEUE(tp)) {
          tp->u1.tqp->cnt++;
        }
        else if (NIL_THD_IS_SUSPENDED(tp)) {
          *tp->u1.trp = NULL;
        }
        /*lint -restore*/
        (void) chSchReadyI(tp, MSG_TIMEOUT);
      }
    }
    /* Lock released in order to give a preemption chance on those
       architectures supporting IRQ preemption.*/
    chSysUnlockFromISR();
    tp++;
    chSysLockFromISR();
  } while (tp < &nil.threads[CH_CFG_MAX_THREADS]);
#else
  thread_t *tp = &nil.threads[0];
  sysinterval_t next = (sysinterval_t)0;

  chDbgAssert(nil.nexttime == port_timer_get_alarm(), "time mismatch");

  do {
    sysinterval_t timeout = tp->timeout;

    /* Is the thread in a wait state with timeout?.*/
    if (timeout > (sysinterval_t)0) {

      chDbgAssert(!NIL_THD_IS_READY(tp), "is ready");
      chDbgAssert(timeout >= chTimeDiffX(nil.lasttime, nil.nexttime),
                  "skipped one");

      /* The volatile field is updated once, here.*/
      timeout -= chTimeDiffX(nil.lasttime, nil.nexttime);
      tp->timeout = timeout;

      if (timeout == (sysinterval_t)0) {
        /* Timeout on thread queues requires a special handling because the
           counter must be incremented.*/
        if (NIL_THD_IS_WTQUEUE(tp)) {
          tp->u1.tqp->cnt++;
        }
        else {
          if (NIL_THD_IS_SUSPENDED(tp)) {
            *tp->u1.trp = NULL;
          }
        }
        (void) chSchReadyI(tp, MSG_TIMEOUT);
      }
      else {
        if (timeout <= (sysinterval_t)(next - (sysinterval_t)1)) {
          next = timeout;
        }
      }
    }

    /* Lock released in order to give a preemption chance on those
       architectures supporting IRQ preemption.*/
    chSysUnlockFromISR();
    tp++;
    chSysLockFromISR();
  } while (tp < &nil.threads[CH_CFG_MAX_THREADS]);

  nil.lasttime = nil.nexttime;
  if (next > (sysinterval_t)0) {
    nil.nexttime = chTimeAddX(nil.nexttime, next);
    port_timer_set_alarm(nil.nexttime);
  }
  else {
    /* No tick event needed.*/
    port_timer_stop_alarm();
  }
#endif
}

#if (CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Unconditionally enters the kernel lock state.
 * @note    Can be called without previous knowledge of the current lock state.
 *          The final state is "s-locked".
 *
 * @special
 */
void chSysUnconditionalLock(void) {

  if (!port_is_locked(port_get_lock_status())) {
    chSysLock();
  }
}

/**
 * @brief   Unconditionally leaves the kernel lock state.
 * @note    Can be called without previous knowledge of the current lock state.
 *          The final state is "normal".
 *
 * @special
 */
void chSysUnconditionalUnlock(void) {

  if (port_is_locked(port_get_lock_status())) {
    chSysUnlock();
  }
}

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

/**
 * @brief   Makes the specified thread ready for execution.
 *
 * @param[in] tp        pointer to the @p thread_t object
 * @param[in] msg       the wakeup message
 *
 * @return              The same reference passed as parameter.
 */
thread_t *chSchReadyI(thread_t *tp, msg_t msg) {

  chDbgCheckClassI();
  chDbgCheck((tp >= nil.threads) && (tp < &nil.threads[CH_CFG_MAX_THREADS]));
  chDbgAssert(!NIL_THD_IS_READY(tp), "already ready");
  chDbgAssert(nil.next <= nil.current, "priority ordering");

  tp->u1.msg = msg;
  tp->state = NIL_STATE_READY;
  tp->timeout = (sysinterval_t)0;
  if (tp < nil.next) {
    nil.next = tp;
  }
  return tp;
}

/**
 * @brief   Evaluates if preemption is required.
 * @details The decision is taken by comparing the relative priorities and
 *          depending on the state of the round robin timeout counter.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself or from within the port layer.
 *
 * @retval true         if there is a thread that must go in running state
 *                      immediately.
 * @retval false        if preemption is not required.
 *
 * @special
 */
bool chSchIsPreemptionRequired(void) {

  return chSchIsRescRequiredI();
}

/**
 * @brief   Switches to the first thread on the runnable queue.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself or from within the port layer.
 *
 * @special
 */
void chSchDoPreemption(void) {
  thread_t *otp = nil.current;

  nil.current = nil.next;
  if (otp == &nil.threads[CH_CFG_MAX_THREADS]) {
    CH_CFG_IDLE_LEAVE_HOOK();
  }
  port_switch(nil.next, otp);
}

/**
 * @brief   Reschedules if needed.
 *
 * @sclass
 */
void chSchRescheduleS(void) {

  chDbgCheckClassS();

  if (chSchIsRescRequiredI()) {
    chSchDoPreemption();
  }
}

/**
 * @brief   Puts the current thread to sleep into the specified state with
 *          timeout specification.
 * @details The thread goes into a sleeping state, if it is not awakened
 *          explicitly within the specified system time then it is forcibly
 *          awakened with a @p MSG_TIMEOUT low level message.
 *
 * @param[in] newstate  the new thread state or a semaphore pointer
 * @param[in] timeout   the number of ticks before the operation timeouts.
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The wakeup message.
 * @retval MSG_TIMEOUT  if a timeout occurred.
 *
 * @sclass
 */
msg_t chSchGoSleepTimeoutS(tstate_t newstate, sysinterval_t timeout) {
  thread_t *ntp, *otp = nil.current;

  chDbgCheckClassS();

  chDbgAssert(otp != &nil.threads[CH_CFG_MAX_THREADS],
               "idle cannot sleep");

  /* Storing the wait object for the current thread.*/
  otp->state = newstate;

#if CH_CFG_ST_TIMEDELTA > 0
  if (timeout != TIME_INFINITE) {
    systime_t abstime;

    /* TIMEDELTA makes sure to have enough time to reprogram the timer
       before the free-running timer counter reaches the selected timeout.*/
    if (timeout < (sysinterval_t)CH_CFG_ST_TIMEDELTA) {
      timeout = (sysinterval_t)CH_CFG_ST_TIMEDELTA;
    }

    /* Absolute time of the timeout event.*/
    abstime = chTimeAddX(chVTGetSystemTimeX(), timeout);

    if (nil.lasttime == nil.nexttime) {
      /* Special case, first thread asking for a timeout.*/
      port_timer_start_alarm(abstime);
      nil.nexttime = abstime;
    }
    else {
      /* Special case, there are already other threads with a timeout
         activated, evaluating the order.*/
      if (chTimeIsInRangeX(abstime, nil.lasttime, nil.nexttime)) {
        port_timer_set_alarm(abstime);
        nil.nexttime = abstime;
      }
    }

    /* Timeout settings.*/
    otp->timeout = abstime - nil.lasttime;
  }
#else

  /* Timeout settings.*/
  otp->timeout = timeout;
#endif

  /* Scanning the whole threads array.*/
  ntp = nil.threads;
  while (true) {
    /* Is this thread ready to execute?*/
    if (NIL_THD_IS_READY(ntp)) {
      nil.current = nil.next = ntp;
      if (ntp == &nil.threads[CH_CFG_MAX_THREADS]) {
        CH_CFG_IDLE_ENTER_HOOK();
      }
      port_switch(ntp, otp);
      return nil.current->u1.msg;
    }

    /* Points to the next thread in lowering priority order.*/
    ntp++;
    chDbgAssert(ntp <= &nil.threads[CH_CFG_MAX_THREADS],
                "pointer out of range");
  }
}

/**
 * @brief   Checks if the specified time is within the specified time range.
 * @note    When start==end then the function returns always false because the
 *          time window has zero size.
 *
 * @param[in] time      the time to be verified
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @xclass
 */
bool chTimeIsInRangeX(systime_t time, systime_t start, systime_t end) {

  return (bool)((systime_t)((systime_t)(time) - (systime_t)(start)) <
                (systime_t)((systime_t)(end) - (systime_t)(start)));
}

/**
 * @brief   Creates a new thread into a static memory area.
 * @details The new thread is initialized and make ready to execute.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 *
 * @param[out] tdp      pointer to the thread descriptor structure
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread.
 *
 * @iclass
 */
thread_t *chThdCreateI(const thread_descriptor_t *tdp) {
  thread_t *tp;

  chDbgCheck((tdp->prio < (tprio_t)CH_CFG_MAX_THREADS) &&
             (tdp->wbase != NULL) &&
             MEM_IS_ALIGNED(tdp->wbase, PORT_WORKING_AREA_ALIGN) &&
             (tdp->wend > tdp->wbase) &&
             MEM_IS_ALIGNED(tdp->wbase, PORT_STACK_ALIGN) &&
             (tdp->funcp != NULL));

  chDbgCheckClassI();

  /* Pointer to the thread slot to be used.*/
  tp = &nil.threads[tdp->prio];
  chDbgAssert(NIL_THD_IS_WTSTART(tp) || NIL_THD_IS_FINAL(tp),
              "priority slot taken");

#if CH_CFG_USE_EVENTS == TRUE
  tp->epmask = (eventmask_t)0;
#endif
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
  tp->wabase = (stkalign_t *)tdp->wbase;
#endif

  /* Port dependent thread initialization.*/
  PORT_SETUP_CONTEXT(tp, tdp->wbase, tdp->wend, tdp->funcp, tdp->arg);

  /* Initialization hook.*/
  CH_CFG_THREAD_EXT_INIT_HOOK(tp);

  /* Readying up thread.*/
  return chSchReadyI(tp, MSG_OK);
}

/**
 * @brief   Creates a new thread into a static memory area.
 * @details The new thread is initialized and make ready to execute.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 *
 * @param[out] tdp      pointer to the thread descriptor structure
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread.
 *
 * @api
 */
thread_t *chThdCreate(const thread_descriptor_t *tdp) {
  thread_t *tp;

  chSysLock();
  tp = chThdCreateI(tdp);
  chSchRescheduleS();
  chSysUnlock();

  return tp;
}

/**
 * @brief   Terminates the current thread.
 * @details The thread goes in the @p CH_STATE_FINAL state holding the
 *          specified exit status code, other threads can retrieve the
 *          exit status code by invoking the function @p chThdWait().
 * @post    Eventual code after this function will never be executed,
 *          this function never returns. The compiler has no way to
 *          know this so do not assume that the compiler would remove
 *          the dead code.
 *
 * @param[in] msg       thread exit code
 *
 * @api
 */
void chThdExit(msg_t msg) {

  chSysLock();

  /* Exit handler hook.*/
  CH_CFG_THREAD_EXIT_HOOK(nil.current);

#if CH_CFG_USE_WAITEXIT == TRUE
  {
    /* Waking up any waiting thread.*/
    thread_t *tp = nil.threads;
    while (tp < &nil.threads[CH_CFG_MAX_THREADS]) {
      /* Is this thread waiting for current thread termination?*/
      if ((tp->state == NIL_STATE_WTEXIT) && (tp->u1.tp == nil.current)) {
        (void) chSchReadyI(tp, msg);
      }
      tp++;
    }
  }
#endif

  /* Going into final state with exit message stored.*/
  nil.current->u1.msg = msg;
  (void) chSchGoSleepTimeoutS(NIL_STATE_FINAL, TIME_INFINITE);

  /* The thread never returns here.*/
  chDbgAssert(false, "zombies apocalypse");
}

/**
 * @brief   Blocks the execution of the invoking thread until the specified
 *          thread terminates then the exit code is returned.
 *
 * @param[in] tp        pointer to the thread
 * @return              The exit code from the terminated thread.
 *
 * @api
 */
msg_t chThdWait(thread_t *tp) {
  msg_t msg;

  chSysLock();
  if (NIL_THD_IS_FINAL(tp)) {
    msg = tp->u1.msg;
  }
  else {
    nil.current->u1.tp = tp;
    msg = chSchGoSleepTimeoutS(NIL_STATE_WTEXIT, TIME_INFINITE);
  }
  chSysUnlock();

  return msg;
}

/**
 * @brief   Sends the current thread sleeping and sets a reference variable.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The wake up message.
 *
 * @sclass
 */
msg_t chThdSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout) {

  chDbgAssert(*trp == NULL, "not NULL");

  if (TIME_IMMEDIATE == timeout) {
    return MSG_TIMEOUT;
  }

  *trp = nil.current;
  nil.current->u1.trp = trp;
  return chSchGoSleepTimeoutS(NIL_STATE_SUSPENDED, timeout);
}

/**
 * @brief   Wakes up a thread waiting on a thread reference object.
 * @note    This function must not reschedule because it can be called from
 *          ISR context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdResumeI(thread_reference_t *trp, msg_t msg) {

  if (*trp != NULL) {
    thread_reference_t tr = *trp;

    chDbgAssert(NIL_THD_IS_SUSPENDED(tr), "not suspended");

    *trp = NULL;
    (void) chSchReadyI(tr, msg);
  }
}

/**
 * @brief   Wakes up a thread waiting on a thread reference object.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] msg       the message code
 *
 * @api
 */
void chThdResume(thread_reference_t *trp, msg_t msg) {

  chSysLock();
  chThdResumeS(trp, msg);
  chSysUnlock();
}

/**
 * @brief   Suspends the invoking thread for the specified time.
 *
 * @param[in] timeout   the delay in system ticks
 *
 * @api
 */
void chThdSleep(sysinterval_t timeout) {

  chSysLock();
  chThdSleepS(timeout);
  chSysUnlock();
}

/**
 * @brief   Suspends the invoking thread until the system time arrives to the
 *          specified value.
 *
 * @param[in] abstime   absolute system time
 *
 * @api
 */
void chThdSleepUntil(systime_t abstime) {

  chSysLock();
  chThdSleepUntilS(abstime);
  chSysUnlock();
}

/**
 * @brief   Enqueues the caller thread on a threads queue object.
 * @details The caller thread is enqueued and put to sleep until it is
 *          dequeued or the specified timeouts expires.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t structure
 * @param[in] timeout   the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The message from @p osalQueueWakeupOneI() or
 *                      @p osalQueueWakeupAllI() functions.
 * @retval MSG_TIMEOUT  if the thread has not been dequeued within the
 *                      specified timeout or if the function has been
 *                      invoked with @p TIME_IMMEDIATE as timeout
 *                      specification.
 *
 * @sclass
 */
msg_t chThdEnqueueTimeoutS(threads_queue_t *tqp, sysinterval_t timeout) {

  chDbgCheckClassS();
  chDbgCheck(tqp != NULL);

  chDbgAssert(tqp->cnt <= (cnt_t)0, "invalid counter");

  if (TIME_IMMEDIATE == timeout) {
    return MSG_TIMEOUT;
  }

  tqp->cnt--;
  nil.current->u1.tqp = tqp;
  return chSchGoSleepTimeoutS(NIL_STATE_WTQUEUE, timeout);
}

/**
 * @brief   Dequeues and wakes up one thread from the threads queue object.
 * @details Dequeues one thread from the queue without checking if the queue
 *          is empty.
 * @pre     The queue must contain at least an object.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t structure
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdDoDequeueNextI(threads_queue_t *tqp, msg_t msg) {
  thread_t *tp;

  chDbgAssert(tqp->cnt < (cnt_t)0, "empty queue");

  tqp->cnt++;
  tp = nil_find_thread(NIL_STATE_WTQUEUE, (void *)tqp);

  chDbgAssert(tp != NULL, "thread not found");

  (void) chSchReadyI(tp, msg);
}

/**
 * @brief   Dequeues and wakes up one thread from the threads queue object,
 *          if any.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t structure
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdDequeueNextI(threads_queue_t *tqp, msg_t msg) {

  chDbgCheckClassI();
  chDbgCheck(tqp != NULL);

  if (tqp->cnt < (cnt_t)0) {
    chThdDoDequeueNextI(tqp, msg);
  }
}

/**
 * @brief   Dequeues and wakes up all threads from the threads queue object.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t structure
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdDequeueAllI(threads_queue_t *tqp, msg_t msg) {

  chDbgCheckClassI();
  chDbgCheck(tqp != NULL);

  tqp->cnt = nil_ready_all((void *)tqp, tqp->cnt, msg);
}

/** @} */
