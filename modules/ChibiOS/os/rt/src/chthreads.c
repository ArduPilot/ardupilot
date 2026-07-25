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
 * @file    rt/src/chthreads.c
 * @brief   Threads code.
 *
 * @addtogroup threads
 * @details Threads related APIs and services.
 *          <h2>Operation mode</h2>
 *          A thread is an abstraction of an independent instructions flow.
 *          In ChibiOS/RT a thread is represented by a "C" function owning
 *          a processor context, state informations and a dedicated stack
 *          area. In this scenario static variables are shared among all
 *          threads while automatic variables are local to the thread.<br>
 *          Operations defined for threads:
 *          - <b>Create</b>, a thread is started on the specified thread
 *            function. This operation is available in multiple variants,
 *            both static and dynamic.
 *          - <b>Exit</b>, a thread terminates by returning from its top
 *            level function or invoking a specific API, the thread can
 *            return a value that can be retrieved by other threads.
 *          - <b>Wait</b>, a thread waits for the termination of another
 *            thread and retrieves its return value.
 *          - <b>Resume</b>, a thread created in suspended state is started.
 *          - <b>Sleep</b>, the execution of a thread is suspended for the
 *            specified amount of time or the specified future absolute time
 *            is reached.
 *          - <b>SetPriority</b>, a thread changes its own priority level.
 *          - <b>Yield</b>, a thread voluntarily renounces to its time slot.
 *          .
 * @{
 */

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

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
 * @brief   Initializes a thread structure.
 * @note    This is an internal functions, do not use it in application code.
 *
 * @param[in] oip       pointer to the OS instance
 * @param[in] tp        pointer to the thread
 * @param[in] name      thread name
 * @param[in] prio      the priority level for the new thread
 * @return              The same thread pointer passed as parameter.
 *
 * @notapi
 */
thread_t *__thd_object_init(os_instance_t *oip,
                            thread_t *tp,
                            const char *name,
                            tprio_t prio) {

  tp->hdr.pqueue.prio   = prio;
  tp->state             = CH_STATE_WTSTART;
  tp->flags             = CH_FLAG_MODE_STATIC;
  tp->owner             = oip;
#if CH_CFG_TIME_QUANTUM > 0
  tp->ticks             = (tslices_t)CH_CFG_TIME_QUANTUM;
#endif
#if CH_CFG_USE_MUTEXES == TRUE
  tp->realprio          = prio;
  tp->mtxlist           = NULL;
#endif
#if CH_CFG_USE_EVENTS == TRUE
  tp->epending          = (eventmask_t)0;
#endif
#if CH_DBG_THREADS_PROFILING == TRUE
  tp->time              = (systime_t)0;
#endif
#if CH_CFG_USE_REGISTRY == TRUE
  tp->refs              = (trefs_t)1;
  tp->name              = name;
  REG_INSERT(oip, tp);
#else
  (void)name;
#endif
#if CH_CFG_USE_WAITEXIT == TRUE
  ch_list_init(&tp->waiting);
#endif
#if CH_CFG_USE_MESSAGES == TRUE
  ch_queue_init(&tp->msgqueue);
#endif
#if CH_DBG_STATISTICS == TRUE
  chTMObjectInit(&tp->stats);
#endif
  CH_CFG_THREAD_INIT_HOOK(tp);
  return tp;
}

#if (CH_DBG_FILL_THREADS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Stack fill utility.
 *
 * @param[in] startp    first address to fill
 * @param[in] endp      last address to fill +1
 *
 * @notapi
 */
void __thd_stackfill(uint8_t *startp, uint8_t *endp) {

  do {
    *startp++ = CH_DBG_STACK_FILL_VALUE;
  } while (likely(startp < endp));
}
#endif /* CH_DBG_FILL_THREADS */

/**
 * @brief   Creates a new thread.
 * @details The new thread is initialized but not inserted in the ready list,
 *          the initial state is @p CH_STATE_WTSTART.
 * @post    The created thread has a reference counter set to one, it is
 *          caller responsibility to call @p chThdRelease() or @p chThdWait()
 *          in order to release the reference. The thread persists in the
 *          registry until its reference counter reaches zero.
 * @post    The initialized thread can be subsequently started by invoking
 *          @p chThdStart(), @p chThdStartI() or @p chSchWakeupS()
 *          depending on the execution context.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 * @note    Threads created using this function do not obey to the
 *          @p CH_DBG_FILL_THREADS debug option because it would keep
 *          the kernel locked for too much time.
 *
 * @param[in] tdp       pointer to the thread descriptor
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @iclass
 */
thread_t *chThdCreateSuspendedI(const thread_descriptor_t *tdp) {
  thread_t *tp;

  chDbgCheckClassI();
  chDbgCheck(tdp != NULL);
  chDbgCheck(MEM_IS_ALIGNED(tdp->wbase, PORT_WORKING_AREA_ALIGN) &&
             MEM_IS_ALIGNED(tdp->wend, PORT_STACK_ALIGN) &&
             (tdp->wend > tdp->wbase) &&
             (((size_t)tdp->wend - (size_t)tdp->wbase) >= THD_WORKING_AREA_SIZE(0)));
  chDbgCheck((tdp->prio <= HIGHPRIO) && (tdp->funcp != NULL));

  /* The thread structure is laid out in the upper part of the thread
     workspace. The thread position structure is aligned to the required
     stack alignment because it represents the stack top.*/
  tp = threadref(((uint8_t *)tdp->wend -
                 MEM_ALIGN_NEXT(sizeof (thread_t), PORT_STACK_ALIGN)));

#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
  /* Stack boundary.*/
  tp->wabase = tdp->wbase;
#endif

  /* Setting up the port-dependent part of the working area.*/
  PORT_SETUP_CONTEXT(tp, tdp->wbase, tp, tdp->funcp, tdp->arg);

  /* The thread object is initialized but not started.*/
#if CH_CFG_SMP_MODE != FALSE
  if (tdp->instance != NULL) {
    return __thd_object_init(tdp->instance, tp, tdp->name, tdp->prio);
  }
#endif

  return __thd_object_init(currcore, tp, tdp->name, tdp->prio);
}

/**
 * @brief   Creates a new thread.
 * @details The new thread is initialized but not inserted in the ready list,
 *          the initial state is @p CH_STATE_WTSTART.
 * @post    The created thread has a reference counter set to one, it is
 *          caller responsibility to call @p chThdRelease() or @p chThdWait()
 *          in order to release the reference. The thread persists in the
 *          registry until its reference counter reaches zero.
 * @post    The initialized thread can be subsequently started by invoking
 *          @p chThdStart(), @p chThdStartI() or @p chSchWakeupS()
 *          depending on the execution context.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 *
 * @param[in] tdp       pointer to the thread descriptor
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @api
 */
thread_t *chThdCreateSuspended(const thread_descriptor_t *tdp) {
  thread_t *tp;

#if CH_CFG_USE_REGISTRY == TRUE
  chDbgAssert(chRegFindThreadByWorkingArea(tdp->wbase) == NULL,
              "working area in use");
#endif

#if CH_DBG_FILL_THREADS == TRUE
  __thd_stackfill((uint8_t *)tdp->wbase, (uint8_t *)tdp->wend);
#endif

  chSysLock();
  tp = chThdCreateSuspendedI(tdp);
  chSysUnlock();

  return tp;
}

/**
 * @brief   Creates a new thread.
 * @details The new thread is initialized and make ready to execute.
 * @post    The created thread has a reference counter set to one, it is
 *          caller responsibility to call @p chThdRelease() or @p chThdWait()
 *          in order to release the reference. The thread persists in the
 *          registry until its reference counter reaches zero.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 * @note    Threads created using this function do not obey to the
 *          @p CH_DBG_FILL_THREADS debug option because it would keep
 *          the kernel locked for too much time.
 *
 * @param[in] tdp       pointer to the thread descriptor
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @iclass
 */
thread_t *chThdCreateI(const thread_descriptor_t *tdp) {

  return chSchReadyI(chThdCreateSuspendedI(tdp));
}

/**
 * @brief   Creates a new thread.
 * @details The new thread is initialized and make ready to execute.
 * @post    The created thread has a reference counter set to one, it is
 *          caller responsibility to call @p chThdRelease() or @p chThdWait()
 *          in order to release the reference. The thread persists in the
 *          registry until its reference counter reaches zero.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 *
 * @param[in] tdp       pointer to the thread descriptor
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @iclass
 */
thread_t *chThdCreate(const thread_descriptor_t *tdp) {
  thread_t *tp;

#if (CH_CFG_USE_REGISTRY == TRUE) &&                                        \
    ((CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE))
  chDbgAssert(chRegFindThreadByWorkingArea(tdp->wbase) == NULL,
              "working area in use");
#endif

#if CH_DBG_FILL_THREADS == TRUE
  __thd_stackfill((uint8_t *)tdp->wbase, (uint8_t *)tdp->wend);
#endif

  chSysLock();
  tp = chThdCreateSuspendedI(tdp);
  chSchWakeupS(tp, MSG_OK);
  chSysUnlock();

  return tp;
}

/**
 * @brief   Creates a new thread.
 * @post    The created thread has a reference counter set to one, it is
 *          caller responsibility to call @p chThdRelease() or @p chThdWait()
 *          in order to release the reference. The thread persists in the
 *          registry until its reference counter reaches zero.
 * @note    A thread can terminate by calling @p chThdExit() or by simply
 *          returning from its main function.
 *
 * @param[out] wsp      pointer to a working area dedicated to the thread stack
 * @param[in] size      size of the working area
 * @param[in] prio      priority level for the new thread
 * @param[in] pf        thread function
 * @param[in] arg       an argument passed to the thread function. It can be
 *                      @p NULL.
 * @return              The pointer to the @p thread_t structure allocated for
 *                      the thread into the working space area.
 *
 * @api
 */
thread_t *chThdCreateStatic(void *wsp, size_t size,
                            tprio_t prio, tfunc_t pf, void *arg) {
  thread_t *tp;

  chDbgCheck((wsp != NULL) &&
             MEM_IS_ALIGNED(wsp, PORT_WORKING_AREA_ALIGN) &&
             (size >= THD_WORKING_AREA_SIZE(0)) &&
             MEM_IS_ALIGNED(size, PORT_STACK_ALIGN) &&
             (prio <= HIGHPRIO) && (pf != NULL));

#if (CH_CFG_USE_REGISTRY == TRUE) &&                                        \
    ((CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE))
  chDbgAssert(chRegFindThreadByWorkingArea(wsp) == NULL,
              "working area in use");
#endif

#if CH_DBG_FILL_THREADS == TRUE
  __thd_stackfill((uint8_t *)wsp, (uint8_t *)wsp + size);
#endif

  chSysLock();

  /* The thread structure is laid out in the upper part of the thread
     workspace. The thread position structure is aligned to the required
     stack alignment because it represents the stack top.*/
  tp = threadref(((uint8_t *)wsp + size -
                 MEM_ALIGN_NEXT(sizeof (thread_t), PORT_STACK_ALIGN)));

#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
  /* Stack boundary.*/
  tp->wabase = (stkalign_t *)wsp;
#endif

  /* Setting up the port-dependent part of the working area.*/
  PORT_SETUP_CONTEXT(tp, wsp, tp, pf, arg);

  tp = __thd_object_init(currcore, tp, "noname", prio);

  /* Starting the thread immediately.*/
  chSchWakeupS(tp, MSG_OK);
  chSysUnlock();

  return tp;
}

/**
 * @brief   Starts a thread created with @p chThdCreateSuspended().
 *
 * @param[in] tp        pointer to the thread
 * @return              Thread to be started.
 *
 * @api
 */
thread_t *chThdStart(thread_t *tp) {

  chSysLock();
  chDbgAssert(tp->state == CH_STATE_WTSTART, "wrong state");
  chSchWakeupS(tp, MSG_OK);
  chSysUnlock();

  return tp;
}

#if (CH_CFG_USE_REGISTRY == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Adds a reference to a thread object.
 * @pre     The configuration option @p CH_CFG_USE_REGISTRY must be enabled in
 *          order to use this function.
 *
 * @param[in] tp        pointer to the thread
 * @return              The same thread pointer passed as parameter
 *                      representing the new reference.
 *
 * @api
 */
thread_t *chThdAddRef(thread_t *tp) {

  chSysLock();
  chDbgAssert(tp->refs < (trefs_t)255, "too many references");
  tp->refs++;
  chSysUnlock();

  return tp;
}

/**
 * @brief   Releases a reference to a thread object.
 * @details If the references counter reaches zero <b>and</b> the thread
 *          is in the @p CH_STATE_FINAL state then the thread's memory is
 *          returned to the proper allocator and the thread is removed
 *          from the registry.<br>
 *          Threads whose counter reaches zero and are still active become
 *          "detached". Detached static threads will be removed from the
 *          registry on termination. Detached non-static threads can only be
 *          removed by performing a registry scan operation.
 * @pre     The configuration option @p CH_CFG_USE_REGISTRY must be enabled in
 *          order to use this function.
 * @note    Static threads are not affected, only removed from the registry.
 *
 * @param[in] tp        pointer to the thread
 *
 * @api
 */
void chThdRelease(thread_t *tp) {

  chSysLock();
  chDbgAssert(tp->refs > (trefs_t)0, "not referenced");
  tp->refs--;

  /* If the references counter reaches zero and the thread is in its
     terminated state then the memory can be returned to the proper
     allocator.*/
  if ((tp->refs == (trefs_t)0) && (tp->state == CH_STATE_FINAL)) {
    REG_REMOVE(tp);
    chSysUnlock();

#if CH_CFG_USE_DYNAMIC == TRUE
    switch (tp->flags & CH_FLAG_MODE_MASK) {
#if CH_CFG_USE_HEAP == TRUE
    case CH_FLAG_MODE_HEAP:
      chHeapFree(chThdGetWorkingAreaX(tp));
      break;
#endif
#if CH_CFG_USE_MEMPOOLS == TRUE
    case CH_FLAG_MODE_MPOOL:
      chPoolFree(tp->mpool, chThdGetWorkingAreaX(tp));
      break;
#endif
    default:
      /* Nothing else to do for static threads.*/
      break;
    }
#endif /* CH_CFG_USE_DYNAMIC == TRUE */
    return;
  }
  chSysUnlock();
}
#endif /* CH_CFG_USE_REGISTRY == TRUE */

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
  chThdExitS(msg);
  /* The thread never returns here.*/
}

/**
 * @brief   Terminates the current thread.
 * @details The thread goes in the @p CH_STATE_FINAL state holding the
 *          specified exit status code, other threads can retrieve the
 *          exit status code by invoking the function @p chThdWait().
 * @post    Exiting a non-static thread that does not have references
 *          (detached) causes the thread to remain in the registry.
 *          It can only be removed by performing a registry scan operation.
 * @post    Eventual code after this function will never be executed,
 *          this function never returns. The compiler has no way to
 *          know this so do not assume that the compiler would remove
 *          the dead code.
 *
 * @param[in] msg       thread exit code
 *
 * @sclass
 */
void chThdExitS(msg_t msg) {
  thread_t *currtp = chThdGetSelfX();

  /* Storing exit message.*/
  currtp->u.exitcode = msg;

  /* Exit handler hook.*/
  CH_CFG_THREAD_EXIT_HOOK(currtp);

#if CH_CFG_USE_WAITEXIT == TRUE
  /* Waking up any waiting thread.*/
  while (unlikely(ch_list_notempty(&currtp->waiting))) {
    (void) chSchReadyI(threadref(ch_list_unlink(&currtp->waiting)));
  }
#endif

#if CH_CFG_USE_REGISTRY == TRUE
  if (unlikely(currtp->refs == (trefs_t)0)) {
#if CH_CFG_USE_DYNAMIC == TRUE
    /* Static threads are immediately removed from the registry because there
       is no memory to recover.*/
    if (unlikely(((currtp->flags & CH_FLAG_MODE_MASK) == CH_FLAG_MODE_STATIC))) {
      REG_REMOVE(currtp);
    }
#else
    REG_REMOVE(currtp);
#endif
  }
#endif

  /* Going into final state.*/
  chSchGoSleepS(CH_STATE_FINAL);

  /* The thread never returns here.*/
  chDbgAssert(false, "zombies apocalypse");
}

#if (CH_CFG_USE_WAITEXIT == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Blocks the execution of the invoking thread until the specified
 *          thread terminates then the exit code is returned.
 * @details This function waits for the specified thread to terminate then
 *          decrements its reference counter, if the counter reaches zero then
 *          the thread working area is returned to the proper allocator and
 *          the thread is removed from the registry.
 * @pre     The configuration option @p CH_CFG_USE_WAITEXIT must be enabled in
 *          order to use this function.
 * @post    Enabling @p chThdWait() requires 2-4 (depending on the
 *          architecture) extra bytes in the @p thread_t structure.
 * @note    If @p CH_CFG_USE_DYNAMIC is not specified this function just waits
 *          for the thread termination, no memory allocators are involved.
 *
 * @param[in] tp        pointer to the thread
 * @return              The exit code from the terminated thread.
 *
 * @api
 */
msg_t chThdWait(thread_t *tp) {
  thread_t *currtp = chThdGetSelfX();
  msg_t msg;

  chDbgCheck(tp != NULL);

  chSysLock();
  chDbgAssert(tp != currtp, "waiting self");
#if CH_CFG_USE_REGISTRY == TRUE
  chDbgAssert(tp->refs > (trefs_t)0, "no references");
#endif

  if (likely(tp->state != CH_STATE_FINAL)) {
    ch_list_link(&tp->waiting, &currtp->hdr.list);
    chSchGoSleepS(CH_STATE_WTEXIT);
  }
  msg = tp->u.exitcode;
  chSysUnlock();

#if CH_CFG_USE_REGISTRY == TRUE
  /* Releasing a reference to the thread.*/
  chThdRelease(tp);
#endif

  return msg;
}
#endif /* CH_CFG_USE_WAITEXIT */

/**
 * @brief   Changes the running thread priority level then reschedules if
 *          necessary.
 * @note    The function returns the real thread priority regardless of the
 *          current priority that could be higher than the real priority
 *          because the priority inheritance mechanism.
 *
 * @param[in] newprio   the new priority level of the running thread
 * @return              The old priority level.
 *
 * @api
 */
tprio_t chThdSetPriority(tprio_t newprio) {
  thread_t *currtp = chThdGetSelfX();
  tprio_t oldprio;

  chDbgCheck(newprio <= HIGHPRIO);

  chSysLock();
#if CH_CFG_USE_MUTEXES == TRUE
  oldprio = currtp->realprio;
  if ((currtp->hdr.pqueue.prio == currtp->realprio) ||
      (newprio > currtp->hdr.pqueue.prio)) {
    currtp->hdr.pqueue.prio = newprio;
  }
  currtp->realprio = newprio;
#else
  oldprio = currtp->hdr.pqueue.prio;
  currtp->hdr.pqueue.prio = newprio;
#endif
  chSchRescheduleS();
  chSysUnlock();

  return oldprio;
}

/**
 * @brief   Requests a thread termination.
 * @pre     The target thread must be written to invoke periodically
 *          @p chThdShouldTerminate() and terminate cleanly if it returns
 *          @p true.
 * @post    The specified thread will terminate after detecting the termination
 *          condition.
 *
 * @param[in] tp        pointer to the thread
 *
 * @api
 */
void chThdTerminate(thread_t *tp) {

  chSysLock();
  tp->flags |= CH_FLAG_TERMINATE;
  chSysUnlock();
}

/**
 * @brief   Suspends the invoking thread for the specified time.
 *
 * @param[in] time      the delay in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 *
 * @api
 */
void chThdSleep(sysinterval_t time) {

  chSysLock();
  chThdSleepS(time);
  chSysUnlock();
}

/**
 * @brief   Suspends the invoking thread until the system time arrives to the
 *          specified value.
 * @note    The function has no concept of "past", all specifiable times
 *          are in the future, this means that if you call this function
 *          exceeding your calculated intervals then the function will
 *          return in a far future time, not immediately.
 * @see     chThdSleepUntilWindowed()
 *
 * @param[in] time      absolute system time
 *
 * @api
 */
void chThdSleepUntil(systime_t time) {
  sysinterval_t interval;

  chSysLock();
  interval = chTimeDiffX(chVTGetSystemTimeX(), time);
  if (likely(interval > (sysinterval_t)0)) {
    chThdSleepS(interval);
  }
  chSysUnlock();
}

/**
 * @brief   Suspends the invoking thread until the system time arrives to the
 *          specified value.
 * @note    The system time is assumed to be between @p prev and @p next
 *          else the call is assumed to have been called outside the
 *          allowed time interval, in this case no sleep is performed.
 * @see     chThdSleepUntil()
 *
 * @param[in] prev      absolute system time of the previous deadline
 * @param[in] next      absolute system time of the next deadline
 * @return              the @p next parameter
 *
 * @api
 */
systime_t chThdSleepUntilWindowed(systime_t prev, systime_t next) {
  systime_t time;

  chSysLock();
  time = chVTGetSystemTimeX();
  if (likely(chTimeIsInRangeX(time, prev, next))) {
    chThdSleepS(chTimeDiffX(time, next));
  }
  chSysUnlock();

  return next;
}

/**
 * @brief   Yields the time slot.
 * @details Yields the CPU control to the next thread in the ready list with
 *          equal priority, if any.
 *
 * @api
 */
void chThdYield(void) {

  chSysLock();
  chSchDoYieldS();
  chSysUnlock();
}

/**
 * @brief   Sends the current thread sleeping and sets a reference variable.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @return              The wake up message.
 *
 * @sclass
 */
msg_t chThdSuspendS(thread_reference_t *trp) {
  thread_t *tp = chThdGetSelfX();

  chDbgAssert(*trp == NULL, "not NULL");

  *trp = tp;
  tp->u.wttrp = trp;
  chSchGoSleepS(CH_STATE_SUSPENDED);

  return chThdGetSelfX()->u.rdymsg;
}

/**
 * @brief   Sends the current thread sleeping and sets a reference variable.
 * @note    This function must reschedule, it can only be called from thread
 *          context.
 *
 * @param[in] trp       a pointer to a thread reference object
 * @param[in] timeout   the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not suspended and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
 *                      .
 * @return              The wake up message.
 * @retval MSG_TIMEOUT  if the operation timed out.
 *
 * @sclass
 */
msg_t chThdSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout) {
  thread_t *tp = chThdGetSelfX();

  chDbgAssert(*trp == NULL, "not NULL");

  if (unlikely(TIME_IMMEDIATE == timeout)) {
    return MSG_TIMEOUT;
  }

  *trp = tp;
  tp->u.wttrp = trp;

  return chSchGoSleepTimeoutS(CH_STATE_SUSPENDED, timeout);
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
    thread_t *tp = *trp;

    chDbgAssert(tp->state == CH_STATE_SUSPENDED, "not CH_STATE_SUSPENDED");

    *trp = NULL;
    tp->u.rdymsg = msg;
    (void) chSchReadyI(tp);
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
 * @iclass
 */
void chThdResumeS(thread_reference_t *trp, msg_t msg) {

  if (*trp != NULL) {
    thread_t *tp = *trp;

    chDbgAssert(tp->state == CH_STATE_SUSPENDED, "not CH_STATE_SUSPENDED");

    *trp = NULL;
    chSchWakeupS(tp, msg);
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
 * @brief   Enqueues the caller thread on a threads queue object.
 * @details The caller thread is enqueued and put to sleep until it is
 *          dequeued or the specified timeouts expires.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t object
 * @param[in] timeout   the timeout in system ticks, the special values are
 *                      handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state.
 *                      - @a TIME_IMMEDIATE the thread is not enqueued and
 *                        the function returns @p MSG_TIMEOUT as if a timeout
 *                        occurred.
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
  thread_t *currtp = chThdGetSelfX();

  if (unlikely(TIME_IMMEDIATE == timeout)) {
    return MSG_TIMEOUT;
  }

  ch_queue_insert(&tqp->queue, (ch_queue_t *)currtp);

  return chSchGoSleepTimeoutS(CH_STATE_QUEUED, timeout);
}

/**
 * @brief   Dequeues and wakes up one thread from the threads queue object,
 *          if any.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t object
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdDequeueNextI(threads_queue_t *tqp, msg_t msg) {

  if (ch_queue_notempty(&tqp->queue)) {
    chThdDoDequeueNextI(tqp, msg);
  }
}

/**
 * @brief   Dequeues and wakes up all threads from the threads queue object.
 *
 * @param[in] tqp       pointer to a @p threads_queue_t object
 * @param[in] msg       the message code
 *
 * @iclass
 */
void chThdDequeueAllI(threads_queue_t *tqp, msg_t msg) {

  while (ch_queue_notempty(&tqp->queue)) {
    chThdDoDequeueNextI(tqp, msg);
  }
}

/** @} */
