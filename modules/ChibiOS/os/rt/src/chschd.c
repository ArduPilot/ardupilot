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
 * @file    rt/src/chschd.c
 * @brief   Scheduler code.
 *
 * @addtogroup scheduler
 * @details This module provides the default portable scheduler code.
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

/**
 * @brief   Inserts a thread in the Ready List placing it behind its peers.
 * @details The thread is positioned behind all threads with higher or equal
 *          priority.
 * @pre     The thread must not be already inserted in any list through its
 *          @p next and @p prev or list corruption would occur.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] tp        the thread to be made ready
 * @return              The thread pointer.
 *
 * @notapi
 */
static thread_t *__sch_ready_behind(thread_t *tp) {

  chDbgAssert((tp->state != CH_STATE_READY) &&
              (tp->state != CH_STATE_FINAL),
              "invalid state");

  /* Tracing the event.*/
  __trace_ready(tp, tp->u.rdymsg);

  /* The thread is marked ready.*/
  tp->state = CH_STATE_READY;

  /* Insertion in the priority queue.*/
  return threadref(ch_pqueue_insert_behind(&tp->owner->rlist.pqueue,
                                           &tp->hdr.pqueue));
}

/**
 * @brief   Inserts a thread in the Ready List placing it ahead its peers.
 * @details The thread is positioned ahead all threads with higher or equal
 *          priority.
 * @pre     The thread must not be already inserted in any list through its
 *          @p next and @p prev or list corruption would occur.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] tp        the thread to be made ready
 * @return              The thread pointer.
 *
 * @notapi
 */
static thread_t *__sch_ready_ahead(thread_t *tp) {

  chDbgAssert((tp->state != CH_STATE_READY) &&
              (tp->state != CH_STATE_FINAL),
              "invalid state");

  /* Tracing the event.*/
  __trace_ready(tp, tp->u.rdymsg);

  /* The thread is marked ready.*/
  tp->state = CH_STATE_READY;

  /* Insertion in the priority queue.*/
  return threadref(ch_pqueue_insert_ahead(&tp->owner->rlist.pqueue,
                                          &tp->hdr.pqueue));
}

/**
 * @brief   Switches to the first thread on the runnable queue.
 * @details The current thread is positioned in the ready list behind all
 *          threads having the same priority. The thread regains its time
 *          quantum.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself.
 *
 * @notapi
 */
static void __sch_reschedule_behind(void) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);
  thread_t *ntp;

  /* Picks the first thread from the ready queue and makes it current.*/
  ntp = threadref(ch_pqueue_remove_highest(&oip->rlist.pqueue));
  ntp->state = CH_STATE_CURRENT;
  __instance_set_currthread(oip, ntp);

  /* Handling idle-leave hook.*/
  if (otp->hdr.pqueue.prio == IDLEPRIO) {
    CH_CFG_IDLE_LEAVE_HOOK();
  }

#if CH_CFG_TIME_QUANTUM > 0
  /* It went behind peers so it gets a new time quantum.*/
  otp->ticks = (tslices_t)CH_CFG_TIME_QUANTUM;
#endif

  /* Placing in ready list behind peers.*/
  otp = __sch_ready_behind(otp);

  /* Swap operation as tail call.*/
  chSysSwitch(ntp, otp);
}

/**
 * @brief   Switches to the first thread on the runnable queue.
 * @details The current thread is positioned in the ready list ahead of all
 *          threads having the same priority.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself.
 *
 * @notapi
 */
static void __sch_reschedule_ahead(void) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);
  thread_t *ntp;

  /* Picks the first thread from the ready queue and makes it current.*/
  ntp = threadref(ch_pqueue_remove_highest(&oip->rlist.pqueue));
  ntp->state = CH_STATE_CURRENT;
  __instance_set_currthread(oip, ntp);

  /* Handling idle-leave hook.*/
  if (otp->hdr.pqueue.prio == IDLEPRIO) {
    CH_CFG_IDLE_LEAVE_HOOK();
  }

  /* Placing in ready list ahead of peers.*/
  otp = __sch_ready_ahead(otp);

  /* Swap operation as tail call.*/
  chSysSwitch(ntp, otp);
}

/*
 * Timeout wakeup callback.
 */
static void __sch_wakeup(virtual_timer_t *vtp, void *p) {
  thread_t *tp = threadref(p);

  (void)vtp;

  chSysLockFromISR();
  switch (tp->state) {
  case CH_STATE_READY:
    /* Handling the special case where the thread has been made ready by
       another thread with higher priority.*/
    chSysUnlockFromISR();
    return;
  case CH_STATE_SUSPENDED:
    *tp->u.wttrp = NULL;
    break;
#if CH_CFG_USE_SEMAPHORES == TRUE
  case CH_STATE_WTSEM:
    chSemFastSignalI(tp->u.wtsemp);
#endif
    /* Falls through.*/
  case CH_STATE_QUEUED:
    /* Falls through.*/
#if CH_CFG_USE_MESSAGES == TRUE
  case CH_STATE_SNDMSGQ:
    /* Falls through.*/
#endif
#if (CH_CFG_USE_CONDVARS == TRUE) && (CH_CFG_USE_CONDVARS_TIMEOUT == TRUE)
  case CH_STATE_WTCOND:
#endif
    /* States requiring dequeuing.*/
    (void) ch_queue_dequeue(&tp->hdr.queue);
    break;
  default:
    /* Any other state, nothing to do.*/
    break;
  }

  /* Standard message for timeout conditions.*/
  tp->u.rdymsg = MSG_TIMEOUT;

  /* Goes behind peers because it went to sleep voluntarily.*/
  (void) __sch_ready_behind(tp);
  chSysUnlockFromISR();

  return;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

#if (CH_CFG_OPTIMIZE_SPEED == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   Inserts a thread into a priority ordered queue.
 * @note    The insertion is done by scanning the list from the highest
 *          priority toward the lowest.
 *
 * @param[in] qp        the pointer to the threads list header
 * @param[in] tp        the pointer to the thread to be inserted in the list
 *
 * @notapi
 */
void ch_sch_prio_insert(ch_queue_t *qp, ch_queue_t *tp) {

  ch_queue_t *cp = qp;
  do {
    cp = cp->next;
  } while ((cp != qp) &&
           (threadref(cp)->hdr.pqueue.prio >= threadref(tp)->hdr.pqueue.prio));
  tp->next       = cp;
  tp->prev       = cp->prev;
  tp->prev->next = tp;
  cp->prev       = tp;
}
#endif /* CH_CFG_OPTIMIZE_SPEED */

/**
 * @brief   Inserts a thread in the Ready List placing it behind its peers.
 * @details The thread is positioned behind all threads with higher or equal
 *          priority.
 * @pre     The thread must not be already inserted in any list through its
 *          @p next and @p prev or list corruption would occur.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] tp        the thread to be made ready
 * @return              The thread pointer.
 *
 * @iclass
 */
thread_t *chSchReadyI(thread_t *tp) {

  chDbgCheckClassI();
  chDbgCheck(tp != NULL);

#if CH_CFG_SMP_MODE == TRUE
  if (tp->owner != currcore) {
    /* Readying up the remote thread and triggering a reschedule on
       the other core.*/
    chSysNotifyInstance(tp->owner);
  }
#endif

  return __sch_ready_behind(tp);
}

/**
 * @brief   Puts the current thread to sleep into the specified state.
 * @details The thread goes into a sleeping state. The possible
 *          @ref thread_states are defined into @p chschd.h.
 *
 * @param[in] newstate  the new thread state
 *
 * @sclass
 */
void chSchGoSleepS(tstate_t newstate) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);
  thread_t *ntp;

  chDbgCheckClassS();

  chDbgAssert(otp != chSysGetIdleThreadX(), "sleeping in idle thread");
  chDbgAssert(otp->owner == oip, "invalid core");

  /* New state.*/
  otp->state = newstate;

#if CH_CFG_TIME_QUANTUM > 0
  /* The thread is renouncing its remaining time slices so it will have a new
     time quantum when it will wakeup.*/
  otp->ticks = (tslices_t)CH_CFG_TIME_QUANTUM;
#endif

  /* Next thread in ready list becomes current.*/
  ntp = threadref(ch_pqueue_remove_highest(&oip->rlist.pqueue));
  ntp->state = CH_STATE_CURRENT;
  __instance_set_currthread(oip, ntp);

  /* Handling idle-enter hook.*/
  if (ntp->hdr.pqueue.prio == IDLEPRIO) {
    CH_CFG_IDLE_ENTER_HOOK();
  }

  /* Swap operation as tail call.*/
  chSysSwitch(ntp, otp);
}

/**
 * @brief   Puts the current thread to sleep into the specified state with
 *          timeout specification.
 * @details The thread goes into a sleeping state, if it is not awakened
 *          explicitly within the specified timeout then it is forcibly
 *          awakened with a @p MSG_TIMEOUT low level message. The possible
 *          @ref thread_states are defined into @p chschd.h.
 *
 * @param[in] newstate  the new thread state
 * @param[in] timeout   the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE the thread enters an infinite sleep
 *                        state, this is equivalent to invoking
 *                        @p chSchGoSleepS() but, of course, less efficient.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @return              The wakeup message.
 * @retval MSG_TIMEOUT  if a timeout occurs.
 *
 * @sclass
 */
msg_t chSchGoSleepTimeoutS(tstate_t newstate, sysinterval_t timeout) {
  thread_t *tp = __instance_get_currthread(currcore);

  chDbgCheckClassS();

  if (TIME_INFINITE != timeout) {
    virtual_timer_t vt;

    chVTDoSetI(&vt, timeout, __sch_wakeup, (void *)tp);
    chSchGoSleepS(newstate);
    if (chVTIsArmedI(&vt)) {
      chVTDoResetI(&vt);
    }
  }
  else {
    chSchGoSleepS(newstate);
  }

  return tp->u.rdymsg;
}

/**
 * @brief   Wakes up a thread.
 * @details The thread is inserted into the ready list or immediately made
 *          running depending on its relative priority compared to the current
 *          thread.
 * @pre     The thread must not be already inserted in any list through its
 *          @p next and @p prev or list corruption would occur.
 * @note    It is equivalent to a @p chSchReadyI() followed by a
 *          @p chSchRescheduleS() but much more efficient.
 * @note    The function assumes that the current thread has the highest
 *          priority.
 *
 * @param[in] ntp       the thread to be made ready
 * @param[in] msg       the wakeup message
 *
 * @sclass
 */
void chSchWakeupS(thread_t *ntp, msg_t msg) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);

  chDbgCheckClassS();

  chDbgAssert((oip->rlist.pqueue.next == &oip->rlist.pqueue) ||
              (oip->rlist.current->hdr.pqueue.prio >= oip->rlist.pqueue.next->prio),
              "priority order violation");

  /* Storing the message to be retrieved by the target thread when it will
     restart execution.*/
  ntp->u.rdymsg = msg;

#if CH_CFG_SMP_MODE == TRUE
  if (ntp->owner != oip) {
    /* Readying up the remote thread and triggering a reschedule on
       the other core.*/
    chSysNotifyInstance(ntp->owner);
    (void) __sch_ready_behind(ntp);
    return;
  }
#endif

  /* If the woken thread has a not-greater priority than the current
     one then it is just inserted in the ready list else it made
     running immediately and the invoking thread goes in the ready
     list instead.
     Note, we are favoring the path where the woken thread has higher
     priority.*/
  if (unlikely(ntp->hdr.pqueue.prio <= otp->hdr.pqueue.prio)) {
    (void) __sch_ready_behind(ntp);
  }
  else {
    /* The old thread goes back in the ready list ahead of its peers
       because it has not exhausted its time slice.*/
    otp = __sch_ready_ahead(otp);

    /* Handling idle-leave hook.*/
    if (otp->hdr.pqueue.prio == IDLEPRIO) {
      CH_CFG_IDLE_LEAVE_HOOK();
    }

    /* The extracted thread is marked as current.*/
    ntp->state = CH_STATE_CURRENT;
    __instance_set_currthread(oip, ntp);

    /* Swap operation as tail call.*/
    chSysSwitch(ntp, otp);
  }
}

/**
 * @brief   Performs a reschedule if a higher priority thread is runnable.
 * @details If a thread with a higher priority than the current thread is in
 *          the ready list then make the higher priority thread running.
 * @note    Only local threads are considered, other cores are signaled
 *          and perform a reschedule locally.
 *
 * @sclass
 */
void chSchRescheduleS(void) {
  os_instance_t *oip = currcore;
  thread_t *tp = __instance_get_currthread(oip);

  chDbgCheckClassS();

  /* Note, we are favoring the path where the reschedule is necessary
     because higher priority threads are ready.*/
  if (likely(firstprio(&oip->rlist.pqueue) > tp->hdr.pqueue.prio)) {
    __sch_reschedule_ahead();
  }
}

#if !defined(CH_SCH_IS_PREEMPTION_REQUIRED_HOOKED)
/**
 * @brief   Evaluates if preemption is required.
 * @details The decision is taken by comparing the relative priorities and
 *          depending on the state of the round robin timeout counter.
 * @note    Not a user function, it is meant to be invoked from within
 *          the port layer in the IRQ-related preemption code.
 *
 * @retval true         if there is a thread that must go in running state
 *                      immediately.
 * @retval false        if preemption is not required.
 *
 * @special
 */
bool chSchIsPreemptionRequired(void) {
  os_instance_t *oip = currcore;
  thread_t *tp = __instance_get_currthread(oip);

  tprio_t p1 = firstprio(&oip->rlist.pqueue);
  tprio_t p2 = tp->hdr.pqueue.prio;

#if CH_CFG_TIME_QUANTUM > 0
  /* If the running thread has not reached its time quantum, reschedule only
     if the first thread on the ready queue has a higher priority.
     Otherwise, if the running thread has used up its time quantum, reschedule
     if the first thread on the ready queue has equal or higher priority.*/
  return (tp->ticks > (tslices_t)0) ? (p1 > p2) : (p1 >= p2);
#else
  /* If the round robin preemption feature is not enabled then performs a
     simpler comparison.*/
  return p1 > p2;
#endif
}
#endif /* !defined(CH_SCH_IS_PREEMPTION_REQUIRED_HOOKED) */

#if !defined(CH_SCH_DO_PREEMPTION_HOOKED)
/**
 * @brief   Switches to the first thread on the runnable queue.
 * @details The current thread is positioned in the ready list behind or
 *          ahead of all threads having the same priority depending on
 *          if it used its whole time slice.
 * @note    Not a user function, it is meant to be invoked from within
 *          the port layer in the IRQ-related preemption code.
 *
 * @special
 */
void chSchDoPreemption(void) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);
  thread_t *ntp;

  /* Picks the first thread from the ready queue and makes it current.*/
  ntp = threadref(ch_pqueue_remove_highest(&oip->rlist.pqueue));
  ntp->state = CH_STATE_CURRENT;
  __instance_set_currthread(oip, ntp);

  /* Handling idle-leave hook.*/
  if (otp->hdr.pqueue.prio == IDLEPRIO) {
    CH_CFG_IDLE_LEAVE_HOOK();
  }

#if CH_CFG_TIME_QUANTUM > 0
  /* If CH_CFG_TIME_QUANTUM is enabled then there are two different scenarios
     to handle on preemption: time quantum elapsed or not.*/
  if (otp->ticks == (tslices_t)0) {

    /* The thread consumed its time quantum so it is enqueued behind threads
       with same priority level, however, it acquires a new time quantum.*/
    otp = __sch_ready_behind(otp);

    /* The thread being swapped out receives a new time quantum.*/
    otp->ticks = (tslices_t)CH_CFG_TIME_QUANTUM;
  }
  else {
    /* The thread didn't consume all its time quantum so it is put ahead of
       threads with equal priority and does not acquire a new time quantum.*/
    otp = __sch_ready_ahead(otp);
  }
#else /* !(CH_CFG_TIME_QUANTUM > 0) */
  /* If the round-robin mechanism is disabled then the thread goes always
     ahead of its peers.*/
  otp = __sch_ready_ahead(otp);
#endif /* !(CH_CFG_TIME_QUANTUM > 0) */

  /* Swap operation as tail call.*/
  chSysSwitch(ntp, otp);
}
#endif /* !defined(CH_SCH_DO_PREEMPTION_HOOKED) */

#if !defined(CH_SCH_PREEMPTION_HOOKED)
/**
 * @brief   All-in-one preemption code.
 * @note    Not a user function, it is meant to be invoked from within
 *          the port layer in the IRQ-related preemption code.
 *
 * @special
 */
void chSchPreemption(void) {
  os_instance_t *oip = currcore;
  thread_t *tp = __instance_get_currthread(oip);
  tprio_t p1 = firstprio(&oip->rlist.pqueue);
  tprio_t p2 = tp->hdr.pqueue.prio;

  /* Note, we are favoring the path where preemption is necessary
     because higher priority threads are ready.*/
#if CH_CFG_TIME_QUANTUM > 0
  if (tp->ticks > (tslices_t)0) {
    if (likely(p1 > p2)) {
      __sch_reschedule_ahead();
    }
  }
  else {
    if (likely(p1 >= p2)) {
      __sch_reschedule_behind();
    }
  }
#else /* CH_CFG_TIME_QUANTUM == 0 */
  if (likely(p1 > p2)) {
    __sch_reschedule_ahead();
  }
#endif /* CH_CFG_TIME_QUANTUM == 0 */
}
#endif /* !defined(CH_SCH_PREEMPTION_HOOKED) */

/**
 * @brief   Yields the time slot.
 * @details Yields the CPU control to the next thread in the ready list with
 *          equal or higher priority, if any.
 *
 * @sclass
 */
void chSchDoYieldS(void) {
  os_instance_t *oip = currcore;
  thread_t *tp = __instance_get_currthread(oip);

  chDbgCheckClassS();

  /* If this function has been called then it is likely there are threads
     at same priority level.*/
  if (likely(firstprio(&oip->rlist.pqueue) >= tp->hdr.pqueue.prio)) {
    __sch_reschedule_behind();
  }
}

/**
 * @brief   Makes runnable the fist thread in the ready list, does not
 *          reschedule internally.
 * @details The current thread is positioned in the ready list ahead of all
 *          threads having the same priority.
 * @note    Not a user function, it is meant to be invoked by the scheduler
 *          itself.
 *
 * @return              The pointer to the thread being switched in.
 *
 * @special
 */
thread_t *chSchSelectFirst(void) {
  os_instance_t *oip = currcore;
  thread_t *otp = __instance_get_currthread(oip);
  thread_t *ntp;

  /* Picks the first thread from the ready queue and makes it current.*/
  ntp = threadref(ch_pqueue_remove_highest(&oip->rlist.pqueue));
  ntp->state = CH_STATE_CURRENT;
  __instance_set_currthread(oip, ntp);

  /* Handling idle-leave hook.*/
  if (otp->hdr.pqueue.prio == IDLEPRIO) {
    CH_CFG_IDLE_LEAVE_HOOK();
  }

  /* Placing in ready list ahead of peers.*/
  (void) __sch_ready_ahead(otp);

  return ntp;
}

/** @} */
