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
 * @file    nil/src/chevt.c
 * @brief   Nil RTOS events source file.
 *
 * @addtogroup NIL_EVENTS
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

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
 * @brief   Registers an Event Listener on an Event Source.
 * @details Once a thread has registered as listener on an event source it
 *          will be notified of all events broadcasted there.
 * @note    Multiple Event Listeners can specify the same bits to be ORed to
 *          different threads.
 *
 * @param[in] esp       pointer to the  @p event_source_t structure
 * @param[in] elp       pointer to the @p event_listener_t structure
 * @param[in] events    events to be ORed to the thread when
 *                      the event source is broadcasted
 * @param[in] wflags    mask of flags the listening thread is interested in
 *
 * @api
 */
void chEvtRegisterMaskWithFlags(event_source_t *esp,
                                event_listener_t *elp,
                                eventmask_t events,
                                eventflags_t wflags) {

  chDbgCheck((esp != NULL) && (elp != NULL));

  chSysLock();
  elp->next     = esp->next;
  esp->next     = elp;
  elp->listener = chThdGetSelfX();
  elp->events   = events;
  elp->flags    = (eventflags_t)0;
  elp->wflags   = wflags;
  chSysUnlock();
}

/**
 * @brief   Unregisters an Event Listener from its Event Source.
 * @note    If the event listener is not registered on the specified event
 *          source then the function does nothing.
 * @note    For optimal performance it is better to perform the unregister
 *          operations in inverse order of the register operations (elements
 *          are found on top of the list).
 *
 * @param[in] esp       pointer to the  @p event_source_t structure
 * @param[in] elp       pointer to the @p event_listener_t structure
 *
 * @api
 */
void chEvtUnregister(event_source_t *esp, event_listener_t *elp) {
  event_listener_t *p;

  chDbgCheck((esp != NULL) && (elp != NULL));

  /*lint -save -e9087 -e740 [11.3, 1.3] Cast required by list handling.*/
  p = (event_listener_t *)esp;
  /*lint -restore*/
  chSysLock();
  /*lint -save -e9087 -e740 [11.3, 1.3] Cast required by list handling.*/
  while (p->next != (event_listener_t *)esp) {
  /*lint -restore*/
    if (p->next == elp) {
      p->next = elp->next;
      break;
    }
    p = p->next;
  }
  chSysUnlock();
}

/**
 * @brief   Clears the pending events specified in the events mask.
 *
 * @param[in] events    the events to be cleared
 * @return              The mask of pending events that were cleared.
 *
 * @iclass
 */
eventmask_t chEvtGetAndClearEventsI(eventmask_t events) {
  eventmask_t m;

  m = chThdGetSelfX()->epmask & events;
  chThdGetSelfX()->epmask &= ~events;

  return m;
}

/**
 * @brief   Clears the pending events specified in the events mask.
 *
 * @param[in] events    the events to be cleared
 * @return              The mask of pending events that were cleared.
 *
 * @api
 */
eventmask_t chEvtGetAndClearEvents(eventmask_t events) {
  eventmask_t m;

  chSysLock();
  m = chEvtGetAndClearEventsI(events);
  chSysUnlock();

  return m;
}

/**
 * @brief   Adds (OR) a set of events to the current thread, this is
 *          @b much faster than using @p chEvtBroadcast() or @p chEvtSignal().
 *
 * @param[in] events    the events to be added
 * @return              The mask of currently pending events.
 *
 * @api
 */
eventmask_t chEvtAddEvents(eventmask_t events) {
  eventmask_t newevt;

  chSysLock();
  newevt = chEvtAddEventsI(events);
  chSysUnlock();

  return newevt;
}

/**
 * @brief   Signals all the Event Listeners registered on the specified Event
 *          Source.
 * @details This function variants ORs the specified event flags to all the
 *          threads registered on the @p event_source_t in addition to the
 *          event flags specified by the threads themselves in the
 *          @p event_listener_t objects.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 * @param[in] flags     the flags set to be added to the listener flags mask
 *
 * @iclass
 */
void chEvtBroadcastFlagsI(event_source_t *esp, eventflags_t flags) {
  event_listener_t *elp;

  chDbgCheckClassI();
  chDbgCheck(esp != NULL);

  elp = esp->next;
  /*lint -save -e9087 -e740 [11.3, 1.3] Cast required by list handling.*/
  while (elp != (event_listener_t *)esp) {
  /*lint -restore*/
    elp->flags |= flags;
    /* When flags == 0 the thread will always be signaled because the
       source does not emit any flag.*/
    if ((flags == (eventflags_t)0) ||
        ((flags & elp->wflags) != (eventflags_t)0)) {
      chEvtSignalI(elp->listener, elp->events);
    }
    elp = elp->next;
  }
}

/**
 * @brief   Returns the flags associated to an @p event_listener_t.
 * @details The flags are returned and the @p event_listener_t flags mask is
 *          cleared.
 *
 * @param[in] elp       pointer to the @p event_listener_t structure
 * @return              The flags added to the listener by the associated
 *                      event source.
 *
 * @api
 */
eventflags_t chEvtGetAndClearFlags(event_listener_t *elp) {
  eventflags_t flags;

  chSysLock();
  flags = elp->flags;
  elp->flags = (eventflags_t)0;
  chSysUnlock();

  return flags & elp->wflags;
}

/**
 * @brief   Adds a set of event flags directly to the specified @p thread_t.
 *
 * @param[in] tp        the thread to be signaled
 * @param[in] events    the event flags set to be ORed
 *
 * @api
 */
void chEvtSignal(thread_t *tp, eventmask_t events) {

  chSysLock();
  chEvtSignalI(tp, events);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Adds a set of event flags directly to the specified @p thread_t.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] tp        the thread to be signaled
 * @param[in] events    the event flags set to be ORed
 *
 * @iclass
 */
void chEvtSignalI(thread_t *tp, eventmask_t events) {

  chDbgCheckClassI();
  chDbgCheck(tp != NULL);

  tp->epmask |= events;
  if ((NIL_THD_IS_WTOREVT(tp) &&
       ((tp->epmask & tp->u1.ewmask) != (eventmask_t)0)) ||
      (NIL_THD_IS_WTANDEVT(tp) &&
       ((tp->epmask & tp->u1.ewmask) == tp->u1.ewmask))) {
    (void) chSchReadyI(tp, MSG_OK);
  }
}

/**
 * @brief   Signals all the Event Listeners registered on the specified Event
 *          Source.
 * @details This function variants ORs the specified event flags to all the
 *          threads registered on the @p event_source_t in addition to the
 *          event flags specified by the threads themselves in the
 *          @p event_listener_t objects.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 * @param[in] flags     the flags set to be added to the listener flags mask
 *
 * @api
 */
void chEvtBroadcastFlags(event_source_t *esp, eventflags_t flags) {

  chSysLock();
  chEvtBroadcastFlagsI(esp, flags);
  chSchRescheduleS();
  chSysUnlock();
}

/**
 * @brief   Returns the unmasked flags associated to an @p event_listener_t.
 * @details The flags are returned and the @p event_listener_t flags mask is
 *          cleared.
 *
 * @param[in] elp       pointer to the @p event_listener_t structure
 * @return              The flags added to the listener by the associated
 *                      event source.
 *
 * @iclass
 */
eventflags_t chEvtGetAndClearFlagsI(event_listener_t *elp) {
  eventflags_t flags;

  flags = elp->flags;
  elp->flags = (eventflags_t)0;

  return flags & elp->wflags;
}

/**
 * @brief   Invokes the event handlers associated to an event flags mask.
 *
 * @param[in] events    mask of events to be dispatched
 * @param[in] handlers  an array of @p evhandler_t. The array must have size
 *                      equal to the number of bits in eventmask_t.
 *
 * @api
 */
void chEvtDispatch(const evhandler_t *handlers, eventmask_t events) {
  eventid_t eid;

  chDbgCheck(handlers != NULL);

  eid = (eventid_t)0;
  while (events != (eventmask_t)0) {
    if ((events & EVENT_MASK(eid)) != (eventmask_t)0) {
      chDbgAssert(handlers[eid] != NULL, "null handler");
      events &= ~EVENT_MASK(eid);
      handlers[eid](eid);
    }
    eid++;
  }
}

/**
 * @brief   Waits for exactly one of the specified events.
 * @details The function waits for one event among those specified in
 *          @p events to become pending then the event is cleared and returned.
 * @note    One and only one event is served in the function, the one with the
 *          lowest event id. The function is meant to be invoked into a loop
 *          in order to serve all the pending events.<br>
 *          This means that Event Listeners with a lower event identifier have
 *          an higher priority.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The mask of the lowest event id served and cleared.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
eventmask_t chEvtWaitOneTimeout(eventmask_t events, sysinterval_t timeout) {
  thread_t *ctp = nil.current;
  eventmask_t m;

  chSysLock();
  m = ctp->epmask & events;
  if (m == (eventmask_t)0) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();

      return (eventmask_t)0;
    }
    ctp->u1.ewmask = events;
    if (chSchGoSleepTimeoutS(NIL_STATE_WTOREVT, timeout) < MSG_OK) {
      chSysUnlock();

      return (eventmask_t)0;
    }
    m = ctp->epmask & events;
  }
  m ^= m & (m - (eventmask_t)1);
  ctp->epmask &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for any of the specified events.
 * @details The function waits for any event among those specified in
 *          @p mask to become pending then the events are cleared and
 *          returned.
 *
 * @param[in] mask      mask of the event flags that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The mask of the served and cleared events.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, sysinterval_t timeout) {
  thread_t *ctp = nil.current;
  eventmask_t m;

  chSysLock();
  if ((m = (ctp->epmask & mask)) == (eventmask_t)0) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();

      return (eventmask_t)0;
    }
    ctp->u1.ewmask = mask;
    if (chSchGoSleepTimeoutS(NIL_STATE_WTOREVT, timeout) < MSG_OK) {
      chSysUnlock();

      return (eventmask_t)0;
    }
    m = ctp->epmask & mask;
  }
  ctp->epmask &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for all the specified events.
 * @details The function waits for all the events specified in @p mask to
 *          become pending then the events are cleared and returned.
 *
 * @param[in] mask      mask of the event flags that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The mask of the served and cleared events.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
eventmask_t chEvtWaitAllTimeout(eventmask_t mask, sysinterval_t timeout) {
  thread_t *ctp = nil.current;

  chSysLock();
  if ((ctp->epmask & mask) != mask) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();

      return (eventmask_t)0;
    }
    ctp->u1.ewmask = mask;
    if (chSchGoSleepTimeoutS(NIL_STATE_WTANDEVT, timeout) < MSG_OK) {
      chSysUnlock();

      return (eventmask_t)0;
    }
  }
  ctp->epmask &= ~mask;
  chSysUnlock();

  return mask;
}

#endif /* CH_CFG_USE_EVENTS == TRUE */

/** @} */
