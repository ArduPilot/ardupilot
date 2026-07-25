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
/*
   Concepts and parts of this file have been contributed by Scott (skute).
 */

/**
 * @file    rt/src/chevents.c
 * @brief   Events code.
 *
 * @addtogroup events
 * @details Event Flags, Event Sources and Event Listeners.
 *          <h2>Operation mode</h2>
 *          Each thread has a mask of pending events inside its
 *          @p thread_t structure.
 *          Operations defined for events:
 *          - <b>Wait</b>, the invoking thread goes to sleep until a certain
 *            AND/OR combination of events are signaled.
 *          - <b>Clear</b>, a mask of events is cleared from the pending
 *            events, the cleared events mask is returned (only the
 *            events that were actually pending and then cleared).
 *          - <b>Signal</b>, an events mask is directly ORed to the mask of
 *            the signaled thread.
 *          - <b>Broadcast</b>, each thread registered on an Event Source is
 *            signaled with the events specified in its Event Listener.
 *          - <b>Dispatch</b>, an events mask is scanned and for each bit set
 *            to one an associated handler function is invoked. Bit masks are
 *            scanned from bit zero upward.
 *          .
 *          An Event Source is a special object that can be "broadcasted" by
 *          a thread or an interrupt service routine. Broadcasting an Event
 *          Source has the effect that all the threads registered on the
 *          Event Source will be signaled with an events mask.<br>
 *          An unlimited number of Event Sources can exists in a system and
 *          each thread can be listening on an unlimited number of
 *          them.
 * @pre     In order to use the Events APIs the @p CH_CFG_USE_EVENTS option
 *          must be enabled in @p chconf.h.
 * @post    Enabling events requires 1-4 (depending on the architecture)
 *          extra bytes in the @p thread_t structure.
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
 * @iclass
 */
void chEvtRegisterMaskWithFlagsI(event_source_t *esp,
                                 event_listener_t *elp,
                                 eventmask_t events,
                                 eventflags_t wflags) {
  thread_t *currtp = chThdGetSelfX();

  chDbgCheckClassI();
  chDbgCheck((esp != NULL) && (elp != NULL));

  elp->next     = esp->next;
  esp->next     = elp;
  elp->listener = currtp;
  elp->events   = events;
  elp->flags    = (eventflags_t)0;
  elp->wflags   = wflags;
}

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

  chSysLock();
  chEvtRegisterMaskWithFlagsI(esp, elp, events, wflags);
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
  thread_t *currtp = chThdGetSelfX();
  eventmask_t m;

  chDbgCheckClassI();

  m = currtp->epending & events;
  currtp->epending &= ~events;

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

  chDbgCheckClassI();
  chDbgCheck(elp != NULL);

  flags = elp->flags;
  elp->flags = (eventflags_t)0;

  return flags & elp->wflags;
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

  chDbgCheck(elp != NULL);

  chSysLock();
  flags = elp->flags;
  elp->flags = (eventflags_t)0;
  chSysUnlock();

  return flags & elp->wflags;
}

/**
 * @brief   Adds a set of event flags directly to the specified @p thread_t.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] tp        the thread to be signaled
 * @param[in] events    the events set to be ORed
 *
 * @iclass
 */
void chEvtSignalI(thread_t *tp, eventmask_t events) {

  chDbgCheckClassI();
  chDbgCheck(tp != NULL);

  tp->epending |= events;
  /* Test on the AND/OR conditions wait states.*/
  if (((tp->state == CH_STATE_WTOREVT) &&
       ((tp->epending & tp->u.ewmask) != (eventmask_t)0)) ||
      ((tp->state == CH_STATE_WTANDEVT) &&
       ((tp->epending & tp->u.ewmask) == tp->u.ewmask))) {
    tp->u.rdymsg = MSG_OK;
    (void) chSchReadyI(tp);
  }
}

/**
 * @brief   Adds a set of event flags directly to the specified @p thread_t.
 *
 * @param[in] tp        the thread to be signaled
 * @param[in] events    the events set to be ORed
 *
 * @api
 */
void chEvtSignal(thread_t *tp, eventmask_t events) {

  chDbgCheck(tp != NULL);

  chSysLock();
  chEvtSignalI(tp, events);
  chSchRescheduleS();
  chSysUnlock();
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

#if (CH_CFG_OPTIMIZE_SPEED == TRUE) ||                                      \
    (CH_CFG_USE_EVENTS_TIMEOUT == FALSE) ||                                 \
    defined(__DOXYGEN__)
/**
 * @brief   Waits for exactly one of the specified events.
 * @details The function waits for one event among those specified in
 *          @p events to become pending then the event is cleared and returned.
 * @note    One and only one event is served in the function, the one with the
 *          lowest event id. The function is meant to be invoked into a loop in
 *          order to serve all the pending events.<br>
 *          This means that Event Listeners with a lower event identifier have
 *          an higher priority.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @return              The mask of the lowest event id served and cleared.
 *
 * @api
 */
eventmask_t chEvtWaitOne(eventmask_t events) {
  thread_t *currtp = chThdGetSelfX();
  eventmask_t m;

  chSysLock();
  m = currtp->epending & events;
  if (m == (eventmask_t)0) {
    currtp->u.ewmask = events;
    chSchGoSleepS(CH_STATE_WTOREVT);
    m = currtp->epending & events;
  }
  m ^= m & (m - (eventmask_t)1);
  currtp->epending &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for any of the specified events.
 * @details The function waits for any event among those specified in
 *          @p events to become pending then the events are cleared and
 *          returned.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @return              The mask of the served and cleared events.
 *
 * @api
 */
eventmask_t chEvtWaitAny(eventmask_t events) {
  thread_t *currtp = chThdGetSelfX();
  eventmask_t m;

  chSysLock();
  m = currtp->epending & events;
  if (m == (eventmask_t)0) {
    currtp->u.ewmask = events;
    chSchGoSleepS(CH_STATE_WTOREVT);
    m = currtp->epending & events;
  }
  currtp->epending &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for all the specified events.
 * @details The function waits for all the events specified in @p events to
 *          become pending then the events are cleared and returned.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS requires all the events
 * @return              The mask of the served and cleared events.
 *
 * @api
 */
eventmask_t chEvtWaitAll(eventmask_t events) {
  thread_t *currtp = chThdGetSelfX();

  chSysLock();
  if ((currtp->epending & events) != events) {
    currtp->u.ewmask = events;
    chSchGoSleepS(CH_STATE_WTANDEVT);
  }
  currtp->epending &= ~events;
  chSysUnlock();

  return events;
}
#endif /* CH_CFG_OPTIMIZE_SPEED || !CH_CFG_USE_EVENTS_TIMEOUT */

#if (CH_CFG_USE_EVENTS_TIMEOUT == TRUE) || defined(__DOXYGEN__)
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
  thread_t *currtp = chThdGetSelfX();
  eventmask_t m;

  chSysLock();
  m = currtp->epending & events;
  if (m == (eventmask_t)0) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();
      return (eventmask_t)0;
    }
    currtp->u.ewmask = events;
    if (chSchGoSleepTimeoutS(CH_STATE_WTOREVT, timeout) < MSG_OK) {
      chSysUnlock();
      return (eventmask_t)0;
    }
    m = currtp->epending & events;
  }
  m ^= m & (m - (eventmask_t)1);
  currtp->epending &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for any of the specified events.
 * @details The function waits for any event among those specified in
 *          @p events to become pending then the events are cleared and
 *          returned.
 *
 * @param[in] events    events that the function should wait
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
eventmask_t chEvtWaitAnyTimeout(eventmask_t events, sysinterval_t timeout) {
  thread_t *currtp = chThdGetSelfX();
  eventmask_t m;

  chSysLock();
  m = currtp->epending & events;
  if (m == (eventmask_t)0) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();
      return (eventmask_t)0;
    }
    currtp->u.ewmask = events;
    if (chSchGoSleepTimeoutS(CH_STATE_WTOREVT, timeout) < MSG_OK) {
      chSysUnlock();
      return (eventmask_t)0;
    }
    m = currtp->epending & events;
  }
  currtp->epending &= ~m;
  chSysUnlock();

  return m;
}

/**
 * @brief   Waits for all the specified events.
 * @details The function waits for all the events specified in @p events to
 *          become pending then the events are cleared and returned.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS requires all the events
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
eventmask_t chEvtWaitAllTimeout(eventmask_t events, sysinterval_t timeout) {
  thread_t *currtp = chThdGetSelfX();

  chSysLock();
  if ((currtp->epending & events) != events) {
    if (TIME_IMMEDIATE == timeout) {
      chSysUnlock();
      return (eventmask_t)0;
    }
    currtp->u.ewmask = events;
    if (chSchGoSleepTimeoutS(CH_STATE_WTANDEVT, timeout) < MSG_OK) {
      chSysUnlock();
      return (eventmask_t)0;
    }
  }
  currtp->epending &= ~events;
  chSysUnlock();

  return events;
}
#endif /* CH_CFG_USE_EVENTS_TIMEOUT == TRUE */

#endif /* CH_CFG_USE_EVENTS == TRUE */

/** @} */
