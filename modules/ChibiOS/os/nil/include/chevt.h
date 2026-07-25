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
 * @file    nil/include/chevt.h
 * @brief   Nil RTOS events header file.
 *
 * @addtogroup NIL_EVENTS
 * @{
 */

#ifndef CHEVT_H
#define CHEVT_H

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct event_listener event_listener_t;

/**
 * @brief   Event Listener structure.
 */
struct event_listener {
  event_listener_t      *next;          /**< @brief Next Event Listener
                                                    registered on the event
                                                    source.                 */
  thread_t              *listener;      /**< @brief Thread interested in the
                                                    event source.           */
  eventmask_t           events;         /**< @brief Events to be set in
                                                    the listening thread.   */
  eventflags_t          flags;          /**< @brief Flags added to the listener
                                                    by the event source.    */
  eventflags_t          wflags;         /**< @brief Flags that this listener
                                                    interested in.          */
};

/**
 * @brief   Event Source structure.
 */
typedef struct event_source {
  event_listener_t      *next;          /**< @brief First Event Listener
                                                    registered on the Event
                                                    Source.                 */
} event_source_t;

/**
 * @brief   Event Handler callback function.
 */
typedef void (*evhandler_t)(eventid_t id);

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   All events allowed mask.
 */
#define ALL_EVENTS      ((eventmask_t)-1)

/**
 * @brief   Returns an event mask from an event identifier.
 */
#define EVENT_MASK(eid) ((eventmask_t)1 << (eventmask_t)(eid))

/**
 * @brief   Data part of a static event source initializer.
 * @details This macro should be used when statically initializing an event
 *          source that is part of a bigger structure.
 * @param name          the name of the event source variable
 */
#define _EVENTSOURCE_DATA(name) {(event_listener_t *)(&name)}

/**
 * @brief   Static event source initializer.
 * @details Statically initialized event sources require no explicit
 *          initialization using @p chEvtInit().
 *
 * @param name          the name of the event source variable
 */
#define EVENTSOURCE_DECL(name) event_source_t name = _EVENTSOURCE_DATA(name)

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Initializes an Event Source.
 * @note    This function can be invoked before the kernel is initialized
 *          because it just prepares a @p event_source_t structure.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 *
 * @init
 */
#define chEvtObjectInit(esp) do {                                           \
  (esp)->next = (event_listener_t *)(esp);                                  \
} while (0)

/**
 * @brief   Registers an Event Listener on an Event Source.
 * @details Once a thread has registered as listener on an event source it
 *          will be notified of all events broadcasted there.
 * @note    Multiple Event Listeners can specify the same bits to be ORed to
 *          different threads.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 * @param[out] elp      pointer to the @p event_listener_t structure
 * @param[in] events    the mask of events to be ORed to the thread when
 *                      the event source is broadcasted
 *
 * @api
 */
#define chEvtRegisterMask(esp, elp, events)                                 \
  chEvtRegisterMaskWithFlags(esp, elp, events, (eventflags_t)-1)

/**
 * @brief   Registers an Event Listener on an Event Source.
 * @note    Multiple Event Listeners can use the same event identifier, the
 *          listener will share the callback function.
 *
 * @param[in] esp       pointer to the  @p event_source_t structure
 * @param[out] elp      pointer to the @p event_listener_t structure
 * @param[in] event     numeric identifier assigned to the Event Listener.
 *                      The value must range between zero and the size, in bit,
 *                      of the @p eventmask_t type minus one.
 *
 * @api
 */
#define chEvtRegister(esp, elp, event)                                      \
  chEvtRegisterMask(esp, elp, EVENT_MASK(event))

/**
 * @brief   Verifies if there is at least one @p event_listener_t registered.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 * @return              The event source status.
 *
 * @iclass
 */
#define chEvtIsListeningI(esp) (bool)((esp) != (event_source_t *)(esp)->next)

/**
 * @brief   Signals all the Event Listeners registered on the specified Event
 *          Source.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 *
 * @api
 */
#define chEvtBroadcast(esp) chEvtBroadcastFlags(esp, (eventflags_t)0)

/**
 * @brief   Signals all the Event Listeners registered on the specified Event
 *          Source.
 * @post    This function does not reschedule so a call to a rescheduling
 *          function must be performed before unlocking the kernel. Note that
 *          interrupt handlers always reschedule on exit so an explicit
 *          reschedule must not be performed in ISRs.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 *
 * @iclass
 */
#define chEvtBroadcastI(esp) chEvtBroadcastFlagsI(esp, (eventflags_t)0)

/**
 * @brief   Adds (OR) a set of events to the current thread, this is
 *          @b much faster than using @p chEvtBroadcast() or @p chEvtSignal().
 *
 * @param[in] events    the events to be added
 * @return              The mask of currently pending events.
 *
 * @iclass
 */
#define chEvtAddEventsI(events) (nil.current->epmask |= events)

/**
 * @brief   Returns the events mask.
 * @details The pending events mask is returned but not altered in any way.
 *
 * @return              The pending events mask.
 *
 * @api
 */
#define chEvtGetEventsX(void) (nil.current->epmask)

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
 * @return              The mask of the lowest event id served and cleared.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
#define chEvtWaitOne(events) chEvtWaitOneTimeout(events, TIME_INFINITE)

/**
 * @brief   Waits for any of the specified events.
 * @details The function waits for any event among those specified in
 *          @p mask to become pending then the events are cleared and
 *          returned.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @return              The mask of the served and cleared events.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
#define chEvtWaitAny(events) chEvtWaitAnyTimeout(events, TIME_INFINITE)

/**
 * @brief   Waits for all the specified events.
 * @details The function waits for all the events specified in @p mask to
 *          become pending then the events are cleared and returned.
 *
 * @param[in] events    events that the function should wait
 *                      for, @p ALL_EVENTS enables all the events
 * @return              The mask of the served and cleared events.
 * @retval 0            if the operation has timed out.
 *
 * @api
 */
#define chEvtWaitAll(events) chEvtWaitAllTimeout(events, TIME_INFINITE)

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chEvtRegisterMaskWithFlags(event_source_t *esp,
                                  event_listener_t *elp,
                                  eventmask_t events,
                                  eventflags_t wflags);
  void chEvtUnregister(event_source_t *esp, event_listener_t *elp);
  eventmask_t chEvtGetAndClearEventsI(eventmask_t events);
  eventmask_t chEvtGetAndClearEvents(eventmask_t events);
  eventmask_t chEvtAddEvents(eventmask_t events);
  eventflags_t chEvtGetAndClearFlags(event_listener_t *elp);
  eventflags_t chEvtGetAndClearFlagsI(event_listener_t *elp);
  void chEvtSignal(thread_t *tp, eventmask_t events);
  void chEvtSignalI(thread_t *tp, eventmask_t events);
  void chEvtBroadcastFlags(event_source_t *esp, eventflags_t flags);
  void chEvtBroadcastFlagsI(event_source_t *esp, eventflags_t flags);
  void chEvtDispatch(const evhandler_t *handlers, eventmask_t events);
  eventmask_t chEvtWaitOneTimeout(eventmask_t events, sysinterval_t timeout);
  eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, sysinterval_t timeout);
  eventmask_t chEvtWaitAllTimeout(eventmask_t mask, sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* CH_CFG_USE_EVENTS == TRUE */

#endif /* CHEVT_H */

/** @} */
