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
 * @file    rt/include/chevents.h
 * @brief   Events macros and structures.
 *
 * @addtogroup events
 * @{
 */

#ifndef CHEVENTS_H
#define CHEVENTS_H

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
#define __EVENTSOURCE_DATA(name) {(event_listener_t *)(&name)}

/**
 * @brief   Static event source initializer.
 * @details Statically initialized event sources require no explicit
 *          initialization using @p chEvtInit().
 *
 * @param name          the name of the event source variable
 */
#define EVENTSOURCE_DECL(name) event_source_t name = __EVENTSOURCE_DATA(name)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chEvtRegisterMaskWithFlagsI(event_source_t *esp,
                                   event_listener_t *elp,
                                   eventmask_t events,
                                   eventflags_t wflags);
  void chEvtRegisterMaskWithFlags(event_source_t *esp,
                                  event_listener_t *elp,
                                  eventmask_t events,
                                  eventflags_t wflags);
  void chEvtUnregister(event_source_t *esp, event_listener_t *elp);
  eventmask_t chEvtGetAndClearEventsI(eventmask_t events);
  eventmask_t chEvtGetAndClearEvents(eventmask_t events);
  eventmask_t chEvtAddEvents(eventmask_t events);
  eventflags_t chEvtGetAndClearFlagsI(event_listener_t *elp);
  eventflags_t chEvtGetAndClearFlags(event_listener_t *elp);
  void chEvtSignal(thread_t *tp, eventmask_t events);
  void chEvtSignalI(thread_t *tp, eventmask_t events);
  void chEvtBroadcastFlags(event_source_t *esp, eventflags_t flags);
  void chEvtBroadcastFlagsI(event_source_t *esp, eventflags_t flags);
  void chEvtDispatch(const evhandler_t *handlers, eventmask_t events);
#if (CH_CFG_OPTIMIZE_SPEED == TRUE) || (CH_CFG_USE_EVENTS_TIMEOUT == FALSE)
  eventmask_t chEvtWaitOne(eventmask_t events);
  eventmask_t chEvtWaitAny(eventmask_t events);
  eventmask_t chEvtWaitAll(eventmask_t events);
#endif
#if CH_CFG_USE_EVENTS_TIMEOUT == TRUE
  eventmask_t chEvtWaitOneTimeout(eventmask_t events, sysinterval_t timeout);
  eventmask_t chEvtWaitAnyTimeout(eventmask_t events, sysinterval_t timeout);
  eventmask_t chEvtWaitAllTimeout(eventmask_t events, sysinterval_t timeout);
#endif
#ifdef __cplusplus
}
#endif

#if (CH_CFG_OPTIMIZE_SPEED == FALSE) && (CH_CFG_USE_EVENTS_TIMEOUT == TRUE)
#define chEvtWaitOne(mask) chEvtWaitOneTimeout(mask, TIME_INFINITE)
#define chEvtWaitAny(mask) chEvtWaitAnyTimeout(mask, TIME_INFINITE)
#define chEvtWaitAll(mask) chEvtWaitAllTimeout(mask, TIME_INFINITE)
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes an Event Source.
 * @note    This function can be invoked before the kernel is initialized
 *          because it just prepares a @p event_source_t structure.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 *
 * @init
 */
static inline void chEvtObjectInit(event_source_t *esp) {

  esp->next = (event_listener_t *)esp;
}

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
static inline void chEvtRegisterMask(event_source_t *esp,
                                     event_listener_t *elp,
                                     eventmask_t events) {

  chEvtRegisterMaskWithFlags(esp, elp, events, (eventflags_t)-1);
}

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
static inline void chEvtRegister(event_source_t *esp,
                                 event_listener_t *elp,
                                 eventid_t event) {

  chEvtRegisterMask(esp, elp, EVENT_MASK(event));
}

/**
 * @brief   Verifies if there is at least one @p event_listener_t registered.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 * @return              The event source status.
 *
 * @iclass
 */
static inline bool chEvtIsListeningI(event_source_t *esp) {

  return (bool)(esp != (event_source_t *)esp->next);
}

/**
 * @brief   Signals all the Event Listeners registered on the specified Event
 *          Source.
 *
 * @param[in] esp       pointer to the @p event_source_t structure
 *
 * @api
 */
static inline void chEvtBroadcast(event_source_t *esp) {

  chEvtBroadcastFlags(esp, (eventflags_t)0);
}

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
static inline void chEvtBroadcastI(event_source_t *esp) {

  chEvtBroadcastFlagsI(esp, (eventflags_t)0);
}

/**
 * @brief   Adds (OR) a set of events to the current thread, this is
 *          @b much faster than using @p chEvtBroadcast() or @p chEvtSignal().
 *
 * @param[in] events    the events to be added
 * @return              The mask of currently pending events.
 *
 * @iclass
 */
static inline eventmask_t chEvtAddEventsI(eventmask_t events) {

  return __sch_get_currthread()->epending |= events;
}

/**
 * @brief   Returns the events mask.
 * @details The pending events mask is returned but not altered in any way.
 *
 * @return              The pending events mask.
 *
 * @api
 */
static inline eventmask_t chEvtGetEventsX(void) {

  return __sch_get_currthread()->epending;
}

#endif /* CH_CFG_USE_EVENTS == TRUE */

#endif /* CHEVENTS_H */

/** @} */
