/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "rt_test_root.h"

/**
 * @file    rt_test_sequence_010.c
 * @brief   Test Sequence 010 code.
 *
 * @page rt_test_sequence_010 [10] Event Sources and Event Flags
 *
 * File: @ref rt_test_sequence_010.c
 *
 * <h2>Description</h2>
 * This module implements the test sequence for the Events subsystem.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_EVENTS == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_010_001
 * - @subpage rt_test_010_002
 * - @subpage rt_test_010_003
 * - @subpage rt_test_010_004
 * - @subpage rt_test_010_005
 * - @subpage rt_test_010_006
 * - @subpage rt_test_010_007
 * .
 */

#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static EVENTSOURCE_DECL(es1);
static EVENTSOURCE_DECL(es2);

static void h1(eventid_t id) {(void)id;test_emit_token('A');}
static void h2(eventid_t id) {(void)id;test_emit_token('B');}
static void h3(eventid_t id) {(void)id;test_emit_token('C');}
static ROMCONST evhandler_t evhndl[] = {h1, h2, h3};

static THD_FUNCTION(evt_thread3, p) {

  chThdSleepMilliseconds(50);
  chEvtSignal((thread_t *)p, 1);
}

static THD_FUNCTION(evt_thread7, p) {

  (void)p;
  chEvtBroadcast(&es1);
  chThdSleepMilliseconds(50);
  chEvtBroadcast(&es2);
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_010_001 [10.1] Events registration
 *
 * <h2>Description</h2>
 * Two event listeners are registered on an event source and then
 * unregistered in the same order.<br> The test expects that the even
 * source has listeners after the registrations and after the first
 * unregistration, then, after the second unegistration, the test
 * expects no more listeners.
 *
 * <h2>Test Steps</h2>
 * - [10.1.1] An Event Source is initialized.
 * - [10.1.2] Two Event Listeners are registered on the Event Source,
 *   the Event Source is tested to have listeners.
 * - [10.1.3] An Event Listener is unregistered, the Event Source must
 *   still have listeners.
 * - [10.1.4] An Event Listener is unregistered, the Event Source must
 *   not have listeners.
 * .
 */

static void rt_test_010_001_execute(void) {
  event_listener_t el1, el2;

  /* [10.1.1] An Event Source is initialized.*/
  test_set_step(1);
  {
    chEvtObjectInit(&es1);
  }
  test_end_step(1);

  /* [10.1.2] Two Event Listeners are registered on the Event Source,
     the Event Source is tested to have listeners.*/
  test_set_step(2);
  {
    chEvtRegisterMask(&es1, &el1, 1);
    chEvtRegisterMask(&es1, &el2, 2);
    test_assert_lock(chEvtIsListeningI(&es1), "no listener");
  }
  test_end_step(2);

  /* [10.1.3] An Event Listener is unregistered, the Event Source must
     still have listeners.*/
  test_set_step(3);
  {
    chEvtUnregister(&es1, &el1);
    test_assert_lock(chEvtIsListeningI(&es1), "no listener");
  }
  test_end_step(3);

  /* [10.1.4] An Event Listener is unregistered, the Event Source must
     not have listeners.*/
  test_set_step(4);
  {
    chEvtUnregister(&es1, &el2);
    test_assert_lock(!chEvtIsListeningI(&es1), "stuck listener");
  }
  test_end_step(4);
}

static const testcase_t rt_test_010_001 = {
  "Events registration",
  NULL,
  NULL,
  rt_test_010_001_execute
};

/**
 * @page rt_test_010_002 [10.2] Event Flags dispatching
 *
 * <h2>Description</h2>
 * The test dispatches three event flags and verifies that the
 * associated event handlers are invoked in LSb-first order.
 *
 * <h2>Test Steps</h2>
 * - [10.2.1] Three evenf flag bits are raised then chEvtDispatch() is
 *   invoked, the sequence of handlers calls is tested.
 * .
 */

static void rt_test_010_002_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
}

static void rt_test_010_002_execute(void) {

  /* [10.2.1] Three evenf flag bits are raised then chEvtDispatch() is
     invoked, the sequence of handlers calls is tested.*/
  test_set_step(1);
  {
    chEvtDispatch(evhndl, 7);
    test_assert_sequence("ABC", "invalid sequence");
  }
  test_end_step(1);
}

static const testcase_t rt_test_010_002 = {
  "Event Flags dispatching",
  rt_test_010_002_setup,
  NULL,
  rt_test_010_002_execute
};

/**
 * @page rt_test_010_003 [10.3] Events Flags wait using chEvtWaitOne()
 *
 * <h2>Description</h2>
 * Functionality of chEvtWaitOne() is tested under various scenarios.
 *
 * <h2>Test Steps</h2>
 * - [10.3.1] Setting three event flags.
 * - [10.3.2] Calling chEvtWaitOne() three times, each time a single
 *   flag must be returned in order of priority.
 * - [10.3.3] Getting current time and starting a signaler thread, the
 *   thread will set an event flag after 50mS.
 * - [10.3.4] Calling chEvtWaitOne() then verifying that the event has
 *   been received after 50mS and that the event flags mask has been
 *   emptied.
 * .
 */

static void rt_test_010_003_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
}

static void rt_test_010_003_execute(void) {
  eventmask_t m;
  systime_t target_time;

  /* [10.3.1] Setting three event flags.*/
  test_set_step(1);
  {
    chEvtAddEvents(7);
  }
  test_end_step(1);

  /* [10.3.2] Calling chEvtWaitOne() three times, each time a single
     flag must be returned in order of priority.*/
  test_set_step(2);
  {
    m = chEvtWaitOne(ALL_EVENTS);
    test_assert(m == 1, "single event error");
    m = chEvtWaitOne(ALL_EVENTS);
    test_assert(m == 2, "single event error");
    m = chEvtWaitOne(ALL_EVENTS);
    test_assert(m == 4, "single event error");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
  }
  test_end_step(2);

  /* [10.3.3] Getting current time and starting a signaler thread, the
     thread will set an event flag after 50mS.*/
  test_set_step(3);
  {
    target_time = chTimeAddX(test_wait_tick(), TIME_MS2I(50));
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX() - 1,
                                   evt_thread3, chThdGetSelfX());
  }
  test_end_step(3);

  /* [10.3.4] Calling chEvtWaitOne() then verifying that the event has
     been received after 50mS and that the event flags mask has been
     emptied.*/
  test_set_step(4);
  {
    m = chEvtWaitOne(ALL_EVENTS);
    test_assert_time_window(target_time,
                            chTimeAddX(target_time, ALLOWED_DELAY),
                            "out of time window");
    test_assert(m == 1, "event flag error");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
    test_wait_threads();
  }
  test_end_step(4);
}

static const testcase_t rt_test_010_003 = {
  "Events Flags wait using chEvtWaitOne()",
  rt_test_010_003_setup,
  NULL,
  rt_test_010_003_execute
};

/**
 * @page rt_test_010_004 [10.4] Events Flags wait using chEvtWaitAny()
 *
 * <h2>Description</h2>
 * Functionality of chEvtWaitAny() is tested under various scenarios.
 *
 * <h2>Test Steps</h2>
 * - [10.4.1] Setting two, non contiguous, event flags.
 * - [10.4.2] Calling chEvtWaitAny() one time, the two flags must be
 *   returned.
 * - [10.4.3] Getting current time and starting a signaler thread, the
 *   thread will set an event flag after 50mS.
 * - [10.4.4] Calling chEvtWaitAny() then verifying that the event has
 *   been received after 50mS and that the event flags mask has been
 *   emptied.
 * .
 */

static void rt_test_010_004_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
}

static void rt_test_010_004_execute(void) {
  eventmask_t m;
  systime_t target_time;

  /* [10.4.1] Setting two, non contiguous, event flags.*/
  test_set_step(1);
  {
    chEvtAddEvents(5);
  }
  test_end_step(1);

  /* [10.4.2] Calling chEvtWaitAny() one time, the two flags must be
     returned.*/
  test_set_step(2);
  {
    m = chEvtWaitAny(ALL_EVENTS);
    test_assert(m == 5, "unexpected pending bit");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
  }
  test_end_step(2);

  /* [10.4.3] Getting current time and starting a signaler thread, the
     thread will set an event flag after 50mS.*/
  test_set_step(3);
  {
    target_time = chTimeAddX(test_wait_tick(), TIME_MS2I(50));
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX() - 1,
                                   evt_thread3, chThdGetSelfX());
  }
  test_end_step(3);

  /* [10.4.4] Calling chEvtWaitAny() then verifying that the event has
     been received after 50mS and that the event flags mask has been
     emptied.*/
  test_set_step(4);
  {
    m = chEvtWaitAny(ALL_EVENTS);
    test_assert_time_window(target_time,
                            chTimeAddX(target_time, ALLOWED_DELAY),
                            "out of time window");
    test_assert(m == 1, "event flag error");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
    test_wait_threads();
  }
  test_end_step(4);
}

static const testcase_t rt_test_010_004 = {
  "Events Flags wait using chEvtWaitAny()",
  rt_test_010_004_setup,
  NULL,
  rt_test_010_004_execute
};

/**
 * @page rt_test_010_005 [10.5] Events Flags wait using chEvtWaitAll()
 *
 * <h2>Description</h2>
 * Functionality of chEvtWaitAll() is tested under various scenarios.
 *
 * <h2>Test Steps</h2>
 * - [10.5.1] Setting two, non contiguous, event flags.
 * - [10.5.2] Calling chEvtWaitAll() one time, the two flags must be
 *   returned.
 * - [10.5.3] Setting one event flag.
 * - [10.5.4] Getting current time and starting a signaler thread, the
 *   thread will set another event flag after 50mS.
 * - [10.5.5] Calling chEvtWaitAll() then verifying that both event
 *   flags have been received after 50mS and that the event flags mask
 *   has been emptied.
 * .
 */

static void rt_test_010_005_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
}

static void rt_test_010_005_execute(void) {
  eventmask_t m;
  systime_t target_time;

  /* [10.5.1] Setting two, non contiguous, event flags.*/
  test_set_step(1);
  {
    chEvtAddEvents(5);
  }
  test_end_step(1);

  /* [10.5.2] Calling chEvtWaitAll() one time, the two flags must be
     returned.*/
  test_set_step(2);
  {
    m = chEvtWaitAll(5);
    test_assert(m == 5, "unexpected pending bit");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
  }
  test_end_step(2);

  /* [10.5.3] Setting one event flag.*/
  test_set_step(3);
  {
    chEvtAddEvents(4);
  }
  test_end_step(3);

  /* [10.5.4] Getting current time and starting a signaler thread, the
     thread will set another event flag after 50mS.*/
  test_set_step(4);
  {
    target_time = chTimeAddX(test_wait_tick(), TIME_MS2I(50));
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX() - 1,
                                   evt_thread3, chThdGetSelfX());
  }
  test_end_step(4);

  /* [10.5.5] Calling chEvtWaitAll() then verifying that both event
     flags have been received after 50mS and that the event flags mask
     has been emptied.*/
  test_set_step(5);
  {
    m = chEvtWaitAll(5);
    test_assert_time_window(target_time,
                            chTimeAddX(target_time, ALLOWED_DELAY),
                            "out of time window");
    test_assert(m == 5, "event flags error");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
    test_wait_threads();
  }
  test_end_step(5);
}

static const testcase_t rt_test_010_005 = {
  "Events Flags wait using chEvtWaitAll()",
  rt_test_010_005_setup,
  NULL,
  rt_test_010_005_execute
};

#if (CH_CFG_USE_EVENTS_TIMEOUT == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_010_006 [10.6] Events Flags wait timeouts
 *
 * <h2>Description</h2>
 * Timeout functionality is tested for chEvtWaitOneTimeout(),
 * chEvtWaitAnyTimeout() and chEvtWaitAllTimeout().
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_EVENTS_TIMEOUT == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [10.6.1] The functions are invoked first with TIME_IMMEDIATE
 *   timeout, the timeout condition is tested.
 * - [10.6.2] The functions are invoked first with a 50mS timeout, the
 *   timeout condition is tested.
 * .
 */

static void rt_test_010_006_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
}

static void rt_test_010_006_execute(void) {
  eventmask_t m;

  /* [10.6.1] The functions are invoked first with TIME_IMMEDIATE
     timeout, the timeout condition is tested.*/
  test_set_step(1);
  {
    m = chEvtWaitOneTimeout(ALL_EVENTS, TIME_IMMEDIATE);
    test_assert(m == 0, "spurious event");
    m = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_IMMEDIATE);
    test_assert(m == 0, "spurious event");
    m = chEvtWaitAllTimeout(ALL_EVENTS, TIME_IMMEDIATE);
    test_assert(m == 0, "spurious event");
  }
  test_end_step(1);

  /* [10.6.2] The functions are invoked first with a 50mS timeout, the
     timeout condition is tested.*/
  test_set_step(2);
  {
    m = chEvtWaitOneTimeout(ALL_EVENTS, TIME_MS2I(50));
    test_assert(m == 0, "spurious event");
    m = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(50));
    test_assert(m == 0, "spurious event");
    m = chEvtWaitAllTimeout(ALL_EVENTS, TIME_MS2I(50));
    test_assert(m == 0, "spurious event");
  }
  test_end_step(2);
}

static const testcase_t rt_test_010_006 = {
  "Events Flags wait timeouts",
  rt_test_010_006_setup,
  NULL,
  rt_test_010_006_execute
};
#endif /* CH_CFG_USE_EVENTS_TIMEOUT == TRUE */

/**
 * @page rt_test_010_007 [10.7] Broadcasting using chEvtBroadcast()
 *
 * <h2>Description</h2>
 * Functionality of chEvtBroadcast() is tested.
 *
 * <h2>Test Steps</h2>
 * - [10.7.1] Registering on two event sources associating them with
 *   flags 1 and 4.
 * - [10.7.2] Getting current time and starting a broadcaster thread,
 *   the thread broadcast the first Event Source immediately and the
 *   other after 50mS.
 * - [10.7.3] Calling chEvtWaitAll() then verifying that both event
 *   flags have been received after 50mS and that the event flags mask
 *   has been emptied.
 * - [10.7.4] Unregistering from the Event Sources.
 * .
 */

static void rt_test_010_007_setup(void) {
  chEvtGetAndClearEvents(ALL_EVENTS);
  chEvtObjectInit(&es1);
  chEvtObjectInit(&es2);
}

static void rt_test_010_007_execute(void) {
  eventmask_t m;
  event_listener_t el1, el2;
  systime_t target_time;

  /* [10.7.1] Registering on two event sources associating them with
     flags 1 and 4.*/
  test_set_step(1);
  {
    chEvtRegisterMask(&es1, &el1, 1);
    chEvtRegisterMask(&es2, &el2, 4);
  }
  test_end_step(1);

  /* [10.7.2] Getting current time and starting a broadcaster thread,
     the thread broadcast the first Event Source immediately and the
     other after 50mS.*/
  test_set_step(2);
  {
    target_time = chTimeAddX(test_wait_tick(), TIME_MS2I(50));
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX() - 1,
                                   evt_thread7, "A");
  }
  test_end_step(2);

  /* [10.7.3] Calling chEvtWaitAll() then verifying that both event
     flags have been received after 50mS and that the event flags mask
     has been emptied.*/
  test_set_step(3);
  {
    m = chEvtWaitAll(5);
    test_assert_time_window(target_time,
                            chTimeAddX(target_time, ALLOWED_DELAY),
                            "out of time window");
    m = chEvtGetAndClearEvents(ALL_EVENTS);
    test_assert(m == 0, "stuck event");
    test_wait_threads();
  }
  test_end_step(3);

  /* [10.7.4] Unregistering from the Event Sources.*/
  test_set_step(4);
  {
    chEvtUnregister(&es1, &el1);
    chEvtUnregister(&es2, &el2);
    test_assert(!chEvtIsListeningI(&es1), "stuck listener");
    test_assert(!chEvtIsListeningI(&es2), "stuck listener");
  }
  test_end_step(4);
}

static const testcase_t rt_test_010_007 = {
  "Broadcasting using chEvtBroadcast()",
  rt_test_010_007_setup,
  NULL,
  rt_test_010_007_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_010_array[] = {
  &rt_test_010_001,
  &rt_test_010_002,
  &rt_test_010_003,
  &rt_test_010_004,
  &rt_test_010_005,
#if (CH_CFG_USE_EVENTS_TIMEOUT == TRUE) || defined(__DOXYGEN__)
  &rt_test_010_006,
#endif
  &rt_test_010_007,
  NULL
};

/**
 * @brief   Event Sources and Event Flags.
 */
const testsequence_t rt_test_sequence_010 = {
  "Event Sources and Event Flags",
  rt_test_sequence_010_array
};

#endif /* CH_CFG_USE_EVENTS == TRUE */
