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
#include "nil_test_root.h"

/**
 * @file    nil_test_sequence_008.c
 * @brief   Test Sequence 008 code.
 *
 * @page nil_test_sequence_008 [8] Benchmarks
 *
 * File: @ref nil_test_sequence_008.c
 *
 * <h2>Description</h2>
 * This module implements a series of system benchmarks. The benchmarks
 * are useful as a stress test and as a reference when comparing
 * ChibiOS/RT with similar systems.<br> Objective of the test sequence
 * is to provide a performance index for the most critical system
 * subsystems. The performance numbers allow to discover performance
 * regressions between successive ChibiOS/RT releases.
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_008_001
 * - @subpage nil_test_008_002
 * - @subpage nil_test_008_003
 * - @subpage nil_test_008_004
 * - @subpage nil_test_008_005
 * - @subpage nil_test_008_006
 * - @subpage nil_test_008_007
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#if CH_CFG_USE_SEMAPHORES || defined(__DOXYGEN__)
static semaphore_t sem1;
#endif
#if CH_CFG_USE_MUTEXES || defined(__DOXYGEN__)
static mutex_t mtx1;
#endif

#if CH_CFG_USE_MESSAGES
static THD_FUNCTION(bmk_thread1, p) {
  thread_t *tp;
  msg_t msg;

  (void)p;
  do {
    tp = chMsgWait();
    msg = chMsgGet(tp);
    chMsgRelease(tp, msg);
  } while (msg);
}

NOINLINE static unsigned int msg_loop_test(thread_t *tp) {
  systime_t start, end;

  uint32_t n = 0;
  start = test_wait_tick();
  end = chTimeAddX(start, TIME_MS2I(1000));
  do {
    (void)chMsgSend(tp, 1);
    n++;
#if defined(SIMULATOR)
    _sim_check_for_interrupts();
#endif
  } while (chVTIsSystemTimeWithinX(start, end));
  (void)chMsgSend(tp, 0);
  return n;
}
#endif

static THD_FUNCTION(bmk_thread3, p) {

  chThdExit((msg_t)p);
}

static THD_FUNCTION(bmk_thread4, p) {
  msg_t msg;
  thread_t *self = chThdGetSelfX();

  (void)p;
  chSysLock();
  do {
    chSchGoSleepS(NIL_STATE_SUSPENDED);
    msg = self->u1.msg;
  } while (msg == MSG_OK);
  chSysUnlock();
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
/**
 * @page nil_test_008_001 [8.1] Messages performance #1
 *
 * <h2>Description</h2>
 * A message server thread is created with a lower priority than the
 * client thread, the messages throughput per second is measured and
 * the result printed on the output log.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MESSAGES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.1.1] The messenger thread is started at a lower priority than
 *   the current thread.
 * - [8.1.2] The number of messages exchanged is counted in a one
 *   second time window.
 * - [8.1.3] Score is printed.
 * .
 */

static void nil_test_008_001_execute(void) {
  uint32_t n;
  thread_t *tp;

  /* [8.1.1] The messenger thread is started at a lower priority than
     the current thread.*/
  test_set_step(1);
  {
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() + 1,
      .funcp = bmk_thread1,
      .arg   = NULL
    };
    tp = chThdCreate(&td);
  }
  test_end_step(1);

  /* [8.1.2] The number of messages exchanged is counted in a one
     second time window.*/
  test_set_step(2);
  {
    n = msg_loop_test(tp);
    chThdWait(tp);
  }
  test_end_step(2);

  /* [8.1.3] Score is printed.*/
  test_set_step(3);
  {
    test_print("--- Score : ");
    test_printn(n);
    test_print(" msgs/S, ");
    test_printn(n << 1);
    test_println(" ctxswc/S");
  }
  test_end_step(3);
}

static const testcase_t nil_test_008_001 = {
  "Messages performance #1",
  NULL,
  NULL,
  nil_test_008_001_execute
};
#endif /* CH_CFG_USE_MESSAGES == TRUE */

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
/**
 * @page nil_test_008_002 [8.2] Messages performance #2
 *
 * <h2>Description</h2>
 * A message server thread is created with an higher priority than the
 * client thread, the messages throughput per second is measured and
 * the result printed on the output log.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MESSAGES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.2.1] The messenger thread is started at an higher priority than
 *   the current thread.
 * - [8.2.2] The number of messages exchanged is counted in a one
 *   second time window.
 * - [8.2.3] Score is printed.
 * .
 */

static void nil_test_008_002_execute(void) {
  uint32_t n;
  thread_t *tp;

  /* [8.2.1] The messenger thread is started at an higher priority than
     the current thread.*/
  test_set_step(1);
  {
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() - 1,
      .funcp = bmk_thread1,
      .arg   = NULL
    };
    tp = chThdCreate(&td);
  }
  test_end_step(1);

  /* [8.2.2] The number of messages exchanged is counted in a one
     second time window.*/
  test_set_step(2);
  {
    n = msg_loop_test(tp);
    chThdWait(tp);
  }
  test_end_step(2);

  /* [8.2.3] Score is printed.*/
  test_set_step(3);
  {
    test_print("--- Score : ");
    test_printn(n);
    test_print(" msgs/S, ");
    test_printn(n << 1);
    test_println(" ctxswc/S");
  }
  test_end_step(3);
}

static const testcase_t nil_test_008_002 = {
  "Messages performance #2",
  NULL,
  NULL,
  nil_test_008_002_execute
};
#endif /* CH_CFG_USE_MESSAGES == TRUE */

/**
 * @page nil_test_008_003 [8.3] Context Switch performance
 *
 * <h2>Description</h2>
 * A thread is created that just performs a @p chSchGoSleepS() into a
 * loop, the thread is awakened as fast is possible by the tester
 * thread.<br> The Context Switch performance is calculated by
 * measuring the number of iterations after a second of continuous
 * operations.
 *
 * <h2>Test Steps</h2>
 * - [8.3.1] Starting the target thread at an higher priority level.
 * - [8.3.2] Waking up the thread as fast as possible in a one second
 *   time window.
 * - [8.3.3] Stopping the target thread.
 * - [8.3.4] Score is printed.
 * .
 */

static void nil_test_008_003_execute(void) {
  thread_t *tp;
  uint32_t n;

  /* [8.3.1] Starting the target thread at an higher priority level.*/
  test_set_step(1);
  {
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() - 1,
      .funcp = bmk_thread4,
      .arg   = NULL
    };
    tp = chThdCreate(&td);
  }
  test_end_step(1);

  /* [8.3.2] Waking up the thread as fast as possible in a one second
     time window.*/
  test_set_step(2);
  {
    systime_t start, end;

    n = 0;
    start = test_wait_tick();
    end = chTimeAddX(start, TIME_MS2I(1000));
    do {
      chSysLock();
      chSchWakeupS(tp, MSG_OK);
      chSchWakeupS(tp, MSG_OK);
      chSchWakeupS(tp, MSG_OK);
      chSchWakeupS(tp, MSG_OK);
      chSysUnlock();
      n += 4;
    } while (chVTIsSystemTimeWithinX(start, end));
  }
  test_end_step(2);

  /* [8.3.3] Stopping the target thread.*/
  test_set_step(3);
  {
    chSysLock();
    chSchWakeupS(tp, MSG_TIMEOUT);
    chSysUnlock();
    chThdWait(tp);
  }
  test_end_step(3);

  /* [8.3.4] Score is printed.*/
  test_set_step(4);
  {
    test_print("--- Score : ");
    test_printn(n * 2);
    test_println(" ctxswc/S");
  }
  test_end_step(4);
}

static const testcase_t nil_test_008_003 = {
  "Context Switch performance",
  NULL,
  NULL,
  nil_test_008_003_execute
};

/**
 * @page nil_test_008_004 [8.4] Threads performance, full cycle
 *
 * <h2>Description</h2>
 * Threads are continuously created and terminated into a loop. A full
 * chThdCreateStatic() / @p chThdExit() / @p chThdWait() cycle is
 * performed in each iteration.<br> The performance is calculated by
 * measuring the number of iterations after a second of continuous
 * operations.
 *
 * <h2>Test Steps</h2>
 * - [8.4.1] A thread is created at a lower priority level and its
 *   termination detected using @p chThdWait(). The operation is
 *   repeated continuously in a one-second time window.
 * - [8.4.2] Score is printed.
 * .
 */

static void nil_test_008_004_execute(void) {
  uint32_t n;

  /* [8.4.1] A thread is created at a lower priority level and its
     termination detected using @p chThdWait(). The operation is
     repeated continuously in a one-second time window.*/
  test_set_step(1);
  {
    systime_t start, end;
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() + 1,
      .funcp = bmk_thread3,
      .arg   = NULL
    };

    n = 0;
    start = test_wait_tick();
    end = chTimeAddX(start, TIME_MS2I(1000));
    do {
      chThdWait(chThdCreate(&td));
      n++;
    } while (chVTIsSystemTimeWithinX(start, end));
  }
  test_end_step(1);

  /* [8.4.2] Score is printed.*/
  test_set_step(2);
  {
    test_print("--- Score : ");
    test_printn(n);
    test_println(" threads/S");
  }
  test_end_step(2);
}

static const testcase_t nil_test_008_004 = {
  "Threads performance, full cycle",
  NULL,
  NULL,
  nil_test_008_004_execute
};

/**
 * @page nil_test_008_005 [8.5] Threads performance, create/exit only
 *
 * <h2>Description</h2>
 * Threads are continuously created and terminated into a loop. A
 * partial @p chThdCreateStatic() / @p chThdExit() cycle is performed
 * in each iteration, the @p chThdWait() is not necessary because the
 * thread is created at an higher priority so there is no need to wait
 * for it to terminate.<br> The performance is calculated by measuring
 * the number of iterations after a second of continuous operations.
 *
 * <h2>Test Steps</h2>
 * - [8.5.1] A thread is created at an higher priority level and let
 *   terminate immediately. The operation is repeated continuously in a
 *   one-second time window.
 * - [8.5.2] Score is printed.
 * .
 */

static void nil_test_008_005_execute(void) {
  uint32_t n;

  /* [8.5.1] A thread is created at an higher priority level and let
     terminate immediately. The operation is repeated continuously in a
     one-second time window.*/
  test_set_step(1);
  {
    systime_t start, end;
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() - 1,
      .funcp = bmk_thread3,
      .arg   = NULL
    };

    n = 0;
    start = test_wait_tick();
    end = chTimeAddX(start, TIME_MS2I(1000));
    do {
      chThdWait(chThdCreate(&td));
      n++;
    } while (chVTIsSystemTimeWithinX(start, end));
  }
  test_end_step(1);

  /* [8.5.2] Score is printed.*/
  test_set_step(2);
  {
    test_print("--- Score : ");
    test_printn(n);
    test_println(" threads/S");
  }
  test_end_step(2);
}

static const testcase_t nil_test_008_005 = {
  "Threads performance, create/exit only",
  NULL,
  NULL,
  nil_test_008_005_execute
};

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @page nil_test_008_006 [8.6] Semaphores wait/signal performance
 *
 * <h2>Description</h2>
 * A counting semaphore is taken/released into a continuous loop, no
 * Context Switch happens because the counter is always non
 * negative.<br> The performance is calculated by measuring the number
 * of iterations after a second of continuous operations.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_SEMAPHORES == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [8.6.1] A semaphore is teken and released. The operation is
 *   repeated continuously in a one-second time window.
 * - [8.6.2] The score is printed.
 * .
 */

static void nil_test_008_006_setup(void) {
  chSemObjectInit(&sem1, 1);
}

static void nil_test_008_006_execute(void) {
  uint32_t n;

  /* [8.6.1] A semaphore is teken and released. The operation is
     repeated continuously in a one-second time window.*/
  test_set_step(1);
  {
    systime_t start, end;

    n = 0;
    start = test_wait_tick();
    end = chTimeAddX(start, TIME_MS2I(1000));
    do {
      chSemWait(&sem1);
      chSemSignal(&sem1);
      chSemWait(&sem1);
      chSemSignal(&sem1);
      chSemWait(&sem1);
      chSemSignal(&sem1);
      chSemWait(&sem1);
      chSemSignal(&sem1);
      n++;
#if defined(SIMULATOR)
      _sim_check_for_interrupts();
#endif
    } while (chVTIsSystemTimeWithinX(start, end));
  }
  test_end_step(1);

  /* [8.6.2] The score is printed.*/
  test_set_step(2);
  {
    test_print("--- Score : ");
    test_printn(n * 4);
    test_println(" wait+signal/S");
  }
  test_end_step(2);
}

static const testcase_t nil_test_008_006 = {
  "Semaphores wait/signal performance",
  nil_test_008_006_setup,
  NULL,
  nil_test_008_006_execute
};
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

/**
 * @page nil_test_008_007 [8.7] RAM Footprint
 *
 * <h2>Description</h2>
 * The memory size of the various kernel objects is printed.
 *
 * <h2>Test Steps</h2>
 * - [8.7.1] The size of the system area is printed.
 * - [8.7.2] The size of a thread structure is printed.
 * - [8.7.3] The size of a semaphore structure is printed.
 * - [8.7.4] The size of an event source is printed.
 * - [8.7.5] The size of an event listener is printed.
 * - [8.7.6] The size of a mailbox is printed.
 * .
 */

static void nil_test_008_007_execute(void) {

  /* [8.7.1] The size of the system area is printed.*/
  test_set_step(1);
  {
    test_print("--- OS    : ");
    test_printn(sizeof(os_instance_t));
    test_println(" bytes");
  }
  test_end_step(1);

  /* [8.7.2] The size of a thread structure is printed.*/
  test_set_step(2);
  {
    test_print("--- Thread: ");
    test_printn(sizeof(thread_t));
    test_println(" bytes");
  }
  test_end_step(2);

  /* [8.7.3] The size of a semaphore structure is printed.*/
  test_set_step(3);
  {
#if CH_CFG_USE_SEMAPHORES || defined(__DOXYGEN__)
    test_print("--- Semaph: ");
    test_printn(sizeof(semaphore_t));
    test_println(" bytes");
#endif
  }
  test_end_step(3);

  /* [8.7.4] The size of an event source is printed.*/
  test_set_step(4);
  {
#if CH_CFG_USE_EVENTS || defined(__DOXYGEN__)
    test_print("--- EventS: ");
    test_printn(sizeof(event_source_t));
    test_println(" bytes");
#endif
  }
  test_end_step(4);

  /* [8.7.5] The size of an event listener is printed.*/
  test_set_step(5);
  {
#if CH_CFG_USE_EVENTS || defined(__DOXYGEN__)
    test_print("--- EventL: ");
    test_printn(sizeof(event_listener_t));
    test_println(" bytes");
#endif
  }
  test_end_step(5);

  /* [8.7.6] The size of a mailbox is printed.*/
  test_set_step(6);
  {
#if CH_CFG_USE_MAILBOXES || defined(__DOXYGEN__)
    test_print("--- MailB.: ");
    test_printn(sizeof(mailbox_t));
    test_println(" bytes");
#endif
  }
  test_end_step(6);
}

static const testcase_t nil_test_008_007 = {
  "RAM Footprint",
  NULL,
  NULL,
  nil_test_008_007_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_008_array[] = {
#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
  &nil_test_008_001,
#endif
#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
  &nil_test_008_002,
#endif
  &nil_test_008_003,
  &nil_test_008_004,
  &nil_test_008_005,
#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  &nil_test_008_006,
#endif
  &nil_test_008_007,
  NULL
};

/**
 * @brief   Benchmarks.
 */
const testsequence_t nil_test_sequence_008 = {
  "Benchmarks",
  nil_test_sequence_008_array
};
