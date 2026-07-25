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
 * @file    nil_test_sequence_004.c
 * @brief   Test Sequence 004 code.
 *
 * @page nil_test_sequence_004 [4] Semaphores
 *
 * File: @ref nil_test_sequence_004.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/NIL functionalities related to
 * counter semaphores.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_SEMAPHORES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_004_001
 * - @subpage nil_test_004_002
 * - @subpage nil_test_004_003
 * .
 */

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "ch.h"

static thread_t *tp1;
static bool terminate;
static semaphore_t sem1, sem2;

/*
 * Signaler thread.
 */
static THD_FUNCTION(signaler, arg) {

  (void)arg;

  /* Initializing global resources.*/
  terminate = false;
  chSemObjectInit(&sem1, 0);
  chSemObjectInit(&sem2, 0);

  while (!terminate) {
    chSysLock();
    if (chSemGetCounterI(&sem1) < 0)
      chSemSignalI(&sem1);
    chSemResetI(&sem2, 0);
    chSchRescheduleS();
    chSysUnlock();

    chThdSleepMilliseconds(250);
  }
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nil_test_004_001 [4.1] Semaphore primitives, no state change
 *
 * <h2>Description</h2>
 * Wait, Signal and Reset primitives are tested. The testing thread
 * does not trigger a state change.
 *
 * <h2>Test Steps</h2>
 * - [4.1.1] The function chSemWait() is invoked, after return the
 *   counter and the returned message are tested.
 * - [4.1.2] The function chSemSignal() is invoked, after return the
 *   counter is tested.
 * - [4.1.3] The function chSemReset() is invoked, after return the
 *   counter is tested.
 * .
 */

static void nil_test_004_001_setup(void) {
  chSemObjectInit(&sem1, 1);
}

static void nil_test_004_001_teardown(void) {
  chSemReset(&sem1, 0);
}

static void nil_test_004_001_execute(void) {

  /* [4.1.1] The function chSemWait() is invoked, after return the
     counter and the returned message are tested.*/
  test_set_step(1);
  {
    msg_t msg;

    msg = chSemWait(&sem1);
    test_assert_lock(chSemGetCounterI(&sem1) == 0, "wrong counter value");
    test_assert(MSG_OK == msg, "wrong returned message");
  }
  test_end_step(1);

  /* [4.1.2] The function chSemSignal() is invoked, after return the
     counter is tested.*/
  test_set_step(2);
  {
    chSemSignal(&sem1);
    test_assert_lock(chSemGetCounterI(&sem1) == 1, "wrong counter value");
  }
  test_end_step(2);

  /* [4.1.3] The function chSemReset() is invoked, after return the
     counter is tested.*/
  test_set_step(3);
  {
    chSemReset(&sem1, 2);
    test_assert_lock(chSemGetCounterI(&sem1) == 2, "wrong counter value");
  }
  test_end_step(3);
}

static const testcase_t nil_test_004_001 = {
  "Semaphore primitives, no state change",
  nil_test_004_001_setup,
  nil_test_004_001_teardown,
  nil_test_004_001_execute
};

/**
 * @page nil_test_004_002 [4.2] Semaphore primitives, with state change
 *
 * <h2>Description</h2>
 * Wait, Signal and Reset primitives are tested. The testing thread
 * triggers a state change.
 *
 * <h2>Test Steps</h2>
 * - [4.2.1] The function chSemWait() is invoked, after return the
 *   counter and the returned message are tested. The semaphore is
 *   signaled by another thread.
 * - [4.2.2] The function chSemWait() is invoked, after return the
 *   counter and the returned message are tested. The semaphore is
 *   reset by another thread.
 * .
 */

static void nil_test_004_002_setup(void) {
  thread_descriptor_t td = {
    .name  = "signaler",
    .wbase = wa_common,
    .wend  = THD_WORKING_AREA_END(wa_common),
    .prio  = chThdGetPriorityX() - 1,
    .funcp = signaler,
    .arg   = NULL
  };
  tp1 = chThdCreate(&td);
}

static void nil_test_004_002_teardown(void) {
  terminate = true;
  chThdWait(tp1);
}

static void nil_test_004_002_execute(void) {

  /* [4.2.1] The function chSemWait() is invoked, after return the
     counter and the returned message are tested. The semaphore is
     signaled by another thread.*/
  test_set_step(1);
  {
    msg_t msg;

    msg = chSemWait(&sem1);
    test_assert_lock(chSemGetCounterI(&sem1) == 0, "wrong counter value");
    test_assert(MSG_OK == msg, "wrong returned message");
  }
  test_end_step(1);

  /* [4.2.2] The function chSemWait() is invoked, after return the
     counter and the returned message are tested. The semaphore is
     reset by another thread.*/
  test_set_step(2);
  {
    msg_t msg;

    msg = chSemWait(&sem2);
    test_assert_lock(chSemGetCounterI(&sem2) == 0,"wrong counter value");
    test_assert(MSG_RESET == msg, "wrong returned message");
  }
  test_end_step(2);
}

static const testcase_t nil_test_004_002 = {
  "Semaphore primitives, with state change",
  nil_test_004_002_setup,
  nil_test_004_002_teardown,
  nil_test_004_002_execute
};

/**
 * @page nil_test_004_003 [4.3] Semaphores timeout
 *
 * <h2>Description</h2>
 * Timeout on semaphores is tested.
 *
 * <h2>Test Steps</h2>
 * - [4.3.1] The function chSemWaitTimeout() is invoked a first time,
 *   after return the system time, the counter and the returned message
 *   are tested.
 * - [4.3.2] The function chSemWaitTimeout() is invoked again, after
 *   return the system time, the counter and the returned message are
 *   tested.
 * .
 */

static void nil_test_004_003_setup(void) {
  chSemObjectInit(&sem1, 0);
}

static void nil_test_004_003_teardown(void) {
  chSemReset(&sem1, 0);
}

static void nil_test_004_003_execute(void) {
  systime_t time;
  msg_t msg;

  /* [4.3.1] The function chSemWaitTimeout() is invoked a first time,
     after return the system time, the counter and the returned message
     are tested.*/
  test_set_step(1);
  {
    time = chVTGetSystemTimeX();
    msg = chSemWaitTimeout(&sem1, TIME_MS2I(1000));
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(1000)),
                            chTimeAddX(time, TIME_MS2I(1000) + 1),
                            "out of time window");
    test_assert_lock(chSemGetCounterI(&sem1) == 0, "wrong counter value");
    test_assert(MSG_TIMEOUT == msg, "wrong timeout message");
  }
  test_end_step(1);

  /* [4.3.2] The function chSemWaitTimeout() is invoked again, after
     return the system time, the counter and the returned message are
     tested.*/
  test_set_step(2);
  {
    time = chVTGetSystemTimeX();
    msg = chSemWaitTimeout(&sem1, TIME_MS2I(1000));
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(1000)),
                            chTimeAddX(time, TIME_MS2I(1000) + 1),
                            "out of time window");
    test_assert_lock(chSemGetCounterI(&sem1) == 0, "wrong counter value");
    test_assert(MSG_TIMEOUT == msg, "wrong timeout message");
  }
  test_end_step(2);
}

static const testcase_t nil_test_004_003 = {
  "Semaphores timeout",
  nil_test_004_003_setup,
  nil_test_004_003_teardown,
  nil_test_004_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_004_array[] = {
  &nil_test_004_001,
  &nil_test_004_002,
  &nil_test_004_003,
  NULL
};

/**
 * @brief   Semaphores.
 */
const testsequence_t nil_test_sequence_004 = {
  "Semaphores",
  nil_test_sequence_004_array
};

#endif /* CH_CFG_USE_SEMAPHORES == TRUE */
