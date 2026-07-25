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
 * @file    nil_test_sequence_005.c
 * @brief   Test Sequence 005 code.
 *
 * @page nil_test_sequence_005 [5] Suspend/Resume
 *
 * File: @ref nil_test_sequence_005.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/NIL functionalities related to
 * threads suspend/resume.
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_005_001
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static thread_t *tp1;
static bool terminate;
static thread_reference_t tr1;

/*
 * Resumer thread.
 */
static THD_FUNCTION(resumer, arg) {

  (void)arg;

  /* Initializing global resources.*/
  terminate = false;
  tr1 = NULL;

  while (!terminate) {
    chThdResume(&tr1, MSG_OK);
    chThdSleepMilliseconds(250);
  }
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nil_test_005_001 [5.1] Suspend and Resume functionality
 *
 * <h2>Description</h2>
 * The functionality of chThdSuspendTimeoutS() and chThdResumeI() is
 * tested.
 *
 * <h2>Test Steps</h2>
 * - [5.1.1] The function chThdSuspendTimeoutS() is invoked, the thread
 *   is remotely resumed with message @p MSG_OK. On return the message
 *   and the state of the reference are tested.
 * - [5.1.2] The function chThdSuspendTimeoutS() is invoked, the thread
 *   is not resumed so a timeout must occur. On return the message and
 *   the state of the reference are tested.
 * .
 */

static void nil_test_005_001_setup(void) {
  thread_descriptor_t td = {
    .name  = "resumer",
    .wbase = wa_common,
    .wend  = THD_WORKING_AREA_END(wa_common),
    .prio  = chThdGetPriorityX() - 1,
    .funcp = resumer,
    .arg   = NULL
  };
  tp1 = chThdCreate(&td);
}

static void nil_test_005_001_teardown(void) {
  terminate = true;
  chThdWait(tp1);
}

static void nil_test_005_001_execute(void) {
  systime_t time;
  msg_t msg;

  /* [5.1.1] The function chThdSuspendTimeoutS() is invoked, the thread
     is remotely resumed with message @p MSG_OK. On return the message
     and the state of the reference are tested.*/
  test_set_step(1);
  {
    chSysLock();
    msg = chThdSuspendTimeoutS(&tr1, TIME_INFINITE);
    chSysUnlock();
    test_assert(NULL == tr1, "not NULL");
    test_assert(MSG_OK == msg,"wrong returned message");
  }
  test_end_step(1);

  /* [5.1.2] The function chThdSuspendTimeoutS() is invoked, the thread
     is not resumed so a timeout must occur. On return the message and
     the state of the reference are tested.*/
  test_set_step(2);
  {
    thread_reference_t tr = NULL;

    chSysLock();
    time = chVTGetSystemTimeX();
    msg = chThdSuspendTimeoutS(&tr, TIME_MS2I(1000));
    chSysUnlock();
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(1000)),
                            chTimeAddX(time, TIME_MS2I(1000) + 1),
                            "out of time window");
    test_assert(NULL == tr, "not NULL");
    test_assert(MSG_TIMEOUT == msg, "wrong returned message");
  }
  test_end_step(2);
}

static const testcase_t nil_test_005_001 = {
  "Suspend and Resume functionality",
  nil_test_005_001_setup,
  nil_test_005_001_teardown,
  nil_test_005_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_005_array[] = {
  &nil_test_005_001,
  NULL
};

/**
 * @brief   Suspend/Resume.
 */
const testsequence_t nil_test_sequence_005 = {
  "Suspend/Resume",
  nil_test_sequence_005_array
};
