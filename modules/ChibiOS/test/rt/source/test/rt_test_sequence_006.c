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
 * @file    rt_test_sequence_006.c
 * @brief   Test Sequence 006 code.
 *
 * @page rt_test_sequence_006 [6] Suspend/Resume
 *
 * File: @ref rt_test_sequence_006.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/RT functionalities related to
 * threads suspend/resume.
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_006_001
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static thread_reference_t tr1;

static THD_FUNCTION(thread1, p) {

  chSysLock();
  chThdResumeI(&tr1, MSG_OK);
  chSchRescheduleS();
  chSysUnlock();
  test_emit_token(*(char *)p);
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_006_001 [6.1] Suspend and Resume functionality
 *
 * <h2>Description</h2>
 * The functionality of chThdSuspendTimeoutS() and chThdResumeI() is
 * tested.
 *
 * <h2>Test Steps</h2>
 * - [6.1.1] The function chThdSuspendTimeoutS() is invoked, the thread
 *   is remotely resumed with message @p MSG_OK. On return the message
 *   and the state of the reference are tested.
 * - [6.1.2] The function chThdSuspendTimeoutS() is invoked, the thread
 *   is not resumed so a timeout must occur. On return the message and
 *   the state of the reference are tested.
 * .
 */

static void rt_test_006_001_setup(void) {
  tr1 = NULL;
}

static void rt_test_006_001_execute(void) {
  systime_t time;
  msg_t msg;

  /* [6.1.1] The function chThdSuspendTimeoutS() is invoked, the thread
     is remotely resumed with message @p MSG_OK. On return the message
     and the state of the reference are tested.*/
  test_set_step(1);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX()-1, thread1, "A");
    chSysLock();
    msg = chThdSuspendTimeoutS(&tr1, TIME_INFINITE);
    chSysUnlock();
    test_assert(NULL == tr1, "not NULL");
    test_assert(MSG_OK == msg,"wrong returned message");
    test_wait_threads();
  }
  test_end_step(1);

  /* [6.1.2] The function chThdSuspendTimeoutS() is invoked, the thread
     is not resumed so a timeout must occur. On return the message and
     the state of the reference are tested.*/
  test_set_step(2);
  {
    chSysLock();
    time = chVTGetSystemTimeX();
    msg = chThdSuspendTimeoutS(&tr1, TIME_MS2I(10));
    chSysUnlock();
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(10)),
                            chTimeAddX(time, TIME_MS2I(10) + CH_CFG_ST_TIMEDELTA + 1),
                            "out of time window");
    test_assert(NULL == tr1, "not NULL");
    test_assert(MSG_TIMEOUT == msg, "wrong returned message");
  }
  test_end_step(2);
}

static const testcase_t rt_test_006_001 = {
  "Suspend and Resume functionality",
  rt_test_006_001_setup,
  NULL,
  rt_test_006_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_006_array[] = {
  &rt_test_006_001,
  NULL
};

/**
 * @brief   Suspend/Resume.
 */
const testsequence_t rt_test_sequence_006 = {
  "Suspend/Resume",
  rt_test_sequence_006_array
};
