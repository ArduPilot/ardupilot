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
 * @file    rt_test_sequence_009.c
 * @brief   Test Sequence 009 code.
 *
 * @page rt_test_sequence_009 [9] Synchronous Messages
 *
 * File: @ref rt_test_sequence_009.c
 *
 * <h2>Description</h2>
 * This module implements the test sequence for the Synchronous
 * Messages subsystem.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MESSAGES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_009_001
 * .
 */

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static THD_FUNCTION(msg_thread1, p) {

  chMsgSend(p, 'A');
  chMsgSend(p, 'B');
  chMsgSend(p, 'C');
  chMsgSend(p, 'D');
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_009_001 [9.1] Messages Server loop
 *
 * <h2>Description</h2>
 * A messenger thread is spawned that sends four messages back to the
 * tester thread.<br> The test expect to receive the messages in the
 * correct sequence and to not find a fifth message waiting.
 *
 * <h2>Test Steps</h2>
 * - [9.1.1] Starting the messenger thread.
 * - [9.1.2] Waiting for four messages then testing the receive order.
 * .
 */

static void rt_test_009_001_execute(void) {
  thread_t *tp;
  msg_t msg;

  /* [9.1.1] Starting the messenger thread.*/
  test_set_step(1);
  {
    threads[0] = chThdCreateStatic(wa[0], WA_SIZE, chThdGetPriorityX() + 1,
                                   msg_thread1, chThdGetSelfX());
  }
  test_end_step(1);

  /* [9.1.2] Waiting for four messages then testing the receive
     order.*/
  test_set_step(2);
  {
    unsigned i;

    for (i = 0; i < 4; i++) {
      tp = chMsgWait();
      msg = chMsgGet(tp);
      chMsgRelease(tp, msg);
      test_emit_token(msg);
    }
    test_wait_threads();
    test_assert_sequence("ABCD", "invalid sequence");
  }
  test_end_step(2);
}

static const testcase_t rt_test_009_001 = {
  "Messages Server loop",
  NULL,
  NULL,
  rt_test_009_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_009_array[] = {
  &rt_test_009_001,
  NULL
};

/**
 * @brief   Synchronous Messages.
 */
const testsequence_t rt_test_sequence_009 = {
  "Synchronous Messages",
  rt_test_sequence_009_array
};

#endif /* CH_CFG_USE_MESSAGES == TRUE */
