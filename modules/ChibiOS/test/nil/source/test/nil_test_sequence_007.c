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
 * @file    nil_test_sequence_007.c
 * @brief   Test Sequence 007 code.
 *
 * @page nil_test_sequence_007 [7] Synchronous Messages
 *
 * File: @ref nil_test_sequence_007.c
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
 * - @subpage nil_test_007_001
 * .
 */

#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

/*
 * Messager thread.
 */
static THD_FUNCTION(messenger, p) {

  chMsgSend(p, 'A');
  chMsgSend(p, 'B');
  chMsgSend(p, 'C');
  chMsgSend(p, 'D');
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nil_test_007_001 [7.1] Messages Server loop
 *
 * <h2>Description</h2>
 * A messenger thread is spawned that sends four messages back to the
 * tester thread.<br> The test expect to receive the messages in the
 * correct sequence and to not find a fifth message waiting.
 *
 * <h2>Test Steps</h2>
 * - [7.1.1] Starting the messenger thread.
 * - [7.1.2] Waiting for four messages then testing the receive order.
 * .
 */

static void nil_test_007_001_execute(void) {
  thread_t *tp, *tp1;
  msg_t msg;

  /* [7.1.1] Starting the messenger thread.*/
  test_set_step(1);
  {
    thread_descriptor_t td = {
      .name  = "messenger",
      .wbase = wa_common,
      .wend  = THD_WORKING_AREA_END(wa_common),
      .prio  = chThdGetPriorityX() - 1,
      .funcp = messenger,
      .arg   = chThdGetSelfX()
    };
    tp1 = chThdCreate(&td);
  }
  test_end_step(1);

  /* [7.1.2] Waiting for four messages then testing the receive
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
    chThdWait(tp1);
    test_assert_sequence("ABCD", "invalid sequence");
  }
  test_end_step(2);
}

static const testcase_t nil_test_007_001 = {
  "Messages Server loop",
  NULL,
  NULL,
  nil_test_007_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_007_array[] = {
  &nil_test_007_001,
  NULL
};

/**
 * @brief   Synchronous Messages.
 */
const testsequence_t nil_test_sequence_007 = {
  "Synchronous Messages",
  nil_test_sequence_007_array
};

#endif /* CH_CFG_USE_MESSAGES == TRUE */
