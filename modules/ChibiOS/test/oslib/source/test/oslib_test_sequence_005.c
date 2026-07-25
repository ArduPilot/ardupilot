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
#include "oslib_test_root.h"

/**
 * @file    oslib_test_sequence_005.c
 * @brief   Test Sequence 005 code.
 *
 * @page oslib_test_sequence_005 [5] Thread Delegates
 *
 * File: @ref oslib_test_sequence_005.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * Thread Delegates.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_DELEGATES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_005_001
 * .
 */

#if (CH_CFG_USE_DELEGATES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

static bool exit_flag;

static int dis_func0(void) {

  test_emit_token('0');

  return (msg_t)0x55AA;
}

static msg_t dis_func1(msg_t a) {

  test_emit_token((char)a);

  return (msg_t)a;
}

static msg_t dis_func2(msg_t a, msg_t b) {

  test_emit_token((char)a);
  test_emit_token((char)b);

  return (msg_t)a;
}

static msg_t dis_func3(msg_t a, msg_t b, msg_t c) {

  test_emit_token((char)a);
  test_emit_token((char)b);
  test_emit_token((char)c);

  return (msg_t)a;
}

static msg_t dis_func4(msg_t a, msg_t b, msg_t c, msg_t d) {

  test_emit_token((char)a);
  test_emit_token((char)b);
  test_emit_token((char)c);
  test_emit_token((char)d);

  return (msg_t)a;
}

static int dis_func_end(void) {

  test_emit_token('Z');
  exit_flag = true;

  return (msg_t)0xAA55;
}

static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  exit_flag = false;
  do {
    chDelegateDispatch();
  } while (!exit_flag);

  chThdExit(0x0FA5);
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page oslib_test_005_001 [5.1] Dispatcher test
 *
 * <h2>Description</h2>
 * The dispatcher API is tested for functionality.
 *
 * <h2>Test Steps</h2>
 * - [5.1.1] Starting the dispatcher thread.
 * - [5.1.2] Calling the default veneers, checking the result and the
 *   emitted tokens.
 * - [5.1.3] Waiting for the thread to terminate-.
 * .
 */

static void oslib_test_005_001_execute(void) {
  thread_t *tp;

  /* [5.1.1] Starting the dispatcher thread.*/
  test_set_step(1);
  {
    thread_descriptor_t td = {
      .name  = "dispatcher",
      .wbase = waThread1,
      .wend  = THD_WORKING_AREA_END(waThread1),
      .prio  = chThdGetPriorityX() + 1,
      .funcp = Thread1,
      .arg   = NULL
    };
    tp = chThdCreate(&td);
  }
  test_end_step(1);

  /* [5.1.2] Calling the default veneers, checking the result and the
     emitted tokens.*/
  test_set_step(2);
  {
    int retval;

    retval = chDelegateCallDirect0(tp, (delegate_fn0_t)dis_func0);
    test_assert(retval == 0x55AA, "invalid return value");

    retval = chDelegateCallDirect1(tp, (delegate_fn1_t)dis_func1, 'A');
    test_assert(retval == (int)'A', "invalid return value");

    retval = chDelegateCallDirect2(tp, (delegate_fn2_t)dis_func2, 'B', 'C');
    test_assert(retval == (int)'B', "invalid return value");

    retval = chDelegateCallDirect3(tp, (delegate_fn3_t)dis_func3, 'D', 'E', 'F');
    test_assert(retval == (int)'D', "invalid return value");

    retval = chDelegateCallDirect4(tp, (delegate_fn4_t)dis_func4, 'G', 'H', 'I', 'J');
    test_assert(retval == (int)'G', "invalid return value");

    retval = chDelegateCallDirect0(tp, (delegate_fn0_t)dis_func_end);
    test_assert(retval == 0xAA55, "invalid return value");

    test_assert_sequence("0ABCDEFGHIJZ", "unexpected tokens");
  }
  test_end_step(2);

  /* [5.1.3] Waiting for the thread to terminate-.*/
  test_set_step(3);
  {
    msg_t msg = chThdWait(tp);
    test_assert(msg == 0x0FA5, "invalid exit code");
  }
  test_end_step(3);
}

static const testcase_t oslib_test_005_001 = {
  "Dispatcher test",
  NULL,
  NULL,
  oslib_test_005_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_005_array[] = {
  &oslib_test_005_001,
  NULL
};

/**
 * @brief   Thread Delegates.
 */
const testsequence_t oslib_test_sequence_005 = {
  "Thread Delegates",
  oslib_test_sequence_005_array
};

#endif /* CH_CFG_USE_DELEGATES == TRUE */
