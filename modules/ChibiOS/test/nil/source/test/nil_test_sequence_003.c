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
 * @file    nil_test_sequence_003.c
 * @brief   Test Sequence 003 code.
 *
 * @page nil_test_sequence_003 [3] Threads Functionality
 *
 * File: @ref nil_test_sequence_003.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/NIL functionalities related to
 * threading.
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_003_001
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "ch.h"

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nil_test_003_001 [3.1] Thread Sleep functionality
 *
 * <h2>Description</h2>
 * The functionality of @p chThdSleep() and derivatives is tested.
 *
 * <h2>Test Steps</h2>
 * - [3.1.1] The current system time is read then a sleep is performed
 *   for 100 system ticks and on exit the system time is verified
 *   again.
 * - [3.1.2] The current system time is read then a sleep is performed
 *   for 100000 microseconds and on exit the system time is verified
 *   again.
 * - [3.1.3] The current system time is read then a sleep is performed
 *   for 100 milliseconds and on exit the system time is verified
 *   again.
 * - [3.1.4] The current system time is read then a sleep is performed
 *   for 1 second and on exit the system time is verified again.
 * - [3.1.5] Function chThdSleepUntil() is tested with a timeline of
 *   "now" + 100 ticks.
 * .
 */

static void nil_test_003_001_execute(void) {
  systime_t time;

  /* [3.1.1] The current system time is read then a sleep is performed
     for 100 system ticks and on exit the system time is verified
     again.*/
  test_set_step(1);
  {
    time = chVTGetSystemTimeX();
    chThdSleep(100);
    test_assert_time_window(chTimeAddX(time, 100),
                            chTimeAddX(time, 100 + 1),
                            "out of time window");
  }
  test_end_step(1);

  /* [3.1.2] The current system time is read then a sleep is performed
     for 100000 microseconds and on exit the system time is verified
     again.*/
  test_set_step(2);
  {
    time = chVTGetSystemTimeX();
    chThdSleepMicroseconds(100000);
    test_assert_time_window(chTimeAddX(time, TIME_US2I(100000)),
                            chTimeAddX(time, TIME_US2I(100000) + 1),
                            "out of time window");
  }
  test_end_step(2);

  /* [3.1.3] The current system time is read then a sleep is performed
     for 100 milliseconds and on exit the system time is verified
     again.*/
  test_set_step(3);
  {
    time = chVTGetSystemTimeX();
    chThdSleepMilliseconds(100);
    test_assert_time_window(chTimeAddX(time, TIME_MS2I(100)),
                            chTimeAddX(time, TIME_MS2I(100) + 1),
                            "out of time window");
  }
  test_end_step(3);

  /* [3.1.4] The current system time is read then a sleep is performed
     for 1 second and on exit the system time is verified again.*/
  test_set_step(4);
  {
    time = chVTGetSystemTimeX();
    chThdSleepSeconds(1);
    test_assert_time_window(chTimeAddX(time, TIME_S2I(1)),
                            chTimeAddX(time, TIME_S2I(1) + 1),
                            "out of time window");
  }
  test_end_step(4);

  /* [3.1.5] Function chThdSleepUntil() is tested with a timeline of
     "now" + 100 ticks.*/
  test_set_step(5);
  {
    time = chVTGetSystemTimeX();
    chThdSleepUntil(chTimeAddX(time, 100));
    test_assert_time_window(chTimeAddX(time, 100),
                            chTimeAddX(time, 100 + 1),
                            "out of time window");
  }
  test_end_step(5);
}

static const testcase_t nil_test_003_001 = {
  "Thread Sleep functionality",
  NULL,
  NULL,
  nil_test_003_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_003_array[] = {
  &nil_test_003_001,
  NULL
};

/**
 * @brief   Threads Functionality.
 */
const testsequence_t nil_test_sequence_003 = {
  "Threads Functionality",
  nil_test_sequence_003_array
};
