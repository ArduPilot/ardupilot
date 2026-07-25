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
 * @file    rt_test_sequence_004.c
 * @brief   Test Sequence 004 code.
 *
 * @page rt_test_sequence_004 [4] Time Stamps Functionality
 *
 * File: @ref rt_test_sequence_004.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/RT functionalities related to time
 * stamps.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_TIMESTAMP == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_004_001
 * .
 */

#if (CH_CFG_USE_TIMESTAMP == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "ch.h"

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page rt_test_004_001 [4.1] Time Stamps functionality
 *
 * <h2>Description</h2>
 * The functionality of the API @p chVTGetTimeStamp() is tested.
 *
 * <h2>Test Steps</h2>
 * - [4.1.1] Time stamps are generated and checked for monotonicity.
 * .
 */

static void rt_test_004_001_execute(void) {

  /* [4.1.1] Time stamps are generated and checked for monotonicity.*/
  test_set_step(1);
  {
    systime_t start, end;
    systimestamp_t last, now;
    sysinterval_t duration;

    last = chVTGetTimeStamp();
    start = test_wait_tick();
    duration = (sysinterval_t)(TIME_MAX_SYSTIME / 2U);
    if (duration > TIME_MS2I(1000)) {
      duration = TIME_MS2I(1000);
    }
    end = chTimeAddX(start, duration);
    do {
      now = chVTGetTimeStamp();
      test_assert(last <= now, "not monotonic");
      last = now;
#if defined(SIMULATOR)
      _sim_check_for_interrupts();
#endif
    } while (chVTIsSystemTimeWithinX(start, end));
  }
  test_end_step(1);
}

static const testcase_t rt_test_004_001 = {
  "Time Stamps functionality",
  NULL,
  NULL,
  rt_test_004_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_004_array[] = {
  &rt_test_004_001,
  NULL
};

/**
 * @brief   Time Stamps Functionality.
 */
const testsequence_t rt_test_sequence_004 = {
  "Time Stamps Functionality",
  rt_test_sequence_004_array
};

#endif /* CH_CFG_USE_TIMESTAMP == TRUE */
