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
 * @file    nil_test_sequence_002.c
 * @brief   Test Sequence 002 code.
 *
 * @page nil_test_sequence_002 [2] Time and Intervals Functionality
 *
 * File: @ref nil_test_sequence_002.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS/NIL functionalities related to time
 * and intervals management.
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_002_001
 * - @subpage nil_test_002_002
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
 * @page nil_test_002_001 [2.1] System Tick Counter functionality
 *
 * <h2>Description</h2>
 * The functionality of the API @p chVTGetSystemTimeX() is tested.
 *
 * <h2>Test Steps</h2>
 * - [2.1.1] A System Tick Counter increment is expected, the test
 *   simply hangs if it does not happen.
 * .
 */

static void nil_test_002_001_execute(void) {

  /* [2.1.1] A System Tick Counter increment is expected, the test
     simply hangs if it does not happen.*/
  test_set_step(1);
  {
    systime_t time = chVTGetSystemTimeX();
    while (time == chVTGetSystemTimeX()) {
    }
  }
  test_end_step(1);
}

static const testcase_t nil_test_002_001 = {
  "System Tick Counter functionality",
  NULL,
  NULL,
  nil_test_002_001_execute
};

/**
 * @page nil_test_002_002 [2.2] Time ranges functionality
 *
 * <h2>Description</h2>
 * The functionality of the API @p chTimeIsInRangeX() is tested.
 *
 * <h2>Test Steps</h2>
 * - [2.2.1] Checking case where start == end, it must always evaluate
 *   as not in range.
 * - [2.2.2] Checking boundaries for start < end.
 * - [2.2.3] Checking boundaries for start > end.
 * .
 */

static void nil_test_002_002_execute(void) {

  /* [2.2.1] Checking case where start == end, it must always evaluate
     as not in range.*/
  test_set_step(1);
  {
    bool b;

    b = chTimeIsInRangeX((systime_t)0, (systime_t)0, (systime_t)0);
    test_assert(b == false, "in range");
    b = chTimeIsInRangeX((systime_t)-1, (systime_t)0, (systime_t)0);
    test_assert(b == false, "in range");
    b = chTimeIsInRangeX((systime_t)0, (systime_t)-1, (systime_t)-1);
    test_assert(b == false, "in range");
    b = chTimeIsInRangeX((systime_t)-1, (systime_t)-1, (systime_t)-1);
    test_assert(b == false, "in range");
  }
  test_end_step(1);

  /* [2.2.2] Checking boundaries for start < end.*/
  test_set_step(2);
  {
    bool b;

    b = chTimeIsInRangeX((systime_t)10, (systime_t)10, (systime_t)100);
    test_assert(b == true, "not in range");
    b = chTimeIsInRangeX((systime_t)9, (systime_t)10, (systime_t)100);
    test_assert(b == false, "in range");
    b = chTimeIsInRangeX((systime_t)99, (systime_t)10, (systime_t)100);
    test_assert(b == true, "not in range");
    b = chTimeIsInRangeX((systime_t)100, (systime_t)10, (systime_t)100);
    test_assert(b == false, "in range");
  }
  test_end_step(2);

  /* [2.2.3] Checking boundaries for start > end.*/
  test_set_step(3);
  {
    bool b;

    b = chTimeIsInRangeX((systime_t)100, (systime_t)100, (systime_t)10);
    test_assert(b == true, "not in range");
    b = chTimeIsInRangeX((systime_t)99, (systime_t)100, (systime_t)10);
    test_assert(b == false, "in range");
    b = chTimeIsInRangeX((systime_t)9, (systime_t)100, (systime_t)10);
    test_assert(b == true, "not in range");
    b = chTimeIsInRangeX((systime_t)10, (systime_t)100, (systime_t)10);
    test_assert(b == false, "in range");
  }
  test_end_step(3);
}

static const testcase_t nil_test_002_002 = {
  "Time ranges functionality",
  NULL,
  NULL,
  nil_test_002_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_002_array[] = {
  &nil_test_002_001,
  &nil_test_002_002,
  NULL
};

/**
 * @brief   Time and Intervals Functionality.
 */
const testsequence_t nil_test_sequence_002 = {
  "Time and Intervals Functionality",
  nil_test_sequence_002_array
};
