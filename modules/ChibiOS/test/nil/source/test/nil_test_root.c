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

/**
 * @mainpage Test Suite Specification
 * Test suite for ChibiOS/NIL. The purpose of this suite is to perform
 * unit tests on the NIL modules and to converge to 100% code coverage
 * through successive improvements.
 *
 * <h2>Test Sequences</h2>
 * - @subpage nil_test_sequence_001
 * - @subpage nil_test_sequence_002
 * - @subpage nil_test_sequence_003
 * - @subpage nil_test_sequence_004
 * - @subpage nil_test_sequence_005
 * - @subpage nil_test_sequence_006
 * - @subpage nil_test_sequence_007
 * - @subpage nil_test_sequence_008
 * .
 */

/**
 * @file    nil_test_root.c
 * @brief   Test Suite root structures code.
 */

#include "hal.h"
#include "nil_test_root.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Array of test sequences.
 */
const testsequence_t * const nil_test_suite_array[] = {
  &nil_test_sequence_001,
  &nil_test_sequence_002,
  &nil_test_sequence_003,
#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
  &nil_test_sequence_004,
#endif
  &nil_test_sequence_005,
#if (CH_CFG_USE_EVENTS == TRUE) || defined(__DOXYGEN__)
  &nil_test_sequence_006,
#endif
#if (CH_CFG_USE_MESSAGES == TRUE) || defined(__DOXYGEN__)
  &nil_test_sequence_007,
#endif
  &nil_test_sequence_008,
  NULL
};

/**
 * @brief   Test suite root structure.
 */
const testsuite_t nil_test_suite = {
  "ChibiOS/NIL Test Suite",
  nil_test_suite_array
};

/*===========================================================================*/
/* Shared code.                                                              */
/*===========================================================================*/

THD_WORKING_AREA(wa_common, 128);

/*
 * Delays execution until next system time tick.
 */
systime_t test_wait_tick(void) {

  chThdSleep(1);
  return chVTGetSystemTimeX();
}

#endif /* !defined(__DOXYGEN__) */
