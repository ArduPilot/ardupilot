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
 * Test suite for ChibiOS OS Library. The purpose of this suite is to
 * perform unit tests on the library modules and to converge to 100%
 * code coverage through successive improvements.
 *
 * <h2>Test Sequences</h2>
 * - @subpage oslib_test_sequence_001
 * - @subpage oslib_test_sequence_002
 * - @subpage oslib_test_sequence_003
 * - @subpage oslib_test_sequence_004
 * - @subpage oslib_test_sequence_005
 * - @subpage oslib_test_sequence_006
 * - @subpage oslib_test_sequence_007
 * - @subpage oslib_test_sequence_008
 * - @subpage oslib_test_sequence_009
 * .
 */

/**
 * @file    oslib_test_root.c
 * @brief   Test Suite root structures code.
 */

#include "hal.h"
#include "oslib_test_root.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Array of test sequences.
 */
const testsequence_t * const oslib_test_suite_array[] = {
  &oslib_test_sequence_001,
#if (CH_CFG_USE_MAILBOXES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_002,
#endif
#if (CH_CFG_USE_PIPES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_003,
#endif
#if (CH_CFG_USE_JOBS == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_004,
#endif
#if (CH_CFG_USE_DELEGATES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_005,
#endif
#if (CH_CFG_USE_OBJ_CACHES == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_006,
#endif
#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_007,
#endif
#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)
  &oslib_test_sequence_008,
#endif
#if ((CH_CFG_USE_FACTORY == TRUE) && (CH_CFG_USE_MEMPOOLS == TRUE) && (CH_CFG_USE_HEAP == TRUE)) || defined(__DOXYGEN__)
  &oslib_test_sequence_009,
#endif
  NULL
};

/**
 * @brief   Test suite root structure.
 */
const testsuite_t oslib_test_suite = {
  "ChibiOS OS Library Test Suite",
  oslib_test_suite_array
};

/*===========================================================================*/
/* Shared code.                                                              */
/*===========================================================================*/

#endif /* !defined(__DOXYGEN__) */
