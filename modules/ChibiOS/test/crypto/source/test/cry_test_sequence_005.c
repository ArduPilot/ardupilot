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
#include "cry_test_root.h"

/**
 * @file    cry_test_sequence_005.c
 * @brief   Test Sequence 005 code.
 *
 * @page cry_test_sequence_005 [5] TRNG
 *
 * File: @ref cry_test_sequence_005.c
 *
 * <h2>Description</h2>
 * TRNG testing.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_005_001
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

static const CRYConfig configTRNG_Polling=
{
    TRANSFER_POLLING,
    0
};



/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_005_001 [5.1] TRNG Polling
 *
 * <h2>Description</h2>
 * testing TRNG in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [5.1.1] Random generation and test.
 * .
 */

static void cry_test_005_001_setup(void) {
  cryStart(&CRYD1, &configTRNG_Polling);


}

static void cry_test_005_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_005_001_execute(void) {
    cryerror_t ret;

  /* [5.1.1] Random generation and test.*/
  test_set_step(1);
  {
    uint32_t random[4];
    int i;

    ret = cryTRNG(&CRYD1,(uint8_t*)&random);

    test_assert(ret == CRY_NOERROR , "failed random");

    SHOW_DATA(&random[0],4);

    for (i=0;i<4;i++) {
    	test_assert(random[i]  != 0 , "failed random generation (zero)");
    }
    for (i=1;i<4;i++)
    {
    	test_assert(random[i-1]  != random[i] , "failed random generation");
    }


  }
  test_end_step(1);
}

static const testcase_t cry_test_005_001 = {
  "TRNG Polling",
  cry_test_005_001_setup,
  cry_test_005_001_teardown,
  cry_test_005_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_005_array[] = {
  &cry_test_005_001,
  NULL
};

/**
 * @brief   TRNG.
 */
const testsequence_t cry_test_sequence_005 = {
  "TRNG",
  cry_test_sequence_005_array
};
