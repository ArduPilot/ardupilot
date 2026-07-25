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
#include "mfs_test_root.h"

/**
 * @file    mfs_test_sequence_003.c
 * @brief   Test Sequence 003 code.
 *
 * @page mfs_test_sequence_003 [3] API Invalid Cases tests
 *
 * File: @ref mfs_test_sequence_003.c
 *
 * <h2>Description</h2>
 * This test sequence tests the error coded returned by the various
 * APIs when called when the system is not initialized.
 *
 * <h2>Test Cases</h2>
 * - @subpage mfs_test_003_001
 * - @subpage mfs_test_003_002
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "hal_mfs.h"

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page mfs_test_003_001 [3.1] Initialization error from APIs
 *
 * <h2>Description</h2>
 * The API functions are invoked without prior initialization.
 *
 * <h2>Test Steps</h2>
 * - [3.1.1] The function mfsErase() is called, MFS_ERR_INV_STATE is
 *   expected.
 * - [3.1.2] The function mfsWriteRecord() is called, MFS_ERR_INV_STATE
 *   is expected.
 * - [3.1.3] The function mfsEraseRecord() is called, MFS_ERR_INV_STATE
 *   is expected.
 * - [3.1.4] The function mfsReadRecord() is called, MFS_ERR_INV_STATE
 *   is expected.
 * - [3.1.5] The function mfsPerformGarbageCollection() is called,
 *   MFS_ERR_INV_STATE is expected.
 * .
 */

static void mfs_test_003_001_execute(void) {

  /* [3.1.1] The function mfsErase() is called, MFS_ERR_INV_STATE is
     expected.*/
  test_set_step(1);
  {
    mfs_error_t err = mfsErase(&mfs1);
    test_assert(err == MFS_ERR_INV_STATE, "mfsErase() returned wrong status");
  }
  test_end_step(1);

  /* [3.1.2] The function mfsWriteRecord() is called, MFS_ERR_INV_STATE
     is expected.*/
  test_set_step(2);
  {
    mfs_error_t err = mfsWriteRecord(&mfs1, 1, 16, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_INV_STATE, "mfsWriteRecord() returned wrong status");
  }
  test_end_step(2);

  /* [3.1.3] The function mfsEraseRecord() is called, MFS_ERR_INV_STATE
     is expected.*/
  test_set_step(3);
  {
    mfs_error_t err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_ERR_INV_STATE, "mfsEraseRecord() returned wrong status");
  }
  test_end_step(3);

  /* [3.1.4] The function mfsReadRecord() is called, MFS_ERR_INV_STATE
     is expected.*/
  test_set_step(4);
  {
    size_t size = sizeof __nocache_mfs_buffer;
    mfs_error_t err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_INV_STATE, "mfsReadRecord() returned wrong status");
  }
  test_end_step(4);

  /* [3.1.5] The function mfsPerformGarbageCollection() is called,
     MFS_ERR_INV_STATE is expected.*/
  test_set_step(5);
  {
    mfs_error_t err = mfsPerformGarbageCollection(&mfs1);
    test_assert(err == MFS_ERR_INV_STATE, "mfsPerformGarbageCollection() returned wrong status");
  }
  test_end_step(5);
}

static const testcase_t mfs_test_003_001 = {
  "Initialization error from APIs",
  NULL,
  NULL,
  mfs_test_003_001_execute
};

/**
 * @page mfs_test_003_002 [3.2] Erasing non existing record
 *
 * <h2>Description</h2>
 * An erase operation is attempted on an non-existing record.
 *
 * <h2>Test Steps</h2>
 * - [3.2.1] Record one is erased, the error MFS_ERR_NOT_FOUND is
 *   expected.
 * .
 */

static void mfs_test_003_002_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
  mfsErase(&mfs1);
}

static void mfs_test_003_002_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_003_002_execute(void) {

  /* [3.2.1] Record one is erased, the error MFS_ERR_NOT_FOUND is
     expected.*/
  test_set_step(1);
  {
    mfs_error_t err;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err != MFS_NO_ERROR, "record was present");
    test_assert(err == MFS_ERR_NOT_FOUND, "invalid error code");
  }
  test_end_step(1);
}

static const testcase_t mfs_test_003_002 = {
  "Erasing non existing record",
  mfs_test_003_002_setup,
  mfs_test_003_002_teardown,
  mfs_test_003_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const mfs_test_sequence_003_array[] = {
  &mfs_test_003_001,
  &mfs_test_003_002,
  NULL
};

/**
 * @brief   API Invalid Cases tests.
 */
const testsequence_t mfs_test_sequence_003 = {
  "API Invalid Cases tests",
  mfs_test_sequence_003_array
};
