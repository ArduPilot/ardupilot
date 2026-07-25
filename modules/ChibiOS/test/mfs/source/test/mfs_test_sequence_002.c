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
 * @file    mfs_test_sequence_002.c
 * @brief   Test Sequence 002 code.
 *
 * @page mfs_test_sequence_002 [2] Transaction Mode tests
 *
 * File: @ref mfs_test_sequence_002.c
 *
 * <h2>Description</h2>
 * This sequence tests the MFS behavior when used in transaction mode,
 * correct cases and expected error cases are tested.
 *
 * <h2>Test Cases</h2>
 * - @subpage mfs_test_002_001
 * - @subpage mfs_test_002_002
 * - @subpage mfs_test_002_003
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "hal_mfs.h"

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page mfs_test_002_001 [2.1] Committing a transaction
 *
 * <h2>Description</h2>
 * A set of new/existing records are written/erased within a
 * transaction then the transaction is committed, the state is checked
 * afterward.
 *
 * <h2>Test Steps</h2>
 * - [2.1.1] Records 1, 2 and 3 are created, MFS_NO_ERROR is expected.
 * - [2.1.2] Presence of records 1, 2 and 3 is verified, MFS_NO_ERROR
 *   is expected.
 * - [2.1.3] Starting a transaction with sufficient pre-allocated
 *   space, MFS_NO_ERROR is expected.
 * - [2.1.4] Atomically erasing record 1, updating record 2, reading
 *   record 3.
 * - [2.1.5] Committing the transaction, MFS_NO_ERROR is expected.
 * - [2.1.6] Testing outcome, records 1 must not be present, record 2
 *   must contain the new value and record 3 must be unchanged.
 * - [2.1.7] Re-mounting the manage storage, MFS_NO_ERROR is expected.
 * - [2.1.8] Testing outcome again after re-start.
 * - [2.1.9] Performing a garbage collection, the result must not
 *   change.
 * - [2.1.10] Testing outcome again after garbage collection.
 * .
 */

static void mfs_test_002_001_setup(void) {
  bank_erase(MFS_BANK_0);
  bank_erase(MFS_BANK_1);
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_002_001_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_002_001_execute(void) {
  uint32_t current_counter;
  uint32_t used_space;

  /* [2.1.1] Records 1, 2 and 3 are created, MFS_NO_ERROR is
     expected.*/
  test_set_step(1);
  {
    mfs_error_t err;

    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 1");
    err = mfsWriteRecord(&mfs1, 2, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 2");
    err = mfsWriteRecord(&mfs1, 3, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 3");
  }
  test_end_step(1);

  /* [2.1.2] Presence of records 1, 2 and 3 is verified, MFS_NO_ERROR
     is expected.*/
  test_set_step(2);
  {
    mfs_error_t err;
    size_t size;

    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
  }
  test_end_step(2);

  /* [2.1.3] Starting a transaction with sufficient pre-allocated
     space, MFS_NO_ERROR is expected.*/
  test_set_step(3);
  {
    mfs_error_t err;

    err = mfsStartTransaction(&mfs1, 1024U);
    test_assert(err == MFS_NO_ERROR, "error starting transaction");
  }
  test_end_step(3);

  /* [2.1.4] Atomically erasing record 1, updating record 2, reading
     record 3.*/
  test_set_step(4);
  {
    mfs_error_t err;
    size_t size;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_NO_ERROR, "error erasing record 1");
    err = mfsWriteRecord(&mfs1, 2, sizeof mfs_pattern32, mfs_pattern32);
    test_assert(err == MFS_NO_ERROR, "error writing record 2");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");
  }
  test_end_step(4);

  /* [2.1.5] Committing the transaction, MFS_NO_ERROR is expected.*/
  test_set_step(5);
  {
    mfs_error_t err;

    err = mfsCommitTransaction(&mfs1);
    test_assert(err == MFS_NO_ERROR, "error committing transaction");

    /* Saving some internal state for successive checks.*/
    current_counter = mfs1.current_counter;
    used_space      = mfs1.used_space;
  }
  test_end_step(5);

  /* [2.1.6] Testing outcome, records 1 must not be present, record 2
     must contain the new value and record 3 must be unchanged.*/
  test_set_step(6);
  {
    mfs_error_t err;
    size_t size;

    /* Record 1 must not be present.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record found");

    /* Record 2 must contain the new value.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern32, "unexpected record length");
    test_assert(memcmp(mfs_pattern32, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Record 3 must be unchanged.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Checking internal data.*/
    test_assert(MFS_BANK_0 == mfs1.current_bank, "internal data mismatch");
    test_assert(current_counter == mfs1.current_counter, "internal data mismatch");
    test_assert(used_space == mfs1.used_space, "internal data mismatch");
  }
  test_end_step(6);

  /* [2.1.7] Re-mounting the manage storage, MFS_NO_ERROR is
     expected.*/
  test_set_step(7);
  {
    mfs_error_t err;

    err = mfsStart(&mfs1, &mfscfg1);
    test_assert(err == MFS_NO_ERROR, "re-start failed");
  }
  test_end_step(7);

  /* [2.1.8] Testing outcome again after re-start.*/
  test_set_step(8);
  {
    mfs_error_t err;
    size_t size;

    /* Record 1 must not be present.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record found");

    /* Record 2 must contain the new value.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern32, "unexpected record length");
    test_assert(memcmp(mfs_pattern32, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Record 3 must be unchanged.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Checking internal data.*/
    test_assert(MFS_BANK_0 == mfs1.current_bank, "internal data mismatch");
    test_assert(current_counter == mfs1.current_counter, "internal data mismatch");
    test_assert(used_space == mfs1.used_space, "internal data mismatch");
  }
  test_end_step(8);

  /* [2.1.9] Performing a garbage collection, the result must not
     change.*/
  test_set_step(9);
  {
    mfs_error_t err;

    err = mfsPerformGarbageCollection(&mfs1);
    test_assert(err == MFS_NO_ERROR, "garbage collection failed");
  }
  test_end_step(9);

  /* [2.1.10] Testing outcome again after garbage collection.*/
  test_set_step(10);
  {
    mfs_error_t err;
    size_t size;

    /* Record 1 must not be present.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record found");

    /* Record 2 must contain the new value.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern32, "unexpected record length");
    test_assert(memcmp(mfs_pattern32, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Record 3 must be unchanged.*/
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Checking internal data.*/
    test_assert(MFS_BANK_1 == mfs1.current_bank, "internal data mismatch");
    test_assert(current_counter == mfs1.current_counter - 1, "internal data mismatch");
    test_assert(used_space == mfs1.used_space, "internal data mismatch");
  }
  test_end_step(10);
}

static const testcase_t mfs_test_002_001 = {
  "Committing a transaction",
  mfs_test_002_001_setup,
  mfs_test_002_001_teardown,
  mfs_test_002_001_execute
};

/**
 * @page mfs_test_002_002 [2.2] Rolling back a transaction
 *
 * <h2>Description</h2>
 * A set of new/existing records are written/erased within a
 * transaction then the transaction is rolled back, the state is
 * checked afterward.
 *
 * <h2>Test Steps</h2>
 * - [2.2.1] Records 1, 2 and 3 are created, MFS_NO_ERROR is expected.
 * - [2.2.2] Presence of records 1, 2 and 3 is verified, MFS_NO_ERROR
 *   is expected.
 * - [2.2.3] Starting a transaction with sufficient pre-allocated
 *   space, MFS_NO_ERROR is expected..
 * - [2.2.4] Atomically erasing record 1, updating record 2, reading
 *   record 3.
 * - [2.2.5] Rolling back the transaction, MFS_NO_ERROR is expected.
 * - [2.2.6] State must not have changed, records 1, 2 and 3 must still
 *   be there unchanged.
 * .
 */

static void mfs_test_002_002_setup(void) {
  bank_erase(MFS_BANK_0);
  bank_erase(MFS_BANK_1);
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_002_002_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_002_002_execute(void) {
  uint32_t current_counter;
  uint32_t used_space;

  /* [2.2.1] Records 1, 2 and 3 are created, MFS_NO_ERROR is
     expected.*/
  test_set_step(1);
  {
    mfs_error_t err;

    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 1");
    err = mfsWriteRecord(&mfs1, 2, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 2");
    err = mfsWriteRecord(&mfs1, 3, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 3");
  }
  test_end_step(1);

  /* [2.2.2] Presence of records 1, 2 and 3 is verified, MFS_NO_ERROR
     is expected.*/
  test_set_step(2);
  {
    mfs_error_t err;
    size_t size;

    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
  }
  test_end_step(2);

  /* [2.2.3] Starting a transaction with sufficient pre-allocated
     space, MFS_NO_ERROR is expected..*/
  test_set_step(3);
  {
    mfs_error_t err;

    err = mfsStartTransaction(&mfs1, 1024U);
    test_assert(err == MFS_NO_ERROR, "error starting transaction");
  }
  test_end_step(3);

  /* [2.2.4] Atomically erasing record 1, updating record 2, reading
     record 3.*/
  test_set_step(4);
  {
    mfs_error_t err;
    size_t size;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_NO_ERROR, "error erasing record 1");
    err = mfsWriteRecord(&mfs1, 2, sizeof mfs_pattern32, mfs_pattern32);
    test_assert(err == MFS_NO_ERROR, "error writing record 2");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");

    /* Saving some internal state for successive checks.*/
    current_counter = mfs1.current_counter;
    used_space      = mfs1.used_space;
  }
  test_end_step(4);

  /* [2.2.5] Rolling back the transaction, MFS_NO_ERROR is expected.*/
  test_set_step(5);
  {
    mfs_error_t err;

    err = mfsRollbackTransaction(&mfs1);
    test_assert(err == MFS_NO_ERROR, "error rolling back transaction");
  }
  test_end_step(5);

  /* [2.2.6] State must not have changed, records 1, 2 and 3 must still
     be there unchanged.*/
  test_set_step(6);
  {
    mfs_error_t err;
    size_t size;

    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "size changed");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "size changed");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "size changed");

    /* Checking internal data.*/
    test_assert(MFS_BANK_1 == mfs1.current_bank, "internal data mismatch");
    test_assert(current_counter == mfs1.current_counter - 1, "internal data mismatch");
    test_assert(used_space == mfs1.used_space, "internal data mismatch");
  }
  test_end_step(6);
}

static const testcase_t mfs_test_002_002 = {
  "Rolling back a transaction",
  mfs_test_002_002_setup,
  mfs_test_002_002_teardown,
  mfs_test_002_002_execute
};

/**
 * @page mfs_test_002_003 [2.3] Transaction triggering an early garbage collect
 *
 * <h2>Description</h2>
 * A transaction is started with sufficient space but not contiguous, a
 * garbage collection is triggered.
 *
 * <h2>Test Steps</h2>
 * - [2.3.1] Filling up the storage by writing records with increasing
 *   IDs, MFS_NO_ERROR is expected.
 * - [2.3.2] Erasing one record, MFS_NO_ERROR is expected.
 * - [2.3.3] Starting a transaction with the whole remaining space,
 *   MFS_ERR_OUT_OF_MEM is expected.
 * - [2.3.4] Starting a transaction with insufficient space for one
 *   more header, MFS_ERR_OUT_OF_MEM is expected.
 * - [2.3.5] Starting a transaction with just enough space for one more
 *   header, MFS_NO_ERROR is expected.
 * - [2.3.6] Rolling back, MFS_NO_ERROR is expected.
 * .
 */

static void mfs_test_002_003_setup(void) {
  bank_erase(MFS_BANK_0);
  bank_erase(MFS_BANK_1);
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_002_003_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_002_003_execute(void) {

  /* [2.3.1] Filling up the storage by writing records with increasing
     IDs, MFS_NO_ERROR is expected.*/
  test_set_step(1);
  {
    mfs_id_t id;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + sizeof mfs_pattern512);

    for (id = 1; id <= id_max; id++) {
      mfs_error_t err;
      size_t size;

      err = mfsWriteRecord(&mfs1, id, sizeof mfs_pattern512, mfs_pattern512);
      test_assert(err == MFS_NO_ERROR, "error creating the record");
      size = sizeof __nocache_mfs_buffer;
      err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
      test_assert(err == MFS_NO_ERROR,
                  "record not found");
      test_assert(size == sizeof mfs_pattern512,
                  "unexpected record length");
      test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                  "wrong record content");
    }
  }
  test_end_step(1);

  /* [2.3.2] Erasing one record, MFS_NO_ERROR is expected.*/
  test_set_step(2);
  {
    mfs_error_t err;
    size_t size;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_NO_ERROR, "error erasing the record");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record not erased");
  }
  test_end_step(2);

  /* [2.3.3] Starting a transaction with the whole remaining space,
     MFS_ERR_OUT_OF_MEM is expected.*/
  test_set_step(3);
  {
    mfs_error_t err;
    size_t size = mfs1.config->bank_size - mfs1.used_space;

    err = mfsStartTransaction(&mfs1, size);
    test_assert(err == MFS_ERR_OUT_OF_MEM, "invalid error code");
  }
  test_end_step(3);

  /* [2.3.4] Starting a transaction with insufficient space for one
     more header, MFS_ERR_OUT_OF_MEM is expected.*/
  test_set_step(4);
  {
    mfs_error_t err;
    size_t size = ((mfs1.config->bank_size - mfs1.used_space) - sizeof (mfs_data_header_t)) + 1;

    err = mfsStartTransaction(&mfs1, size);
    test_assert(err == MFS_ERR_OUT_OF_MEM, "invalid error code");
  }
  test_end_step(4);

  /* [2.3.5] Starting a transaction with just enough space for one more
     header, MFS_NO_ERROR is expected.*/
  test_set_step(5);
  {
    mfs_error_t err;
    size_t size = (mfs1.config->bank_size - mfs1.used_space) - sizeof (mfs_data_header_t);

    err = mfsStartTransaction(&mfs1, size);
    test_assert(err == MFS_NO_ERROR, "error starting transaction");
  }
  test_end_step(5);

  /* [2.3.6] Rolling back, MFS_NO_ERROR is expected.*/
  test_set_step(6);
  {
    mfs_error_t err;

    err = mfsRollbackTransaction(&mfs1);
    test_assert(err == MFS_NO_ERROR, "error rolling back transaction");
  }
  test_end_step(6);
}

static const testcase_t mfs_test_002_003 = {
  "Transaction triggering an early garbage collect",
  mfs_test_002_003_setup,
  mfs_test_002_003_teardown,
  mfs_test_002_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const mfs_test_sequence_002_array[] = {
  &mfs_test_002_001,
  &mfs_test_002_002,
  &mfs_test_002_003,
  NULL
};

/**
 * @brief   Transaction Mode tests.
 */
const testsequence_t mfs_test_sequence_002 = {
  "Transaction Mode tests",
  mfs_test_sequence_002_array
};
