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
 * @file    mfs_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page mfs_test_sequence_001 [1] Functional tests
 *
 * File: @ref mfs_test_sequence_001.c
 *
 * <h2>Description</h2>
 * The APIs are tested for functionality, correct cases and expected
 * error cases are tested.
 *
 * <h2>Test Cases</h2>
 * - @subpage mfs_test_001_001
 * - @subpage mfs_test_001_002
 * - @subpage mfs_test_001_003
 * - @subpage mfs_test_001_004
 * - @subpage mfs_test_001_005
 * - @subpage mfs_test_001_006
 * - @subpage mfs_test_001_007
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
 * @page mfs_test_001_001 [1.1] Testing mfsStart() behavior
 *
 * <h2>Description</h2>
 * The initialization function is tested. This function can fail only
 * in case of Flash Array failures or in case of unexpected internal
 * errors.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] Erasing the flash array using a low level function.
 * - [1.1.2] Calling mfsStart() on an uninitialized flash array,
 *   MFS_NO_ERROR is expected.
 * - [1.1.3] Calling mfsStart() on a newly initialized flash array,
 *   MFS_NO_ERROR is expected.
 * .
 */

static void mfs_test_001_001_setup(void) {
  mfsObjectInit(&mfs1, &__nocache_mfsbuf1);
}

static void mfs_test_001_001_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_001_execute(void) {

  /* [1.1.1] Erasing the flash array using a low level function.*/
  test_set_step(1);
  {
    flash_error_t ferr;

    ferr = bank_erase(MFS_BANK_0);
    test_assert(ferr == FLASH_NO_ERROR, "Bank 0 erase failure");
    ferr = bank_erase(MFS_BANK_1);
    test_assert(ferr == FLASH_NO_ERROR, "Bank 1 erase failure");
  }
  test_end_step(1);

  /* [1.1.2] Calling mfsStart() on an uninitialized flash array,
     MFS_NO_ERROR is expected.*/
  test_set_step(2);
  {
    mfs_error_t err;

    err = mfsStart(&mfs1, &mfscfg1);
    test_assert(err == MFS_NO_ERROR, "initialization error with erased flash");
  }
  test_end_step(2);

  /* [1.1.3] Calling mfsStart() on a newly initialized flash array,
     MFS_NO_ERROR is expected.*/
  test_set_step(3);
  {
    mfs_error_t err;

    err = mfsStart(&mfs1, &mfscfg1);
    test_assert(err == MFS_NO_ERROR, "initialization error with initialized flash");
  }
  test_end_step(3);
}

static const testcase_t mfs_test_001_001 = {
  "Testing mfsStart() behavior",
  mfs_test_001_001_setup,
  mfs_test_001_001_teardown,
  mfs_test_001_001_execute
};

/**
 * @page mfs_test_001_002 [1.2] Checking for non existing record
 *
 * <h2>Description</h2>
 * The records space is explored with an initialized but empty managed
 * storage, no record should exist.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] Exploring the records space, MFS_ERR_NOT_FOUND is expected
 *   for each index.
 * .
 */

static void mfs_test_001_002_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_001_002_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_002_execute(void) {

  /* [1.2.1] Exploring the records space, MFS_ERR_NOT_FOUND is expected
     for each index.*/
  test_set_step(1);
  {
    mfs_error_t err;
    mfs_id_t id;
    size_t size;

    for (id = 1; id <= MFS_CFG_MAX_RECORDS; id++) {
      size = sizeof __nocache_mfs_buffer;
      err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
      test_assert(err == MFS_ERR_NOT_FOUND,
                  "found a record that should not exists");
    }
  }
  test_end_step(1);
}

static const testcase_t mfs_test_001_002 = {
  "Checking for non existing record",
  mfs_test_001_002_setup,
  mfs_test_001_002_teardown,
  mfs_test_001_002_execute
};

/**
 * @page mfs_test_001_003 [1.3] Creating, updating and erasing a record
 *
 * <h2>Description</h2>
 * A record is created, updated several times with different payloads
 * and finally erased.
 *
 * <h2>Test Steps</h2>
 * - [1.3.1] The record must not already exists, MFS_ERR_NOT_FOUND is
 *   expected.
 * - [1.3.2] Creating the record then retrieving it again, MFS_NO_ERROR
 *   is expected, record content and size are compared with the
 *   original.
 * - [1.3.3] Updating the record then retrieving it again, MFS_NO_ERROR
 *   is expected, record content and size are compared with the
 *   original.
 * - [1.3.4] Erasing the record then trying to retrieve it again,
 *   MFS_NO_ERROR is expected on erase, MFS_ERR_NOT_FOUND is expected
 *   on retrieve.
 * .
 */

static void mfs_test_001_003_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_001_003_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_003_execute(void) {
  size_t size;

  /* [1.3.1] The record must not already exists, MFS_ERR_NOT_FOUND is
     expected.*/
  test_set_step(1);
  {
    size = sizeof __nocache_mfs_buffer;
    mfs_error_t err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND , "record was already present");
  }
  test_end_step(1);

  /* [1.3.2] Creating the record then retrieving it again, MFS_NO_ERROR
     is expected, record content and size are compared with the
     original.*/
  test_set_step(2);
  {
    mfs_error_t err;

    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating the record");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern16, "unexpected record length");
    test_assert(memcmp(mfs_pattern16, __nocache_mfs_buffer, size) == 0, "wrong record content");
  }
  test_end_step(2);

  /* [1.3.3] Updating the record then retrieving it again, MFS_NO_ERROR
     is expected, record content and size are compared with the
     original.*/
  test_set_step(3);
  {
    mfs_error_t err;

    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern32, mfs_pattern32);
    test_assert(err == MFS_NO_ERROR, "error updating the record");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern32, "unexpected record length");
    test_assert(memcmp(mfs_pattern32, __nocache_mfs_buffer, size) == 0, "wrong record content");
  }
  test_end_step(3);

  /* [1.3.4] Erasing the record then trying to retrieve it again,
     MFS_NO_ERROR is expected on erase, MFS_ERR_NOT_FOUND is expected
     on retrieve.*/
  test_set_step(4);
  {
    mfs_error_t err;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_NO_ERROR, "error erasing the record");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record not erased");
  }
  test_end_step(4);
}

static const testcase_t mfs_test_001_003 = {
  "Creating, updating and erasing a record",
  mfs_test_001_003_setup,
  mfs_test_001_003_teardown,
  mfs_test_001_003_execute
};

/**
 * @page mfs_test_001_004 [1.4] Erasing the whole storage and re-initialization
 *
 * <h2>Description</h2>
 * The managed storage is erased, initialized and re-mounted.
 *
 * <h2>Test Steps</h2>
 * - [1.4.1] Creating records 1, 2 and 3, MFS_NO_ERROR is expected.
 * - [1.4.2] Records must exist.
 * - [1.4.3] Re-mounting, records must still exist.
 * - [1.4.4] Erasing storage and verify that the records have been
 *   removed, MFS_NO_ERROR is expected on erase, MFS_ERR_NOT_FOUND is
 *   expected on retrieve.
 * .
 */

static void mfs_test_001_004_setup(void) {
  bank_erase(MFS_BANK_0);
  bank_erase(MFS_BANK_1);
  mfsStart(&mfs1, &mfscfg1);
}

static void mfs_test_001_004_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_004_execute(void) {

  /* [1.4.1] Creating records 1, 2 and 3, MFS_NO_ERROR is expected.*/
  test_set_step(1);
  {
    mfs_error_t err;

    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern16, mfs_pattern16);
    test_assert(err == MFS_NO_ERROR, "error creating record 1");
    err = mfsWriteRecord(&mfs1, 2, sizeof mfs_pattern32, mfs_pattern32);
    test_assert(err == MFS_NO_ERROR, "error creating record 2");
    err = mfsWriteRecord(&mfs1, 3, sizeof mfs_pattern10, mfs_pattern10);
    test_assert(err == MFS_NO_ERROR, "error creating record 3");
  }
  test_end_step(1);

  /* [1.4.2] Records must exist.*/
  test_set_step(2);
  {
    mfs_error_t err;
    size_t size;

    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 0 not present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 1 not present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 2 not present");
  }
  test_end_step(2);

  /* [1.4.3] Re-mounting, records must still exist.*/
  test_set_step(3);
  {
    mfs_error_t err;
    size_t size;

    err = mfsStart(&mfs1, &mfscfg1);
    test_assert(err == MFS_NO_ERROR, "re-mount failed");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 0 not present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 1 not present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record 2 not present");
  }
  test_end_step(3);

  /* [1.4.4] Erasing storage and verify that the records have been
     removed, MFS_NO_ERROR is expected on erase, MFS_ERR_NOT_FOUND is
     expected on retrieve.*/
  test_set_step(4);
  {
    mfs_error_t err;
    size_t size;

    err = mfsErase(&mfs1);
    test_assert(err == MFS_NO_ERROR, "storage erase error");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record 0 still present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 2, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record 1 still present");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 3, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record 2 still present");
  }
  test_end_step(4);
}

static const testcase_t mfs_test_001_004 = {
  "Erasing the whole storage and re-initialization",
  mfs_test_001_004_setup,
  mfs_test_001_004_teardown,
  mfs_test_001_004_execute
};

/**
 * @page mfs_test_001_005 [1.5] Testing storage size limit
 *
 * <h2>Description</h2>
 * The storage is entirely filled with different records and the final
 * error is tested.
 *
 * <h2>Test Steps</h2>
 * - [1.5.1] Filling up the storage by writing records with increasing
 *   IDs, MFS_NO_ERROR is expected.
 * - [1.5.2] Creating one more record, should fail, MFS_ERR_OUT_OF_MEM
 *   is expected.
 * - [1.5.3] Adding a smaller record to fill the final gap. A
 *   reinitialization is performed and MFS_NO_ERROR is expected.
 * .
 */

static void mfs_test_001_005_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
  mfsErase(&mfs1);
}

static void mfs_test_001_005_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_005_execute(void) {

  /* [1.5.1] Filling up the storage by writing records with increasing
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

  /* [1.5.2] Creating one more record, should fail, MFS_ERR_OUT_OF_MEM
     is expected.*/
  test_set_step(2);
  {
    mfs_error_t err;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + sizeof mfs_pattern512);

    err = mfsWriteRecord(&mfs1, id_max, sizeof mfs_pattern512 , mfs_pattern512);
    test_assert(err == MFS_ERR_OUT_OF_MEM, "creation didn't fail");
  }
  test_end_step(2);

  /* [1.5.3] Adding a smaller record to fill the final gap. A
     reinitialization is performed and MFS_NO_ERROR is expected.*/
  test_set_step(3);
  {
    mfs_error_t err;
    size_t remaining;

    remaining = (size_t)flashGetSectorOffset(mfscfg1.flashp, mfscfg1.bank0_start) +
                (size_t)mfscfg1.bank_size - (size_t)mfs1.next_offset;
    test_assert(remaining >= sizeof (mfs_data_header_t), "not enough space");

    if (remaining > sizeof (mfs_data_header_t) * 2) {
      err = mfsWriteRecord(&mfs1, MFS_CFG_MAX_RECORDS,
                           remaining - (sizeof (mfs_data_header_t) * 2),
                           mfs_pattern512);
      test_assert(err == MFS_NO_ERROR, "error filling remaining space");
      err = mfsEraseRecord(&mfs1, MFS_CFG_MAX_RECORDS);
      test_assert(err == MFS_NO_ERROR, "error filling remaining space");
    }
    else {
      if (remaining == sizeof (mfs_data_header_t) * 2) {
        err = mfsEraseRecord(&mfs1, 2);
        test_assert(err == MFS_NO_ERROR, "error filling remaining space");
      }
      err = mfsEraseRecord(&mfs1, 1);
      test_assert(err == MFS_NO_ERROR, "error filling remaining space");
    }

    remaining = (size_t)flashGetSectorOffset(mfscfg1.flashp, mfscfg1.bank0_start) +
                (size_t)mfscfg1.bank_size - (size_t)mfs1.next_offset;
    test_assert(remaining == 0U, "remaining space not zero");

    mfsStop(&mfs1);
    err = mfsStart(&mfs1, &mfscfg1);
    test_assert(err == MFS_NO_ERROR, "initialization error");
  }
  test_end_step(3);
}

static const testcase_t mfs_test_001_005 = {
  "Testing storage size limit",
  mfs_test_001_005_setup,
  mfs_test_001_005_teardown,
  mfs_test_001_005_execute
};

/**
 * @page mfs_test_001_006 [1.6] Testing garbage collection by writing
 *
 * <h2>Description</h2>
 * The garbage collection procedure is triggeredby a write operation
 * and the state of both banks is checked.
 *
 * <h2>Test Steps</h2>
 * - [1.6.1] Filling up the storage by writing records with increasing
 *   IDs, MFS_NO_ERROR is expected.
 * - [1.6.2] Erasing one record, MFS_NO_ERROR is expected.
 * - [1.6.3] Writing one more record triggers garbage collection,
 *   MFS_WARN_GC is expected, KS state is checked for correctness after
 *   the operation.
 * - [1.6.4] Checking for all records in the new bank, MFS_NOERROR is
 *   expected for each record.
 * - [1.6.5] Erasing one record, MFS_NO_ERROR is expected.
 * - [1.6.6] Writing one more record triggers garbage collection,
 *   MFS_WARN_GC is expected, MFS object state is checked for
 *   correctness after the operation.
 * - [1.6.7] Checking for all records in the new bank, MFS_NO_ERROR is
 *   expected for each record.
 * .
 */

static void mfs_test_001_006_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
  mfsErase(&mfs1);
}

static void mfs_test_001_006_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_006_execute(void) {

  /* [1.6.1] Filling up the storage by writing records with increasing
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

  /* [1.6.2] Erasing one record, MFS_NO_ERROR is expected.*/
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

  /* [1.6.3] Writing one more record triggers garbage collection,
     MFS_WARN_GC is expected, KS state is checked for correctness after
     the operation.*/
  test_set_step(3);
  {
    mfs_error_t err;
    size_t size;

    test_assert(mfs1.current_counter == 1, "not first instance");
    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern512, mfs_pattern512);
    test_assert(err == MFS_WARN_GC, "error creating the record");
    test_assert(mfs1.current_counter == 2, "not second instance");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern512, "unexpected record length");
    test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                "wrong record content");
    test_assert(mfs1.current_bank == MFS_BANK_1, "unexpected bank");
    test_assert(bank_verify_erased(MFS_BANK_0) == FLASH_NO_ERROR, "bank 0 not erased");
  }
  test_end_step(3);

  /* [1.6.4] Checking for all records in the new bank, MFS_NOERROR is
     expected for each record.*/
  test_set_step(4);
  {
    mfs_id_t id;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + sizeof mfs_pattern512);

    for (id = 1; id <= MFS_CFG_MAX_RECORDS; id++) {
      mfs_error_t err;
      size_t size;

      if (id <= id_max) {
        size = sizeof __nocache_mfs_buffer;
        err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
        test_assert(err == MFS_NO_ERROR, "record not found");
        test_assert(size == sizeof mfs_pattern512, "unexpected record length");
        test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                    "wrong record content");
      }
      else {
        size = sizeof __nocache_mfs_buffer;
        err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
        test_assert(err == MFS_ERR_NOT_FOUND, "found unexpected record");
      }
    }
  }
  test_end_step(4);

  /* [1.6.5] Erasing one record, MFS_NO_ERROR is expected.*/
  test_set_step(5);
  {
    mfs_error_t err;
    size_t size;

    err = mfsEraseRecord(&mfs1, 1);
    test_assert(err == MFS_NO_ERROR, "error erasing the record");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record not erased");
  }
  test_end_step(5);

  /* [1.6.6] Writing one more record triggers garbage collection,
     MFS_WARN_GC is expected, MFS object state is checked for
     correctness after the operation.*/
  test_set_step(6);
  {
    mfs_error_t err;
    size_t size;

    test_assert(mfs1.current_counter == 2, "not second instance");
    err = mfsWriteRecord(&mfs1, 1, sizeof mfs_pattern512, mfs_pattern512);
    test_assert(err == MFS_WARN_GC, "error creating the record");
    test_assert(mfs1.current_counter == 3, "not third instance");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, 1, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_NO_ERROR, "record not found");
    test_assert(size == sizeof mfs_pattern512, "unexpected record length");
    test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                "wrong record content");
    test_assert(mfs1.current_bank == MFS_BANK_0, "unexpected bank");
    test_assert(bank_verify_erased(MFS_BANK_1) == FLASH_NO_ERROR, "bank 1 not erased");
  }
  test_end_step(6);

  /* [1.6.7] Checking for all records in the new bank, MFS_NO_ERROR is
     expected for each record.*/
  test_set_step(7);
  {
    mfs_id_t id;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + sizeof mfs_pattern512);

    for (id = 1; id <= MFS_CFG_MAX_RECORDS; id++) {
      mfs_error_t err;
      size_t size;

      if (id <= id_max) {
        size = sizeof __nocache_mfs_buffer;
        err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
        test_assert(err == MFS_NO_ERROR, "record not found");
        test_assert(size == sizeof mfs_pattern512, "unexpected record length");
        test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                    "wrong record content");
      }
      else {
        size = sizeof __nocache_mfs_buffer;
        err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
        test_assert(err == MFS_ERR_NOT_FOUND, "found unexpected record");
      }
    }
  }
  test_end_step(7);
}

static const testcase_t mfs_test_001_006 = {
  "Testing garbage collection by writing",
  mfs_test_001_006_setup,
  mfs_test_001_006_teardown,
  mfs_test_001_006_execute
};

/**
 * @page mfs_test_001_007 [1.7] Testing garbage collection by erasing
 *
 * <h2>Description</h2>
 * The garbage collection procedure is triggered by an erase operation
 * and the state of both banks is checked.
 *
 * <h2>Test Steps</h2>
 * - [1.7.1] Filling up the storage by writing records with increasing
 *   IDs, MFS_NO_ERROR is expected.
 * - [1.7.2] Erase records until the flash bank is filled entirely.
 * - [1.7.3] Erasing one more record triggers garbage collection,
 *   MFS_WARN_GC is expected, KS state is checked for correctness after
 *   the operation.
 * .
 */

static void mfs_test_001_007_setup(void) {
  mfsStart(&mfs1, &mfscfg1);
  mfsErase(&mfs1);
}

static void mfs_test_001_007_teardown(void) {
  mfsStop(&mfs1);
}

static void mfs_test_001_007_execute(void) {

  /* [1.7.1] Filling up the storage by writing records with increasing
     IDs, MFS_NO_ERROR is expected.*/
  test_set_step(1);
  {
    mfs_id_t id;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + (sizeof mfs_pattern512 / 4));

    for (id = 1; id <= id_max; id++) {
      mfs_error_t err;
      size_t size;

      err = mfsWriteRecord(&mfs1, id, (sizeof mfs_pattern512 / 4), mfs_pattern512);
      test_assert(err == MFS_NO_ERROR, "error creating the record");
      size = sizeof __nocache_mfs_buffer;
      err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
      test_assert(err == MFS_NO_ERROR, "record not found");
      test_assert(size == (sizeof mfs_pattern512 / 4), "unexpected record length");
      test_assert(memcmp(mfs_pattern512, __nocache_mfs_buffer, size) == 0,
                  "wrong record content");
    }
  }
  test_end_step(1);

  /* [1.7.2] Erase records until the flash bank is filled entirely.*/
  test_set_step(2);
  {
    mfs_error_t err;
    size_t size;
    mfs_id_t id;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + (sizeof mfs_pattern512 / 4));
    mfs_id_t n = ((mfscfg1.bank_size - sizeof (mfs_bank_header_t)) -
                  (id_max * (sizeof (mfs_data_header_t) + (sizeof mfs_pattern512 / 4)))) /
                 sizeof (mfs_data_header_t);

    for (id = 1; id <= n; id++) {
      err = mfsEraseRecord(&mfs1, id);
      test_assert(err == MFS_NO_ERROR, "error erasing the record");
      size = sizeof __nocache_mfs_buffer;
      err = mfsReadRecord(&mfs1, id, &size, __nocache_mfs_buffer);
      test_assert(err == MFS_ERR_NOT_FOUND, "record not erased");
    }
  }
  test_end_step(2);

  /* [1.7.3] Erasing one more record triggers garbage collection,
     MFS_WARN_GC is expected, KS state is checked for correctness after
     the operation.*/
  test_set_step(3);
  {
    mfs_error_t err;
    size_t size;
    mfs_id_t id_max = (mfscfg1.bank_size - (sizeof (mfs_bank_header_t) +
                                            sizeof (mfs_data_header_t))) /
                      (sizeof (mfs_data_header_t) + (sizeof mfs_pattern512 / 4));

    test_assert(mfs1.current_counter == 1, "not first instance");
    err = mfsEraseRecord(&mfs1, id_max);
    test_assert(err == MFS_WARN_GC, "error erasing the record");
    test_assert(mfs1.current_counter == 2, "not second instance");
    size = sizeof __nocache_mfs_buffer;
    err = mfsReadRecord(&mfs1, id_max, &size, __nocache_mfs_buffer);
    test_assert(err == MFS_ERR_NOT_FOUND, "record not erased");
    test_assert(mfs1.current_bank == MFS_BANK_1, "unexpected bank");
    test_assert(bank_verify_erased(MFS_BANK_0) == FLASH_NO_ERROR, "bank 0 not erased");
  }
  test_end_step(3);
}

static const testcase_t mfs_test_001_007 = {
  "Testing garbage collection by erasing",
  mfs_test_001_007_setup,
  mfs_test_001_007_teardown,
  mfs_test_001_007_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const mfs_test_sequence_001_array[] = {
  &mfs_test_001_001,
  &mfs_test_001_002,
  &mfs_test_001_003,
  &mfs_test_001_004,
  &mfs_test_001_005,
  &mfs_test_001_006,
  &mfs_test_001_007,
  NULL
};

/**
 * @brief   Functional tests.
 */
const testsequence_t mfs_test_sequence_001 = {
  "Functional tests",
  mfs_test_sequence_001_array
};
