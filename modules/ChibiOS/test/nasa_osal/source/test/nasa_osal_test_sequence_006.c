/* Copyright statement.*/

#include "hal.h"
#include "nasa_osal_test_root.h"

/**
 * @file    nasa_osal_test_sequence_006.c
 * @brief   Test Sequence 006 code.
 *
 * @page nasa_osal_test_sequence_006 [6] Mutex Semaphores Functionality
 *
 * File: @ref nasa_osal_test_sequence_006.c
 *
 * <h2>Description</h2>
 * This sequence tests the NASA OSAL over ChibiOS/RT functionalities
 * related to mutex semaphores.
 *
 * <h2>Test Cases</h2>
 * - @subpage nasa_osal_test_006_001
 * - @subpage nasa_osal_test_006_002
 * - @subpage nasa_osal_test_006_003
 * - @subpage nasa_osal_test_006_004
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "osapi.h"

uint32 msid;

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nasa_osal_test_006_001 [6.1] OS_MutSemCreate() and OS_MutSemDelete() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_MutSemCreate() and OS_MutSemDelete() is
 * tested.
 *
 * <h2>Test Steps</h2>
 * - [6.1.1] OS_MutSemCreate() is invoked with sem_id set to NULL, an
 *   error is expected.
 * - [6.1.2] OS_MutSemCreate() is invoked with sem_name set to NULL, an
 *   error is expected.
 * - [6.1.3] OS_MutSemCreate() is invoked with a very long timer name,
 *   an error is expected.
 * - [6.1.4] OS_MutSemDelete() is invoked with timer_id set to -1, an
 *   error is expected.
 * - [6.1.5] OS_MutSemCreate() is invoked twice with duplicated name,
 *   an error is expected, then the queue is deleted using
 *   OS_MutSemDelete().
 * .
 */

static void nasa_osal_test_006_001_execute(void) {

  /* [6.1.1] OS_MutSemCreate() is invoked with sem_id set to NULL, an
     error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_MutSemCreate(NULL,                     /* Error.*/
                         "failing semaphore",
                         0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [6.1.2] OS_MutSemCreate() is invoked with sem_name set to NULL, an
     error is expected.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_MutSemCreate(&msid,
                         NULL,                      /* Error.*/
                         0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [6.1.3] OS_MutSemCreate() is invoked with a very long timer name,
     an error is expected.*/
  test_set_step(3);
  {
#if 0 /* Semaphore name currently not implemented.*/
    int32 err;

    err = OS_MutSemCreate(&msid,
                         "very very long semaphore name",   /* Error.*/
                         0);
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
#endif
  }
  test_end_step(3);

  /* [6.1.4] OS_MutSemDelete() is invoked with timer_id set to -1, an
     error is expected.*/
  test_set_step(4);
  {
    int32 err;

    err = OS_MutSemDelete((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "wrong semaphore id not detected");
  }
  test_end_step(4);

  /* [6.1.5] OS_MutSemCreate() is invoked twice with duplicated name,
     an error is expected, then the queue is deleted using
     OS_MutSemDelete().*/
  test_set_step(5);
  {
    int32 err;
    uint32 msid1; /*, msid2;*/

    err = OS_MutSemCreate(&msid1, "my semaphore", 0);
    test_assert(err == OS_SUCCESS, "semaphore creation failed");

#if 0 /* Semaphore name currently not implemented.*/
    err = OS_MutSemCreate(&msid2, "my semaphore", 0);
    test_assert(err == OS_ERR_NAME_TAKEN, "name conflict not detected");
#endif

    err = OS_MutSemDelete(msid1);
    test_assert(err == OS_SUCCESS, "semaphore deletion failed");
  }
  test_end_step(5);
}

static const testcase_t nasa_osal_test_006_001 = {
  "OS_MutSemCreate() and OS_MutSemDelete() errors",
  NULL,
  NULL,
  nasa_osal_test_006_001_execute
};

/**
 * @page nasa_osal_test_006_002 [6.2] OS_MutSemGive() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_MutSemGive() is tested.
 *
 * <h2>Test Steps</h2>
 * - [6.2.1] OS_MutSemGive() is invoked with sem_id set to -1, an error
 *   is expected.
 * .
 */

static void nasa_osal_test_006_002_execute(void) {

  /* [6.2.1] OS_MutSemGive() is invoked with sem_id set to -1, an error
     is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_MutSemGive((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "invalid sem_id not detected");
  }
  test_end_step(1);
}

static const testcase_t nasa_osal_test_006_002 = {
  "OS_MutSemGive() errors",
  NULL,
  NULL,
  nasa_osal_test_006_002_execute
};

/**
 * @page nasa_osal_test_006_003 [6.3] OS_MutSemTake() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_MutSemTake() is tested.
 *
 * <h2>Test Steps</h2>
 * - [6.3.1] OS_MutSemTake() is invoked with sem_id set to -1, an error
 *   is expected.
 * .
 */

static void nasa_osal_test_006_003_execute(void) {

  /* [6.3.1] OS_MutSemTake() is invoked with sem_id set to -1, an error
     is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_MutSemTake((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "invalid sem_id not detected");
  }
  test_end_step(1);
}

static const testcase_t nasa_osal_test_006_003 = {
  "OS_MutSemTake() errors",
  NULL,
  NULL,
  nasa_osal_test_006_003_execute
};

/**
 * @page nasa_osal_test_006_004 [6.4] OS_MutSemGetIdByName() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_MutSemGetIdByName() is tested.
 *
 * <h2>Test Steps</h2>
 * - [6.4.1] OS_MutSemGetIdByName() is invoked with sem_id set to NULL,
 *   an error is expected.
 * - [6.4.2] OS_MutSemGetIdByName() is invoked with semaphore name set
 *   to NULL, an error is expected.
 * - [6.4.3] OS_MutSemGetIdByName() is invoked with a very long task
 *   name, an error is expected.
 * .
 */

static void nasa_osal_test_006_004_execute(void) {

  /* [6.4.1] OS_MutSemGetIdByName() is invoked with sem_id set to NULL,
     an error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_MutSemGetIdByName(NULL, "semaphore");
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [6.4.2] OS_MutSemGetIdByName() is invoked with semaphore name set
     to NULL, an error is expected.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_MutSemGetIdByName(&msid, NULL);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [6.4.3] OS_MutSemGetIdByName() is invoked with a very long task
     name, an error is expected.*/
  test_set_step(3);
  {
    int32 err;

    err = OS_MutSemGetIdByName(&msid, "very very long semaphore name");
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
  }
  test_end_step(3);
}

static const testcase_t nasa_osal_test_006_004 = {
  "OS_MutSemGetIdByName() errors",
  NULL,
  NULL,
  nasa_osal_test_006_004_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nasa_osal_test_sequence_006_array[] = {
  &nasa_osal_test_006_001,
  &nasa_osal_test_006_002,
  &nasa_osal_test_006_003,
  &nasa_osal_test_006_004,
  NULL
};

/**
 * @brief   Mutex Semaphores Functionality.
 */
const testsequence_t nasa_osal_test_sequence_006 = {
  "Mutex Semaphores Functionality",
  nasa_osal_test_sequence_006_array
};
