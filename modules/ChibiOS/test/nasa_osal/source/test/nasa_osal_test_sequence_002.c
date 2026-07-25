/* Copyright statement.*/

#include "hal.h"
#include "nasa_osal_test_root.h"

/**
 * @file    nasa_osal_test_sequence_002.c
 * @brief   Test Sequence 002 code.
 *
 * @page nasa_osal_test_sequence_002 [2] Queues Functionality
 *
 * File: @ref nasa_osal_test_sequence_002.c
 *
 * <h2>Description</h2>
 * This sequence tests the NASA OSAL over ChibiOS/RT functionalities
 * related to queues.
 *
 * <h2>Test Cases</h2>
 * - @subpage nasa_osal_test_002_001
 * - @subpage nasa_osal_test_002_002
 * - @subpage nasa_osal_test_002_003
 * - @subpage nasa_osal_test_002_004
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

#include "osapi.h"

uint32 qid, tid;

#define WRITER_NUM_MESSAGES 16
#define MESSAGE_SIZE        20

static void test_task_writer(void) {
  unsigned i;
  int32 err;

  for (i = 0; i < WRITER_NUM_MESSAGES; i++) {
    err = OS_QueuePut(qid, "Hello World", 12, 0);
    if (err != OS_SUCCESS) {
      test_emit_token('*');
    }
  }
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nasa_osal_test_002_001 [2.1] OS_QueueCreate() and OS_QueueDelete() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_QueueCreate() and OS_QueueDelete() is
 * tested.
 *
 * <h2>Test Steps</h2>
 * - [2.1.1] OS_QueueCreate() is invoked with queue_id set to NULL, an
 *   error is expected.
 * - [2.1.2] OS_QueueCreate() is invoked with task_name set to NULL, an
 *   error is expected.
 * - [2.1.3] OS_QueueCreate() is invoked with a very long task name, an
 *   error is expected.
 * - [2.1.4] OS_QueueDelete() is invoked with queue_id set to -1, an
 *   error is expected.
 * - [2.1.5] OS_QueueCreate() is invoked twice with duplicated name, an
 *   error is expected, then the queue is deleted using
 *   OS_QueueDelete().
 * .
 */

static void nasa_osal_test_002_001_execute(void) {

  /* [2.1.1] OS_QueueCreate() is invoked with queue_id set to NULL, an
     error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_QueueCreate(NULL,                      /* Error.*/
                         "failing queue",
                         4,
                         128,
                         0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [2.1.2] OS_QueueCreate() is invoked with task_name set to NULL, an
     error is expected.*/
  test_set_step(2);
  {
    int32 err;
    uint32 qid;

    err = OS_QueueCreate(&qid,
                         NULL,                      /* Error.*/
                         4,
                         128,
                         0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [2.1.3] OS_QueueCreate() is invoked with a very long task name, an
     error is expected.*/
  test_set_step(3);
  {
    int32 err;
    uint32 qid;

    err = OS_QueueCreate(&qid,
                         "very very long queue name",   /* Error.*/
                         4,
                         128,
                         0);
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
  }
  test_end_step(3);

  /* [2.1.4] OS_QueueDelete() is invoked with queue_id set to -1, an
     error is expected.*/
  test_set_step(4);
  {
    int32 err;

    err = OS_QueueDelete((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "wrong queue id not detected");
  }
  test_end_step(4);

  /* [2.1.5] OS_QueueCreate() is invoked twice with duplicated name, an
     error is expected, then the queue is deleted using
     OS_QueueDelete().*/
  test_set_step(5);
  {
    int32 err;
    uint32 qid1, qid2;

    err = OS_QueueCreate(&qid1, "my queue", 4, 128, 0);
    test_assert(err == OS_SUCCESS, "queue creation failed");

    err = OS_QueueCreate(&qid2, "my queue", 4, 128, 0);
    test_assert(err == OS_ERR_NAME_TAKEN, "name conflict not detected");

    err = OS_QueueDelete(qid1);
    test_assert(err == OS_SUCCESS, "queue deletion failed");
  }
  test_end_step(5);
}

static const testcase_t nasa_osal_test_002_001 = {
  "OS_QueueCreate() and OS_QueueDelete() errors",
  NULL,
  NULL,
  nasa_osal_test_002_001_execute
};

/**
 * @page nasa_osal_test_002_002 [2.2] OS_QueueGetIdByName() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_QueueGetIdByName() is tested.
 *
 * <h2>Test Steps</h2>
 * - [2.2.1] OS_QueueGetIdByName() is invoked with queue_id set to
 *   NULL, an error is expected.
 * - [2.2.2] OS_QueueGetIdByName() is invoked with queue_name set to
 *   NULL, an error is expected.
 * - [2.2.3] OS_QueueGetIdByName() is invoked with a very long task
 *   name, an error is expected.
 * .
 */

static void nasa_osal_test_002_002_execute(void) {

  /* [2.2.1] OS_QueueGetIdByName() is invoked with queue_id set to
     NULL, an error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_QueueGetIdByName(NULL, "queue");
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [2.2.2] OS_QueueGetIdByName() is invoked with queue_name set to
     NULL, an error is expected.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_QueueGetIdByName(&qid, NULL);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [2.2.3] OS_QueueGetIdByName() is invoked with a very long task
     name, an error is expected.*/
  test_set_step(3);
  {
    int32 err;

    err = OS_QueueGetIdByName(&qid, "very very long queue name");
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
  }
  test_end_step(3);
}

static const testcase_t nasa_osal_test_002_002 = {
  "OS_QueueGetIdByName() errors",
  NULL,
  NULL,
  nasa_osal_test_002_002_execute
};

/**
 * @page nasa_osal_test_002_003 [2.3] OS_QueuePut() and OS_QueueGet() functionality
 *
 * <h2>Description</h2>
 * A task writes on a queue, the messages are retrieved on the other
 * side in blocking mode.
 *
 * <h2>Test Steps</h2>
 * - [2.3.1] Creataing a queue with depth 4 and message size 20.
 * - [2.3.2] Creating the writer task.
 * - [2.3.3] Reading messages from the writer task.
 * - [2.3.4] Waiting for task termination then checking for errors.
 * .
 */

static void nasa_osal_test_002_003_setup(void) {
  qid = 0;
  tid = 0;
}

static void nasa_osal_test_002_003_teardown(void) {
  if (qid != 0) {
    (void) OS_QueueDelete(qid);
  }

  if (tid != 0) {
    (void) OS_TaskWait(tid);
  }
}

static void nasa_osal_test_002_003_execute(void) {
  uint32 tid;
  unsigned i;

  /* [2.3.1] Creataing a queue with depth 4 and message size 20.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_QueueCreate(&qid, "test queue", 4, MESSAGE_SIZE, 0);
    test_assert(err == OS_SUCCESS, "queue creation failed");
  }
  test_end_step(1);

  /* [2.3.2] Creating the writer task.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_TaskCreate(&tid,
                        "writer task",
                        test_task_writer,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_SUCCESS, "writer task creation failed");
  }
  test_end_step(2);

  /* [2.3.3] Reading messages from the writer task.*/
  test_set_step(3);
  {
    for (i = 0; i < WRITER_NUM_MESSAGES; i++) {
      int32 err;
      char data[MESSAGE_SIZE];
      uint32 copied;

      err = OS_QueueGet(qid, data, MESSAGE_SIZE, &copied, OS_Milli2Ticks(200));
      test_assert(err == OS_SUCCESS, "timed out");
      test_assert(strncmp(data, "Hello World", sizeof (data)) == 0,
                  "wrong message");
    }
  }
  test_end_step(3);

  /* [2.3.4] Waiting for task termination then checking for errors.*/
  test_set_step(4);
  {
    (void) OS_TaskWait(tid);
    tid = 0;
    test_assert_sequence("", "queue write errors occurred");
  }
  test_end_step(4);
}

static const testcase_t nasa_osal_test_002_003 = {
  "OS_QueuePut() and OS_QueueGet() functionality",
  nasa_osal_test_002_003_setup,
  nasa_osal_test_002_003_teardown,
  nasa_osal_test_002_003_execute
};

/**
 * @page nasa_osal_test_002_004 [2.4] OS_QueueGet() with timeout functionality
 *
 * <h2>Description</h2>
 * OS_QueueGet() timeout functionality is tested.
 *
 * <h2>Test Steps</h2>
 * - [2.4.1] Retrieving the queue by name.
 * - [2.4.2] Get operation with a one second timeout, an error is
 *   expected.
 * - [2.4.3] Get operation in non-blocking mode, an error is expected.
 * .
 */

static void nasa_osal_test_002_004_setup(void) {
  qid = 0;
  (void) OS_QueueCreate(&qid, "test queue", 2, MESSAGE_SIZE, 0);
}

static void nasa_osal_test_002_004_teardown(void) {
  if (qid != 0) {
    OS_QueueDelete(qid);
  }
}

static void nasa_osal_test_002_004_execute(void) {
  uint32 local_qid;
  uint32 copied;
  char data[MESSAGE_SIZE];

  /* [2.4.1] Retrieving the queue by name.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_QueueGetIdByName(&local_qid, "test queue");
    test_assert(err == OS_SUCCESS, "queue not found");
  }
  test_end_step(1);

  /* [2.4.2] Get operation with a one second timeout, an error is
     expected.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_QueueGet(qid, data, MESSAGE_SIZE, &copied, OS_Milli2Ticks(1000));
    test_assert(err == OS_QUEUE_TIMEOUT, "unexpected error code");
  }
  test_end_step(2);

  /* [2.4.3] Get operation in non-blocking mode, an error is
     expected.*/
  test_set_step(3);
  {
    int32 err;

    err = OS_QueueGet(qid, data, MESSAGE_SIZE, &copied, OS_CHECK);
    test_assert(err == OS_QUEUE_EMPTY, "unexpected error code");
  }
  test_end_step(3);
}

static const testcase_t nasa_osal_test_002_004 = {
  "OS_QueueGet() with timeout functionality",
  nasa_osal_test_002_004_setup,
  nasa_osal_test_002_004_teardown,
  nasa_osal_test_002_004_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nasa_osal_test_sequence_002_array[] = {
  &nasa_osal_test_002_001,
  &nasa_osal_test_002_002,
  &nasa_osal_test_002_003,
  &nasa_osal_test_002_004,
  NULL
};

/**
 * @brief   Queues Functionality.
 */
const testsequence_t nasa_osal_test_sequence_002 = {
  "Queues Functionality",
  nasa_osal_test_sequence_002_array
};
