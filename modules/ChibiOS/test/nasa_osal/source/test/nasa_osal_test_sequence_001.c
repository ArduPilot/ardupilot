/* Copyright statement.*/

#include "hal.h"
#include "nasa_osal_test_root.h"

/**
 * @file    nasa_osal_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page nasa_osal_test_sequence_001 [1] Tasks Functionality
 *
 * File: @ref nasa_osal_test_sequence_001.c
 *
 * <h2>Description</h2>
 * This sequence tests the NASA OSAL over ChibiOS/RT functionalities
 * related to threading.
 *
 * <h2>Test Cases</h2>
 * - @subpage nasa_osal_test_001_001
 * - @subpage nasa_osal_test_001_002
 * - @subpage nasa_osal_test_001_003
 * - @subpage nasa_osal_test_001_004
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "osapi.h"

static void test_task1(void) {

  test_emit_token('A');
}

static void test_task2(void) {

  test_emit_token('B');
}

static void test_task3(void) {

  test_emit_token('C');
}

static void test_task4(void) {

  test_emit_token('D');
}

static void delete_handler(void) {

  test_emit_token('C');
}

static void test_task_delete(void) {

  test_emit_token('A');
  (void) OS_TaskInstallDeleteHandler(delete_handler);
  while (!OS_TaskDeleteCheck()) {
    (void) OS_TaskDelay(1);
  }
  test_emit_token('B');
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nasa_osal_test_001_001 [1.1] OS_TaskCreate() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_TaskCreate() is tested.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] OS_TaskCreate() is invoked with task_id set to NULL, an
 *   error is expected.
 * - [1.1.2] OS_TaskCreate() is invoked with task_name set to NULL, an
 *   error is expected.
 * - [1.1.3] OS_TaskCreate() is invoked with stack_pointer set to NULL,
 *   an error is expected.
 * - [1.1.4] OS_TaskCreate() is invoked with a very long task name, an
 *   error is expected.
 * - [1.1.5] OS_TaskCreate() is invoked with priority below and above
 *   allowed range, an error is expected.
 * - [1.1.6] OS_TaskCreate() is invoked with a stack size below
 *   minimum, an error is expected.
 * - [1.1.7] OS_TaskCreate() is invoked twice with duplicated name and
 *   then duplicated stack, an error is expected in both cases.
 * .
 */

static void nasa_osal_test_001_001_execute(void) {

  /* [1.1.1] OS_TaskCreate() is invoked with task_id set to NULL, an
     error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TaskCreate(NULL,                   /* Error.*/
                        "failing task",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(1);

  /* [1.1.2] OS_TaskCreate() is invoked with task_name set to NULL, an
     error is expected.*/
  test_set_step(2);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        NULL,                   /* Error.*/
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(2);

  /* [1.1.3] OS_TaskCreate() is invoked with stack_pointer set to NULL,
     an error is expected.*/
  test_set_step(3);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        "failing task",
                        test_task1,
                        (uint32 *)NULL,         /* Error.*/
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(3);

  /* [1.1.4] OS_TaskCreate() is invoked with a very long task name, an
     error is expected.*/
  test_set_step(4);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        "this is a very very long task name", /* Error.*/
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(4);

  /* [1.1.5] OS_TaskCreate() is invoked with priority below and above
     allowed range, an error is expected.*/
  test_set_step(5);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        "failing task",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        0,                      /* Error.*/
                        0);
    test_assert(err == OS_ERR_INVALID_PRIORITY, "priority error not detected");
    test_assert_sequence("", "task executed");

    err = OS_TaskCreate(&tid,
                       "failing task",
                       test_task1,
                       (uint32 *)wa_test1,
                       sizeof wa_test1,
                       256,                     /* Error.*/
                       0);
    test_assert(err == OS_ERR_INVALID_PRIORITY, "priority error not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(5);

  /* [1.1.6] OS_TaskCreate() is invoked with a stack size below
     minimum, an error is expected.*/
  test_set_step(6);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        "failing task",
                        test_task1,
                        (uint32 *)wa_test1,
                        16,                     /* Error.*/
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_INVALID_INT_NUM, "stack insufficient size not detected");
    test_assert_sequence("", "task executed");
  }
  test_end_step(6);

  /* [1.1.7] OS_TaskCreate() is invoked twice with duplicated name and
     then duplicated stack, an error is expected in both cases.*/
  test_set_step(7);
  {
    int32 err;
    uint32 tid;

    err = OS_TaskCreate(&tid,
                        "running task",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_SUCCESS, "task creation failed");

    err = OS_TaskCreate(&tid,
                        "running task",
                        test_task2,
                        (uint32 *)wa_test2,
                        sizeof wa_test2,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_ERR_NAME_TAKEN, "name conflict not detected");

    err = OS_TaskCreate(&tid,
                        "conflicting task",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_ERR_NO_FREE_IDS, "stack conflict not detected");

    err = OS_TaskWait(tid);
    test_assert(err == OS_SUCCESS, "wait failed");
    test_assert_sequence("A", "task not executed");

    err = OS_TaskCreate(&tid,
                        "running task",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_SUCCESS, "task creation failed");

    err = OS_TaskWait(tid);
    test_assert(err == OS_SUCCESS, "wait failed");
    test_assert_sequence("A", "task not executed");
  }
  test_end_step(7);
}

static const testcase_t nasa_osal_test_001_001 = {
  "OS_TaskCreate() errors",
  NULL,
  NULL,
  nasa_osal_test_001_001_execute
};

/**
 * @page nasa_osal_test_001_002 [1.2] OS_TaskCreate() priority ordering
 *
 * <h2>Description</h2>
 * Four tasks are created at different priorities and in different
 * order. The execution order must happen in order of priority
 * regardless the creation order.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] Four tasks are created in priority order from low to high.
 * - [1.2.2] Tasks are made runnable atomically and their execution
 *   order tested.
 * - [1.2.3] Four tasks are created in priority order from high to low.
 * - [1.2.4] Tasks are made runnable atomically and their execution
 *   order tested.
 * - [1.2.5] Four tasks are created in an not ordered way.
 * - [1.2.6] Tasks are made runnable atomically and their execution
 *   order tested.
 * .
 */

static void nasa_osal_test_001_002_execute(void) {

  /* [1.2.1] Four tasks are created in priority order from low to
     high.*/
  test_set_step(1);
  {
    int32 err;
    uint32 tid1, tid2, tid3, tid4;

    err = OS_TaskCreate(&tid4,
                        "running task 4",
                        test_task4,
                        (uint32 *)wa_test4,
                        sizeof wa_test4,
                        TASKS_BASE_PRIORITY - 0,
                        0);
    test_assert(err == OS_SUCCESS, "task 4 creation failed");

    err = OS_TaskCreate(&tid3,
                        "running task 3",
                        test_task3,
                        (uint32 *)wa_test3,
                        sizeof wa_test3,
                        TASKS_BASE_PRIORITY - 1,
                        0);
    test_assert(err == OS_SUCCESS, "task 3 creation failed");

    err = OS_TaskCreate(&tid2,
                        "running task 2",
                        test_task2,
                        (uint32 *)wa_test2,
                        sizeof wa_test2,
                        TASKS_BASE_PRIORITY - 2,
                        0);
    test_assert(err == OS_SUCCESS, "task 2 creation failed");

    err = OS_TaskCreate(&tid1,
                        "running task 1",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY - 3,
                        0);
    test_assert(err == OS_SUCCESS, "task 1 creation failed");
  }
  test_end_step(1);

  /* [1.2.2] Tasks are made runnable atomically and their execution
     order tested.*/
  test_set_step(2);
  {
    OS_TaskDelay(5);
    test_assert_sequence("ABCD", "task order violation");
  }
  test_end_step(2);

  /* [1.2.3] Four tasks are created in priority order from high to
     low.*/
  test_set_step(3);
  {
    int32 err;
    uint32 tid1, tid2, tid3, tid4;

    err = OS_TaskCreate(&tid1,
                        "running task 1",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY - 3,
                        0);
    test_assert(err == OS_SUCCESS, "task 1 creation failed");

    err = OS_TaskCreate(&tid2,
                        "running task 2",
                        test_task2,
                        (uint32 *)wa_test2,
                        sizeof wa_test2,
                        TASKS_BASE_PRIORITY - 2,
                        0);
    test_assert(err == OS_SUCCESS, "task 2 creation failed");

    err = OS_TaskCreate(&tid3,
                        "running task 3",
                        test_task3,
                        (uint32 *)wa_test3,
                        sizeof wa_test3,
                        TASKS_BASE_PRIORITY - 1,
                        0);
    test_assert(err == OS_SUCCESS, "task 3 creation failed");

    err = OS_TaskCreate(&tid4,
                        "running task 4",
                        test_task4,
                        (uint32 *)wa_test4,
                        sizeof wa_test4,
                        TASKS_BASE_PRIORITY - 0,
                        0);
    test_assert(err == OS_SUCCESS, "task 4 creation failed");
  }
  test_end_step(3);

  /* [1.2.4] Tasks are made runnable atomically and their execution
     order tested.*/
  test_set_step(4);
  {
    OS_TaskDelay(5);
    test_assert_sequence("ABCD", "task order violation");
  }
  test_end_step(4);

  /* [1.2.5] Four tasks are created in an not ordered way.*/
  test_set_step(5);
  {
    int32 err;
    uint32 tid1, tid2, tid3, tid4;

    err = OS_TaskCreate(&tid2,
                        "running task 2",
                        test_task2,
                        (uint32 *)wa_test2,
                        sizeof wa_test2,
                        TASKS_BASE_PRIORITY - 2,
                        0);
    test_assert(err == OS_SUCCESS, "task 2 creation failed");

    err = OS_TaskCreate(&tid1,
                        "running task 1",
                        test_task1,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY - 3,
                        0);
    test_assert(err == OS_SUCCESS, "task 1 creation failed");

    err = OS_TaskCreate(&tid4,
                        "running task 4",
                        test_task4,
                        (uint32 *)wa_test4,
                        sizeof wa_test4,
                        TASKS_BASE_PRIORITY - 0,
                        0);
    test_assert(err == OS_SUCCESS, "task 4 creation failed");

    err = OS_TaskCreate(&tid3,
                        "running task 3",
                        test_task3,
                        (uint32 *)wa_test3,
                        sizeof wa_test3,
                        TASKS_BASE_PRIORITY - 1,
                        0);
    test_assert(err == OS_SUCCESS, "task 3 creation failed");
  }
  test_end_step(5);

  /* [1.2.6] Tasks are made runnable atomically and their execution
     order tested.*/
  test_set_step(6);
  {
    OS_TaskDelay(5);
    test_assert_sequence("ABCD", "task order violation");
  }
  test_end_step(6);
}

static const testcase_t nasa_osal_test_001_002 = {
  "OS_TaskCreate() priority ordering",
  NULL,
  NULL,
  nasa_osal_test_001_002_execute
};

/**
 * @page nasa_osal_test_001_003 [1.3] OS_TaskDelete() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_TaskDelete() is tested.
 *
 * <h2>Test Steps</h2>
 * - [1.3.1] OS_TaskDelete() is invoked with task_id set to -1, an
 *   error is expected.
 * .
 */

static void nasa_osal_test_001_003_execute(void) {

  /* [1.3.1] OS_TaskDelete() is invoked with task_id set to -1, an
     error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TaskDelete((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "wrong task id not detected");
  }
  test_end_step(1);
}

static const testcase_t nasa_osal_test_001_003 = {
  "OS_TaskDelete() errors",
  NULL,
  NULL,
  nasa_osal_test_001_003_execute
};

/**
 * @page nasa_osal_test_001_004 [1.4] OS_TaskDelete() and OS_TaskInstallDeleteHandler() functionality
 *
 * <h2>Description</h2>
 * OS_TaskDelete() and OS_TaskInstallDeleteHandler() are tested for
 * functionality.
 *
 * <h2>Test Steps</h2>
 * - [1.4.1] Creating a task executing an infinite loop.
 * - [1.4.2] Letting the task run for a while then deleting it. A check
 *   is performed on the correct execution of the delete handler.
 * .
 */

static void nasa_osal_test_001_004_execute(void) {
  uint32 tid;

  /* [1.4.1] Creating a task executing an infinite loop.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TaskCreate(&tid,
                        "deletable task",
                        test_task_delete,
                        (uint32 *)wa_test1,
                        sizeof wa_test1,
                        TASKS_BASE_PRIORITY,
                        0);
    test_assert(err == OS_SUCCESS, "deletable task creation failed");
  }
  test_end_step(1);

  /* [1.4.2] Letting the task run for a while then deleting it. A check
     is performed on the correct execution of the delete handler.*/
  test_set_step(2);
  {
    int32 err;

    (void) OS_TaskDelay(50);
    err = OS_TaskDelete(tid);
    test_assert(err == OS_SUCCESS, "delete failed");
    test_assert_sequence("ABC", "events order violation");
  }
  test_end_step(2);
}

static const testcase_t nasa_osal_test_001_004 = {
  "OS_TaskDelete() and OS_TaskInstallDeleteHandler() functionality",
  NULL,
  NULL,
  nasa_osal_test_001_004_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nasa_osal_test_sequence_001_array[] = {
  &nasa_osal_test_001_001,
  &nasa_osal_test_001_002,
  &nasa_osal_test_001_003,
  &nasa_osal_test_001_004,
  NULL
};

/**
 * @brief   Tasks Functionality.
 */
const testsequence_t nasa_osal_test_sequence_001 = {
  "Tasks Functionality",
  nasa_osal_test_sequence_001_array
};
