/* Copyright statement.*/

#include "hal.h"
#include "nasa_osal_test_root.h"

/**
 * @file    nasa_osal_test_sequence_003.c
 * @brief   Test Sequence 003 code.
 *
 * @page nasa_osal_test_sequence_003 [3] Timers Functionality
 *
 * File: @ref nasa_osal_test_sequence_003.c
 *
 * <h2>Description</h2>
 * This sequence tests the NASA OSAL over ChibiOS/RT functionalities
 * related to timers.
 *
 * <h2>Test Cases</h2>
 * - @subpage nasa_osal_test_003_001
 * - @subpage nasa_osal_test_003_002
 * - @subpage nasa_osal_test_003_003
 * - @subpage nasa_osal_test_003_004
 * - @subpage nasa_osal_test_003_005
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

#include "osapi.h"

uint32 tmid;
uint32 cnt;

static void tmr_callback(uint32 timer_id) {

  (void)timer_id;

  cnt++;
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nasa_osal_test_003_001 [3.1] OS_TimerCreate() and OS_TimerDelete() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_TimerCreate() and OS_TimerDelete() is
 * tested.
 *
 * <h2>Test Steps</h2>
 * - [3.1.1] OS_TimerCreate() is invoked with timer_id set to NULL, an
 *   error is expected.
 * - [3.1.2] OS_TimerCreate() is invoked with timer_name set to NULL,
 *   an error is expected.
 * - [3.1.3] OS_TimerCreate() is invoked with accuracy set to NULL, an
 *   error is expected.
 * - [3.1.4] OS_TimerCreate() is invoked with callback_ptr set to NULL,
 *   an error is expected.
 * - [3.1.5] OS_TimerCreate() is invoked with a very long timer name,
 *   an error is expected.
 * - [3.1.6] OS_TimerDelete() is invoked with timer_id set to -1, an
 *   error is expected.
 * - [3.1.7] OS_TimerCreate() is invoked twice with duplicated name, an
 *   error is expected, then the queue is deleted using
 *   OS_TimerDelete().
 * .
 */

static void nasa_osal_test_003_001_execute(void) {

  /* [3.1.1] OS_TimerCreate() is invoked with timer_id set to NULL, an
     error is expected.*/
  test_set_step(1);
  {
    int32 err;
    uint32 accuracy;

    err = OS_TimerCreate(NULL,                      /* Error.*/
                         "failing timer",
                         &accuracy,
                         tmr_callback);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [3.1.2] OS_TimerCreate() is invoked with timer_name set to NULL,
     an error is expected.*/
  test_set_step(2);
  {
    int32 err;
    uint32 tmid;
    uint32 accuracy;

    err = OS_TimerCreate(&tmid,
                         NULL,                      /* Error.*/
                         &accuracy,
                         tmr_callback);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [3.1.3] OS_TimerCreate() is invoked with accuracy set to NULL, an
     error is expected.*/
  test_set_step(3);
  {
    int32 err;
    uint32 tmid;

    err = OS_TimerCreate(&tmid,
                         "failing timer",
                         NULL,                      /* Error.*/
                         tmr_callback);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(3);

  /* [3.1.4] OS_TimerCreate() is invoked with callback_ptr set to NULL,
     an error is expected.*/
  test_set_step(4);
  {
    int32 err;
    uint32 tmid;
    uint32 accuracy;

    err = OS_TimerCreate(&tmid,
                         "failing timer",
                         &accuracy,
                         NULL);                     /* Error.*/
    test_assert(err == OS_TIMER_ERR_INVALID_ARGS, "NULL not detected");
  }
  test_end_step(4);

  /* [3.1.5] OS_TimerCreate() is invoked with a very long timer name,
     an error is expected.*/
  test_set_step(5);
  {
    int32 err;
    uint32 tmid;
    uint32 accuracy;

    err = OS_TimerCreate(&tmid,
                         "very very long timer name",   /* Error.*/
                         &accuracy,
                         tmr_callback);
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
  }
  test_end_step(5);

  /* [3.1.6] OS_TimerDelete() is invoked with timer_id set to -1, an
     error is expected.*/
  test_set_step(6);
  {
    int32 err;

    err = OS_TimerDelete((uint32)-1);
    test_assert(err == OS_ERR_INVALID_ID, "wrong timer id not detected");
  }
  test_end_step(6);

  /* [3.1.7] OS_TimerCreate() is invoked twice with duplicated name, an
     error is expected, then the queue is deleted using
     OS_TimerDelete().*/
  test_set_step(7);
  {
    int32 err;
    uint32 tmid1, tmid2;
    uint32 accuracy;

    err = OS_TimerCreate(&tmid1, "my timer", &accuracy, tmr_callback);
    test_assert(err == OS_SUCCESS, "timer creation failed");

    err = OS_TimerCreate(&tmid2, "my timer", &accuracy, tmr_callback);
    test_assert(err == OS_ERR_NAME_TAKEN, "name conflict not detected");

    err = OS_TimerDelete(tmid1);
    test_assert(err == OS_SUCCESS, "timer deletion failed");
  }
  test_end_step(7);
}

static const testcase_t nasa_osal_test_003_001 = {
  "OS_TimerCreate() and OS_TimerDelete() errors",
  NULL,
  NULL,
  nasa_osal_test_003_001_execute
};

/**
 * @page nasa_osal_test_003_002 [3.2] OS_TimerSet() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_TimerSet() is tested.
 *
 * <h2>Test Steps</h2>
 * - [3.2.1] OS_TimerSet() is invoked with timer_id set to -1, an error
 *   is expected.
 * .
 */

static void nasa_osal_test_003_002_execute(void) {

  /* [3.2.1] OS_TimerSet() is invoked with timer_id set to -1, an error
     is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TimerSet((uint32)-1, 10, 10);
    test_assert(err == OS_ERR_INVALID_ID, "invalid timer_id not detected");
  }
  test_end_step(1);
}

static const testcase_t nasa_osal_test_003_002 = {
  "OS_TimerSet() errors",
  NULL,
  NULL,
  nasa_osal_test_003_002_execute
};

/**
 * @page nasa_osal_test_003_003 [3.3] OS_TimerGetIdByName() errors
 *
 * <h2>Description</h2>
 * Parameters checking in OS_TimerGetIdByName() is tested.
 *
 * <h2>Test Steps</h2>
 * - [3.3.1] OS_TimerGetIdByName() is invoked with timer_id set to
 *   NULL, an error is expected.
 * - [3.3.2] OS_TimerGetIdByName() is invoked with timer name set to
 *   NULL, an error is expected.
 * - [3.3.3] OS_TimerGetIdByName() is invoked with a very long task
 *   name, an error is expected.
 * .
 */

static void nasa_osal_test_003_003_execute(void) {

  /* [3.3.1] OS_TimerGetIdByName() is invoked with timer_id set to
     NULL, an error is expected.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TimerGetIdByName(NULL, "timer");
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(1);

  /* [3.3.2] OS_TimerGetIdByName() is invoked with timer name set to
     NULL, an error is expected.*/
  test_set_step(2);
  {
    int32 err;

    err = OS_TimerGetIdByName(&tmid, NULL);
    test_assert(err == OS_INVALID_POINTER, "NULL not detected");
  }
  test_end_step(2);

  /* [3.3.3] OS_TimerGetIdByName() is invoked with a very long task
     name, an error is expected.*/
  test_set_step(3);
  {
    int32 err;

    err = OS_TimerGetIdByName(&tmid, "very very long timer name");
    test_assert(err == OS_ERR_NAME_TOO_LONG, "name limit not detected");
  }
  test_end_step(3);
}

static const testcase_t nasa_osal_test_003_003 = {
  "OS_TimerGetIdByName() errors",
  NULL,
  NULL,
  nasa_osal_test_003_003_execute
};

/**
 * @page nasa_osal_test_003_004 [3.4] OS_TimerSet() one-shot functionality
 *
 * <h2>Description</h2>
 * A timer is tested in one-shot mode.
 *
 * <h2>Test Steps</h2>
 * - [3.4.1] Retrieving the timer by name.
 * - [3.4.2] Setting up the timer for a 70mS one-shot tick.
 * - [3.4.3] Waiting one second then counting the occurred ticks.
 * .
 */

static void nasa_osal_test_003_004_setup(void) {
  uint32 accuracy;

  cnt = 0;
  tmid = 0;
  (void) OS_TimerCreate(&tmid, "test timer", &accuracy, tmr_callback);
}

static void nasa_osal_test_003_004_teardown(void) {
  if (tmid != 0) {
    (void) OS_TimerDelete(tmid);
  }
}

static void nasa_osal_test_003_004_execute(void) {
  uint32 local_tmid;

  /* [3.4.1] Retrieving the timer by name.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TimerGetIdByName(&local_tmid, "test timer");
    test_assert(err == OS_SUCCESS, "timer not found");
  }
  test_end_step(1);

  /* [3.4.2] Setting up the timer for a 70mS one-shot tick.*/
  test_set_step(2);
  {
    uint32 err;

    err = OS_TimerSet(local_tmid, 70000, 0);
    test_assert(err == OS_SUCCESS, "timer setup failed");
  }
  test_end_step(2);

  /* [3.4.3] Waiting one second then counting the occurred ticks.*/
  test_set_step(3);
  {
    (void) OS_TaskDelay(1000);
    test_assert(cnt == 1, "wrong ticks");
  }
  test_end_step(3);
}

static const testcase_t nasa_osal_test_003_004 = {
  "OS_TimerSet() one-shot functionality",
  nasa_osal_test_003_004_setup,
  nasa_osal_test_003_004_teardown,
  nasa_osal_test_003_004_execute
};

/**
 * @page nasa_osal_test_003_005 [3.5] OS_TimerSet() periodic functionality
 *
 * <h2>Description</h2>
 * A timer is tested in periodic mode.
 *
 * <h2>Test Steps</h2>
 * - [3.5.1] Retrieving the timer by name.
 * - [3.5.2] Setting up the timer for a 70mS periodic tick.
 * - [3.5.3] Waiting one second then counting the occurred ticks.
 * - [3.5.4] Stopping the timer.
 * .
 */

static void nasa_osal_test_003_005_setup(void) {
  uint32 accuracy;

  cnt = 0;
  tmid = 0;
  (void) OS_TimerCreate(&tmid, "test timer", &accuracy, tmr_callback);
}

static void nasa_osal_test_003_005_teardown(void) {
  if (tmid != 0) {
    (void) OS_TimerSet(tmid, 0, 0);
    (void) OS_TimerDelete(tmid);
  }
}

static void nasa_osal_test_003_005_execute(void) {
  uint32 local_tmid;

  /* [3.5.1] Retrieving the timer by name.*/
  test_set_step(1);
  {
    int32 err;

    err = OS_TimerGetIdByName(&local_tmid, "test timer");
    test_assert(err == OS_SUCCESS, "timer not found");
  }
  test_end_step(1);

  /* [3.5.2] Setting up the timer for a 70mS periodic tick.*/
  test_set_step(2);
  {
    uint32 err;

    err = OS_TimerSet(local_tmid, 70000, 70000);
    test_assert(err == OS_SUCCESS, "timer setup failed");
  }
  test_end_step(2);

  /* [3.5.3] Waiting one second then counting the occurred ticks.*/
  test_set_step(3);
  {
    (void) OS_TaskDelay(1000);
    test_assert(cnt == 14, "wrong ticks");
  }
  test_end_step(3);

  /* [3.5.4] Stopping the timer.*/
  test_set_step(4);
  {
    uint32 err;

    err = OS_TimerSet(local_tmid, 0, 0);
    test_assert(err == OS_SUCCESS, "timer stop failed");
  }
  test_end_step(4);
}

static const testcase_t nasa_osal_test_003_005 = {
  "OS_TimerSet() periodic functionality",
  nasa_osal_test_003_005_setup,
  nasa_osal_test_003_005_teardown,
  nasa_osal_test_003_005_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nasa_osal_test_sequence_003_array[] = {
  &nasa_osal_test_003_001,
  &nasa_osal_test_003_002,
  &nasa_osal_test_003_003,
  &nasa_osal_test_003_004,
  &nasa_osal_test_003_005,
  NULL
};

/**
 * @brief   Timers Functionality.
 */
const testsequence_t nasa_osal_test_sequence_003 = {
  "Timers Functionality",
  nasa_osal_test_sequence_003_array
};
