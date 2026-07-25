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
#include "oslib_test_root.h"

/**
 * @file    oslib_test_sequence_004.c
 * @brief   Test Sequence 004 code.
 *
 * @page oslib_test_sequence_004 [4] Jobs Queues
 *
 * File: @ref oslib_test_sequence_004.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * Jobs Queues.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_JOBS == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_004_001
 * .
 */

#if (CH_CFG_USE_JOBS == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#define JOBS_QUEUE_SIZE 4

static jobs_queue_t jq;
static job_descriptor_t jobs[JOBS_QUEUE_SIZE];
static msg_t msg_queue[JOBS_QUEUE_SIZE];

static void job_slow(void *arg) {

  test_emit_token((int)arg);
  chThdSleepMilliseconds(10);
}

static THD_WORKING_AREA(wa1Thread1, 256);
static THD_WORKING_AREA(wa2Thread1, 256);
static THD_FUNCTION(Thread1, arg) {
  msg_t msg;

  (void)arg;

  do {
    msg = chJobDispatch(&jq);
  } while (msg == MSG_OK);
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page oslib_test_004_001 [4.1] Dispatcher test
 *
 * <h2>Description</h2>
 * The dispatcher API is tested for functionality.
 *
 * <h2>Test Steps</h2>
 * - [4.1.1] Initializing the Jobs Queue object.
 * - [4.1.2] Starting the dispatcher threads.
 * - [4.1.3] Sending jobs with various timings.
 * - [4.1.4] Sending two null jobs to make threads exit.
 * - [4.1.5] Verifying that all descriptors have been returned.
 * .
 */

static void oslib_test_004_001_execute(void) {
  thread_t *tp1, *tp2;

  /* [4.1.1] Initializing the Jobs Queue object.*/
  test_set_step(1);
  {
    chJobObjectInit(&jq, JOBS_QUEUE_SIZE, jobs, msg_queue);
  }
  test_end_step(1);

  /* [4.1.2] Starting the dispatcher threads.*/
  test_set_step(2);
  {
    thread_descriptor_t td1 = {
      .name  = "dispatcher1",
      .wbase = wa1Thread1,
      .wend  = THD_WORKING_AREA_END(wa1Thread1),
      .prio  = chThdGetPriorityX() - 1,
      .funcp = Thread1,
      .arg   = NULL
    };
    tp1 = chThdCreate(&td1);

    thread_descriptor_t td2 = {
      .name  = "dispatcher2",
      .wbase = wa2Thread1,
      .wend  = THD_WORKING_AREA_END(wa2Thread1),
      .prio  = chThdGetPriorityX() - 2,
      .funcp = Thread1,
      .arg   = NULL
    };
    tp2 = chThdCreate(&td2);
  }
  test_end_step(2);

  /* [4.1.3] Sending jobs with various timings.*/
  test_set_step(3);
  {
    unsigned i;
    job_descriptor_t *jdp;

    for (i = 0; i < 8; i++) {
      jdp = chJobGet(&jq);
      jdp->jobfunc = job_slow;
      jdp->jobarg  = (void *)('a' + i);
      chJobPost(&jq, jdp);
    }
  }
  test_end_step(3);

  /* [4.1.4] Sending two null jobs to make threads exit.*/
  test_set_step(4);
  {
    job_descriptor_t *jdp;

    jdp = chJobGet(&jq);
    jdp->jobfunc = NULL;
    jdp->jobarg  = NULL;
    chJobPost(&jq, jdp);
    jdp = chJobGet(&jq);
    jdp->jobfunc = NULL;
    jdp->jobarg  = NULL;
    chJobPost(&jq, jdp);
    (void) chThdWait(tp1);
    (void) chThdWait(tp2);
    test_assert_sequence("abcdefgh", "unexpected tokens");
  }
  test_end_step(4);

  /* [4.1.5] Verifying that all descriptors have been returned.*/
  test_set_step(5);
  {
    cnt_t cnt;
    size_t used;

    chSysLock();
    cnt = chGuardedPoolGetCounterI(&jq.free);
    used = chMBGetUsedCountI(&jq.mbx);
    chSysUnlock();

    test_assert(cnt == (cnt_t)JOBS_QUEUE_SIZE, "lost descriptors");
    test_assert(used == 0U, "pending jobs");
  }
  test_end_step(5);
}

static const testcase_t oslib_test_004_001 = {
  "Dispatcher test",
  NULL,
  NULL,
  oslib_test_004_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_004_array[] = {
  &oslib_test_004_001,
  NULL
};

/**
 * @brief   Jobs Queues.
 */
const testsequence_t oslib_test_sequence_004 = {
  "Jobs Queues",
  oslib_test_sequence_004_array
};

#endif /* CH_CFG_USE_JOBS == TRUE */
