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
#include "rt_test_root.h"

/**
 * @file    rt_test_sequence_011.c
 * @brief   Test Sequence 011 code.
 *
 * @page rt_test_sequence_011 [11] Dynamic threads
 *
 * File: @ref rt_test_sequence_011.c
 *
 * <h2>Description</h2>
 * This module implements the test sequence for the dynamic thread
 * creation APIs.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_DYNAMIC == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage rt_test_011_001
 * - @subpage rt_test_011_002
 * .
 */

#if (CH_CFG_USE_DYNAMIC == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#if CH_CFG_USE_HEAP
static memory_heap_t heap1;
#endif
#if CH_CFG_USE_MEMPOOLS
static memory_pool_t mp1;
#endif

static THD_FUNCTION(dyn_thread1, p) {

  test_emit_token(*(char *)p);
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_011_001 [11.1] Threads creation from Memory Heap
 *
 * <h2>Description</h2>
 * Two threads are started by allocating the memory from the Memory
 * Heap then a third thread is started with a huge stack
 * requirement.<br> The test expects the first two threads to
 * successfully start and the third one to fail.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_HEAP == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [11.1.1] Getting base priority for threads.
 * - [11.1.2] Getting heap info before the test.
 * - [11.1.3] Creating thread 1, it is expected to succeed.
 * - [11.1.4] Creating thread 2, it is expected to succeed.
 * - [11.1.5] Creating thread 3, it is expected to fail.
 * - [11.1.6] Letting threads execute then checking the start order and
 *   freeing memory.
 * - [11.1.7] Getting heap info again for verification.
 * .
 */

static void rt_test_011_001_setup(void) {
  chHeapObjectInit(&heap1, test_buffer, sizeof test_buffer);
}

static void rt_test_011_001_execute(void) {
  size_t n1, total1, largest1;
  size_t n2, total2, largest2;
  tprio_t prio;

  /* [11.1.1] Getting base priority for threads.*/
  test_set_step(1);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(1);

  /* [11.1.2] Getting heap info before the test.*/
  test_set_step(2);
  {
    n1 = chHeapStatus(&heap1, &total1, &largest1);
    test_assert(n1 == 1, "heap fragmented");
  }
  test_end_step(2);

  /* [11.1.3] Creating thread 1, it is expected to succeed.*/
  test_set_step(3);
  {
    threads[0] = chThdCreateFromHeap(&heap1,
                                     THD_WORKING_AREA_SIZE(THREADS_STACK_SIZE),
                                     "dyn1",
                                     prio-1, dyn_thread1, "A");
    test_assert(threads[0] != NULL, "thread creation failed");
  }
  test_end_step(3);

  /* [11.1.4] Creating thread 2, it is expected to succeed.*/
  test_set_step(4);
  {
    threads[1] = chThdCreateFromHeap(&heap1,
                                     THD_WORKING_AREA_SIZE(THREADS_STACK_SIZE),
                                     "dyn2",
                                     prio-2, dyn_thread1, "B");
    test_assert(threads[1] != NULL, "thread creation failed");
  }
  test_end_step(4);

  /* [11.1.5] Creating thread 3, it is expected to fail.*/
  test_set_step(5);
  {
    threads[2] = chThdCreateFromHeap(&heap1,
                                     (((size_t)-1) >> 1U) + 1U,
                                     "dyn3",
                                     prio-3, dyn_thread1, "C");
    test_assert(threads[2] == NULL, "thread creation not failed");
  }
  test_end_step(5);

  /* [11.1.6] Letting threads execute then checking the start order and
     freeing memory.*/
  test_set_step(6);
  {
    test_wait_threads();
    test_assert_sequence("AB", "invalid sequence");
  }
  test_end_step(6);

  /* [11.1.7] Getting heap info again for verification.*/
  test_set_step(7);
  {
    n2 = chHeapStatus(&heap1, &total2, &largest2);
    test_assert(n1 == n2, "fragmentation changed");
    test_assert(total1 == total2, "total free space changed");
    test_assert(largest1 == largest2, "largest fragment size changed");
  }
  test_end_step(7);
}

static const testcase_t rt_test_011_001 = {
  "Threads creation from Memory Heap",
  rt_test_011_001_setup,
  NULL,
  rt_test_011_001_execute
};
#endif /* CH_CFG_USE_HEAP == TRUE */

#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)
/**
 * @page rt_test_011_002 [11.2] Threads creation from Memory Pool
 *
 * <h2>Description</h2>
 * Five thread creation are attempted from a pool containing only four
 * elements.<br> The test expects the first four threads to
 * successfully start and the last one to fail.
 *
 * <h2>Conditions</h2>
 * This test is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_MEMPOOLS == TRUE
 * .
 *
 * <h2>Test Steps</h2>
 * - [11.2.1] Adding four working areas to the pool.
 * - [11.2.2] Getting base priority for threads.
 * - [11.2.3] Creating the five threads.
 * - [11.2.4] Testing that only the fifth thread creation failed.
 * - [11.2.5] Letting them run, free the memory then checking the
 *   execution sequence.
 * - [11.2.6] Testing that the pool contains four elements again.
 * .
 */

static void rt_test_011_002_setup(void) {
  chPoolObjectInit(&mp1, THD_WORKING_AREA_SIZE(THREADS_STACK_SIZE), NULL);
}

static void rt_test_011_002_execute(void) {
  unsigned i;
  tprio_t prio;

  /* [11.2.1] Adding four working areas to the pool.*/
  test_set_step(1);
  {
    for (i = 0; i < 4; i++)
      chPoolFree(&mp1, wa[i]);
  }
  test_end_step(1);

  /* [11.2.2] Getting base priority for threads.*/
  test_set_step(2);
  {
    prio = chThdGetPriorityX();
  }
  test_end_step(2);

  /* [11.2.3] Creating the five threads.*/
  test_set_step(3);
  {
    threads[0] = chThdCreateFromMemoryPool(&mp1, "dyn1", prio-1, dyn_thread1, "A");
    threads[1] = chThdCreateFromMemoryPool(&mp1, "dyn2", prio-2, dyn_thread1, "B");
    threads[2] = chThdCreateFromMemoryPool(&mp1, "dyn3", prio-3, dyn_thread1, "C");
    threads[3] = chThdCreateFromMemoryPool(&mp1, "dyn4", prio-4, dyn_thread1, "D");
    threads[4] = chThdCreateFromMemoryPool(&mp1, "dyn5", prio-5, dyn_thread1, "E");
  }
  test_end_step(3);

  /* [11.2.4] Testing that only the fifth thread creation failed.*/
  test_set_step(4);
  {
    test_assert((threads[0] != NULL) &&
                (threads[1] != NULL) &&
                (threads[2] != NULL) &&
                (threads[3] != NULL),
                "thread creation failed");
    test_assert(threads[4] == NULL,
                "thread creation not failed");
  }
  test_end_step(4);

  /* [11.2.5] Letting them run, free the memory then checking the
     execution sequence.*/
  test_set_step(5);
  {
    test_wait_threads();
    test_assert_sequence("ABCD", "invalid sequence");
  }
  test_end_step(5);

  /* [11.2.6] Testing that the pool contains four elements again.*/
  test_set_step(6);
  {
    for (i = 0; i < 4; i++)
      test_assert(chPoolAlloc(&mp1) != NULL, "pool list empty");
    test_assert(chPoolAlloc(&mp1) == NULL, "pool list not empty");
  }
  test_end_step(6);
}

static const testcase_t rt_test_011_002 = {
  "Threads creation from Memory Pool",
  rt_test_011_002_setup,
  NULL,
  rt_test_011_002_execute
};
#endif /* CH_CFG_USE_MEMPOOLS == TRUE */

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const rt_test_sequence_011_array[] = {
#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)
  &rt_test_011_001,
#endif
#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)
  &rt_test_011_002,
#endif
  NULL
};

/**
 * @brief   Dynamic threads.
 */
const testsequence_t rt_test_sequence_011 = {
  "Dynamic threads",
  rt_test_sequence_011_array
};

#endif /* CH_CFG_USE_DYNAMIC == TRUE */
