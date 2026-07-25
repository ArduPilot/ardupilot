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
 * @file    oslib_test_sequence_008.c
 * @brief   Test Sequence 008 code.
 *
 * @page oslib_test_sequence_008 [8] Memory Heaps
 *
 * File: @ref oslib_test_sequence_008.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * memory heaps.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_HEAP == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_008_001
 * - @subpage oslib_test_008_002
 * .
 */

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#define ALLOC_SIZE 16
#define HEAP_SIZE (ALLOC_SIZE * 8)

static memory_heap_t test_heap;
static uint8_t test_heap_buffer[HEAP_SIZE];

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page oslib_test_008_001 [8.1] Allocation and fragmentation
 *
 * <h2>Description</h2>
 * Series of allocations/deallocations are performed in carefully
 * designed sequences in order to stimulate all the possible code paths
 * inside the allocator. The test expects to find the heap back to the
 * initial status after each sequence.
 *
 * <h2>Test Steps</h2>
 * - [8.1.1] Testing initial conditions, the heap must not be
 *   fragmented and one free block present.
 * - [8.1.2] Trying to allocate an block bigger than available space,
 *   an error is expected.
 * - [8.1.3] Single block allocation using chHeapAlloc() then the block
 *   is freed using chHeapFree(), must not fail.
 * - [8.1.4] Using chHeapStatus() to assess the heap state. There must
 *   be at least one free block of sufficient size.
 * - [8.1.5] Allocating then freeing in the same order.
 * - [8.1.6] Allocating then freeing in reverse order.
 * - [8.1.7] Small fragments handling. Checking the behavior when
 *   allocating blocks with size not multiple of alignment unit.
 * - [8.1.8] Skipping a fragment, the first fragment in the list is too
 *   small so the allocator must pick the second one.
 * - [8.1.9] Allocating the whole available space.
 * - [8.1.10] Testing final conditions. The heap geometry must be the
 *   same than the one registered at beginning.
 * .
 */

static void oslib_test_008_001_setup(void) {
  chHeapObjectInit(&test_heap, test_heap_buffer, sizeof(test_heap_buffer));
}

static void oslib_test_008_001_execute(void) {
  void *p1, *p2, *p3;
  size_t n, sz;

  /* [8.1.1] Testing initial conditions, the heap must not be
     fragmented and one free block present.*/
  test_set_step(1);
  {
    test_assert(chHeapStatus(&test_heap, &sz, NULL) == 1, "heap fragmented");
  }
  test_end_step(1);

  /* [8.1.2] Trying to allocate an block bigger than available space,
     an error is expected.*/
  test_set_step(2);
  {
    p1 = chHeapAlloc(&test_heap, sizeof test_heap_buffer * 2);
    test_assert(p1 == NULL, "allocation not failed");
  }
  test_end_step(2);

  /* [8.1.3] Single block allocation using chHeapAlloc() then the block
     is freed using chHeapFree(), must not fail.*/
  test_set_step(3);
  {
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    test_assert(p1 != NULL, "allocation failed");
    chHeapFree(p1);
  }
  test_end_step(3);

  /* [8.1.4] Using chHeapStatus() to assess the heap state. There must
     be at least one free block of sufficient size.*/
  test_set_step(4);
  {
    size_t total_size, largest_size;

    n = chHeapStatus(&test_heap, &total_size, &largest_size);
    test_assert(n == 1, "missing free block");
    test_assert(total_size >= ALLOC_SIZE, "unexpected heap state");
    test_assert(total_size == largest_size, "unexpected heap state");
  }
  test_end_step(4);

  /* [8.1.5] Allocating then freeing in the same order.*/
  test_set_step(5);
  {
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    p2 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    p3 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    chHeapFree(p1);                                 /* Does not merge.*/
    chHeapFree(p2);                                 /* Merges backward.*/
    chHeapFree(p3);                                 /* Merges both sides.*/
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 1, "heap fragmented");
  }
  test_end_step(5);

  /* [8.1.6] Allocating then freeing in reverse order.*/
  test_set_step(6);
  {
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    p2 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    p3 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    chHeapFree(p3);                                 /* Merges forward.*/
    chHeapFree(p2);                                 /* Merges forward.*/
    chHeapFree(p1);                                 /* Merges forward.*/
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 1, "heap fragmented");
  }
  test_end_step(6);

  /* [8.1.7] Small fragments handling. Checking the behavior when
     allocating blocks with size not multiple of alignment unit.*/
  test_set_step(7);
  {
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE + 1);
    p2 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    chHeapFree(p1);
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 2, "invalid state");
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    /* Note, the first situation happens when the alignment size is smaller
       than the header size, the second in the other cases.*/
    test_assert((chHeapStatus(&test_heap, &n, NULL) == 1) ||
                (chHeapStatus(&test_heap, &n, NULL) == 2), "heap fragmented");
    chHeapFree(p2);
    chHeapFree(p1);
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 1, "heap fragmented");
  }
  test_end_step(7);

  /* [8.1.8] Skipping a fragment, the first fragment in the list is too
     small so the allocator must pick the second one.*/
  test_set_step(8);
  {
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    p2 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    chHeapFree(p1);
    test_assert( chHeapStatus(&test_heap, &n, NULL) == 2, "invalid state");
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE * 2); /* Skips first fragment.*/
    chHeapFree(p1);
    chHeapFree(p2);
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 1, "heap fragmented");
  }
  test_end_step(8);

  /* [8.1.9] Allocating the whole available space.*/
  test_set_step(9);
  {
    (void)chHeapStatus(&test_heap, &n, NULL);
    p1 = chHeapAlloc(&test_heap, n);
    test_assert(p1 != NULL, "allocation failed");
    test_assert(chHeapStatus(&test_heap, NULL, NULL) == 0, "not empty");
    chHeapFree(p1);
  }
  test_end_step(9);

  /* [8.1.10] Testing final conditions. The heap geometry must be the
     same than the one registered at beginning.*/
  test_set_step(10);
  {
    test_assert(chHeapStatus(&test_heap, &n, NULL) == 1, "heap fragmented");
    test_assert(n == sz, "size changed");
  }
  test_end_step(10);
}

static const testcase_t oslib_test_008_001 = {
  "Allocation and fragmentation",
  oslib_test_008_001_setup,
  NULL,
  oslib_test_008_001_execute
};

/**
 * @page oslib_test_008_002 [8.2] Default Heap
 *
 * <h2>Description</h2>
 * The default heap is pre-allocated in the system. We test base
 * functionality.
 *
 * <h2>Test Steps</h2>
 * - [8.2.1] Single block allocation using chHeapAlloc() then the block
 *   is freed using chHeapFree(), must not fail.
 * - [8.2.2] Testing allocation failure.
 * .
 */

static void oslib_test_008_002_execute(void) {
  void *p1;
  size_t total_size, largest_size;

  /* [8.2.1] Single block allocation using chHeapAlloc() then the block
     is freed using chHeapFree(), must not fail.*/
  test_set_step(1);
  {
    (void)chHeapStatus(NULL, &total_size, &largest_size);
    p1 = chHeapAlloc(&test_heap, ALLOC_SIZE);
    test_assert(p1 != NULL, "allocation failed");
    chHeapFree(p1);
  }
  test_end_step(1);

  /* [8.2.2] Testing allocation failure.*/
  test_set_step(2);
  {
    p1 = chHeapAlloc(NULL, (size_t)-256);
    test_assert(p1 == NULL, "allocation not failed");
  }
  test_end_step(2);
}

static const testcase_t oslib_test_008_002 = {
  "Default Heap",
  NULL,
  NULL,
  oslib_test_008_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_008_array[] = {
  &oslib_test_008_001,
  &oslib_test_008_002,
  NULL
};

/**
 * @brief   Memory Heaps.
 */
const testsequence_t oslib_test_sequence_008 = {
  "Memory Heaps",
  oslib_test_sequence_008_array
};

#endif /* CH_CFG_USE_HEAP == TRUE */
