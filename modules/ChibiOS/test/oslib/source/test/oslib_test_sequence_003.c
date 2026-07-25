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
 * @file    oslib_test_sequence_003.c
 * @brief   Test Sequence 003 code.
 *
 * @page oslib_test_sequence_003 [3] Pipes
 *
 * File: @ref oslib_test_sequence_003.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * pipes.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_PIPES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_003_001
 * - @subpage oslib_test_003_002
 * .
 */

#if (CH_CFG_USE_PIPES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

#define PIPE_SIZE 16

static uint8_t buffer[PIPE_SIZE];
static PIPE_DECL(pipe1, buffer, PIPE_SIZE);

static const uint8_t pipe_pattern[] = "0123456789ABCDEF";

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page oslib_test_003_001 [3.1] Pipes normal API, non-blocking tests
 *
 * <h2>Description</h2>
 * The pipe functionality is tested by loading and emptying it, all
 * conditions are tested.
 *
 * <h2>Test Steps</h2>
 * - [3.1.1] Resetting pipe.
 * - [3.1.2] Writing data, must fail.
 * - [3.1.3] Reading data, must fail.
 * - [3.1.4] Reactivating pipe.
 * - [3.1.5] Filling whole pipe.
 * - [3.1.6] Emptying pipe.
 * - [3.1.7] Small write.
 * - [3.1.8] Filling remaining space.
 * - [3.1.9] Small Read.
 * - [3.1.10] Reading remaining data.
 * - [3.1.11] Small Write.
 * - [3.1.12] Small Read.
 * - [3.1.13] Write wrapping buffer boundary.
 * - [3.1.14] Read wrapping buffer boundary.
 * .
 */

static void oslib_test_003_001_setup(void) {
  chPipeObjectInit(&pipe1, buffer, PIPE_SIZE);
}

static void oslib_test_003_001_execute(void) {

  /* [3.1.1] Resetting pipe.*/
  test_set_step(1);
  {
    chPipeReset(&pipe1);

    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
  }
  test_end_step(1);

  /* [3.1.2] Writing data, must fail.*/
  test_set_step(2);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == 0, "not reset");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
  }
  test_end_step(2);

  /* [3.1.3] Reading data, must fail.*/
  test_set_step(3);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == 0, "not reset");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
  }
  test_end_step(3);

  /* [3.1.4] Reactivating pipe.*/
  test_set_step(4);
  {
    chPipeResume(&pipe1);
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
  }
  test_end_step(4);

  /* [3.1.5] Filling whole pipe.*/
  test_set_step(5);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE, "wrong size");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == PIPE_SIZE),
                "invalid pipe state");
  }
  test_end_step(5);

  /* [3.1.6] Emptying pipe.*/
  test_set_step(6);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE, "wrong size");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
    test_assert(memcmp(pipe_pattern, buf, PIPE_SIZE) == 0, "content mismatch");
  }
  test_end_step(6);

  /* [3.1.7] Small write.*/
  test_set_step(7);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, 4, TIME_IMMEDIATE);
    test_assert(n == 4, "wrong size");
    test_assert((pipe1.rdptr != pipe1.wrptr) &&
                (pipe1.rdptr == pipe1.buffer) &&
                (pipe1.cnt == 4),
                "invalid pipe state");
  }
  test_end_step(7);

  /* [3.1.8] Filling remaining space.*/
  test_set_step(8);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, PIPE_SIZE - 4, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE - 4, "wrong size");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == PIPE_SIZE),
                "invalid pipe state");
  }
  test_end_step(8);

  /* [3.1.9] Small Read.*/
  test_set_step(9);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, 4, TIME_IMMEDIATE);
    test_assert(n == 4, "wrong size");
    test_assert((pipe1.rdptr != pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == PIPE_SIZE - 4),
                "invalid pipe state");
    test_assert(memcmp(pipe_pattern, buf, 4) == 0, "content mismatch");
  }
  test_end_step(9);

  /* [3.1.10] Reading remaining data.*/
  test_set_step(10);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, PIPE_SIZE - 4, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE - 4, "wrong size");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
    test_assert(memcmp(pipe_pattern, buf, PIPE_SIZE - 4) == 0, "content mismatch");
  }
  test_end_step(10);

  /* [3.1.11] Small Write.*/
  test_set_step(11);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, 5, TIME_IMMEDIATE);
    test_assert(n == 5, "wrong size");
    test_assert((pipe1.rdptr != pipe1.wrptr) &&
                (pipe1.rdptr == pipe1.buffer) &&
                (pipe1.cnt == 5),
                "invalid pipe state");
  }
  test_end_step(11);

  /* [3.1.12] Small Read.*/
  test_set_step(12);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, 5, TIME_IMMEDIATE);
    test_assert(n == 5, "wrong size");
    test_assert((pipe1.rdptr == pipe1.wrptr) &&
                (pipe1.wrptr != pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
    test_assert(memcmp(pipe_pattern, buf, 5) == 0, "content mismatch");
  }
  test_end_step(12);

  /* [3.1.13] Write wrapping buffer boundary.*/
  test_set_step(13);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE, "wrong size");
    test_assert((pipe1.rdptr == pipe1.wrptr) &&
                (pipe1.wrptr != pipe1.buffer) &&
                (pipe1.cnt == PIPE_SIZE),
                "invalid pipe state");
  }
  test_end_step(13);

  /* [3.1.14] Read wrapping buffer boundary.*/
  test_set_step(14);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE, "wrong size");
    test_assert((pipe1.rdptr == pipe1.wrptr) &&
                (pipe1.wrptr != pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
    test_assert(memcmp(pipe_pattern, buf, PIPE_SIZE) == 0, "content mismatch");
  }
  test_end_step(14);
}

static const testcase_t oslib_test_003_001 = {
  "Pipes normal API, non-blocking tests",
  oslib_test_003_001_setup,
  NULL,
  oslib_test_003_001_execute
};

/**
 * @page oslib_test_003_002 [3.2] Pipe timeouts
 *
 * <h2>Description</h2>
 * The pipe API is tested for timeouts.
 *
 * <h2>Test Steps</h2>
 * - [3.2.1] Reading while pipe is empty.
 * - [3.2.2] Writing a string larger than pipe buffer.
 * .
 */

static void oslib_test_003_002_setup(void) {
  chPipeObjectInit(&pipe1, buffer, PIPE_SIZE / 2);
}

static void oslib_test_003_002_execute(void) {

  /* [3.2.1] Reading while pipe is empty.*/
  test_set_step(1);
  {
    size_t n;
    uint8_t buf[PIPE_SIZE];

    n = chPipeReadTimeout(&pipe1, buf, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == 0, "wrong size");
    test_assert((pipe1.rdptr == pipe1.buffer) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == 0),
                "invalid pipe state");
  }
  test_end_step(1);

  /* [3.2.2] Writing a string larger than pipe buffer.*/
  test_set_step(2);
  {
    size_t n;

    n = chPipeWriteTimeout(&pipe1, pipe_pattern, PIPE_SIZE, TIME_IMMEDIATE);
    test_assert(n == PIPE_SIZE / 2, "wrong size");
    test_assert((pipe1.rdptr == pipe1.wrptr) &&
                (pipe1.wrptr == pipe1.buffer) &&
                (pipe1.cnt == PIPE_SIZE / 2),
                "invalid pipe state");
  }
  test_end_step(2);
}

static const testcase_t oslib_test_003_002 = {
  "Pipe timeouts",
  oslib_test_003_002_setup,
  NULL,
  oslib_test_003_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_003_array[] = {
  &oslib_test_003_001,
  &oslib_test_003_002,
  NULL
};

/**
 * @brief   Pipes.
 */
const testsequence_t oslib_test_sequence_003 = {
  "Pipes",
  oslib_test_sequence_003_array
};

#endif /* CH_CFG_USE_PIPES == TRUE */
