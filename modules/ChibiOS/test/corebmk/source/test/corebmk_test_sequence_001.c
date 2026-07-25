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
/*
    This module is based on the work of John Walker (April of 1989) and
    merely adapted to work in ChibiOS. The author has not specified
    additional license terms so this is released using the most permissive
    license used in ChibiOS. The license covers the changes only, not the
    original work.
 */

#include "hal.h"
#include "corebmk_test_root.h"

/**
 * @file    corebmk_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page corebmk_test_sequence_001 [1] Information
 *
 * File: @ref corebmk_test_sequence_001.c
 *
 * <h2>Description</h2>
 * This sequence reports configuration and version information about
 * execution environment.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_HEAP == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage corebmk_test_001_001
 * - @subpage corebmk_test_001_002
 * .
 */

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

#include "ch.h"

#include "ffbench_mod.h"

#define ASIZE       64              /* Array edge size.                     */
#define NITERATIONS 10              /* Number of iterations.                */
#define NPASSES     50              /* Number of FFT/Inverse passes.        */

#define max(a, b)   ((a) > (b) ? (a) : (b))
#define min(a, b)   ((a) <= (b) ? (a) : (b))

float *fdatas;
double *fdatad;

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page corebmk_test_001_001 [1.1] Environment Info
 *
 * <h2>Description</h2>
 * Environment-related info are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] Architecture and Compiler information.
 * .
 */

static void corebmk_test_001_001_execute(void) {

  /* [1.1.1] Architecture and Compiler information.*/
  test_set_step(1);
  {
#if defined(PORT_ARCHITECTURE_NAME)
    test_print("--- Architecture:                       ");
    test_println(PORT_ARCHITECTURE_NAME);
#endif
#if defined(PORT_CORE_VARIANT_NAME)
    test_print("--- Core Variant:                       ");
    test_println(PORT_CORE_VARIANT_NAME);
#endif
#if defined(PORT_COMPILER_NAME)
    test_print("--- Compiler:                           ");
    test_println(PORT_COMPILER_NAME);
#endif
#if defined(PORT_INFO)
    test_print("--- Port Info:                          ");
    test_println(PORT_INFO);
#endif
#if defined(PORT_NATURAL_ALIGN)
    test_print("--- Natural alignment:                  ");
    test_printn(PORT_NATURAL_ALIGN);
    test_println("");
#endif
#if defined(PORT_STACK_ALIGN)
    test_print("--- Stack alignment:                    ");
    test_printn(PORT_STACK_ALIGN);
    test_println("");
#endif
#if defined(PORT_WORKING_AREA_ALIGN)
    test_print("--- Working area alignment:             ");
    test_printn(PORT_WORKING_AREA_ALIGN);
    test_println("");
#endif
  }
  test_end_step(1);
}

static const testcase_t corebmk_test_001_001 = {
  "Environment Info",
  NULL,
  NULL,
  corebmk_test_001_001_execute
};

/**
 * @page corebmk_test_001_002 [1.2] Two-dimensional FFT
 *
 * <h2>Description</h2>
 * Two-dimensional FFT benchmark, execution time is reported, expected
 * results are checked.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] Allocating memory for single precision work matrix.
 * - [1.2.2] Printing setup.
 * - [1.2.3] Running single precision FFT iterations.
 * - [1.2.4] Printing execution time.
 * .
 */

static void corebmk_test_001_002_setup(void) {
  fdatas = NULL;
}

static void corebmk_test_001_002_teardown(void) {
  if (fdatas != NULL) {
    chHeapFree((void *)fdatas);
  }
}

static void corebmk_test_001_002_execute(void) {
  time_msecs_t msecs;
  size_t fasize;
  int fanum;
  int faedge;
  int nsize[] = {0, 0, 0};

  /* [1.2.1] Allocating memory for single precision work matrix.*/
  test_set_step(1);
  {
    faedge = ASIZE;                                     /* FFT array edge size.*/
    fanum  = faedge * faedge;
    fasize = ((size_t)(fanum + 1) * 2 * sizeof(float)); /* FFT array size.*/
    fdatas = (float *)chHeapAlloc(NULL, fasize);
    nsize[1] = nsize[2] = faedge;

    test_assert(fdatas != NULL, "single precision matrix allocation failed");
  }
  test_end_step(1);

  /* [1.2.2] Printing setup.*/
  test_set_step(2);
  {
    test_print("--- Matrix: ");
    test_printn(ASIZE);
    test_print("x");
    test_printn(ASIZE);
    test_println("");
    test_print("--- Iter. : ");
    test_printn(NITERATIONS);
    test_println("");
    test_print("--- Passes: ");
    test_printn(NPASSES);
    test_println("");
  }
  test_end_step(2);

  /* [1.2.3] Running single precision FFT iterations.*/
  test_set_step(3);
  {
    systime_t start, end;
    int i, j, k, iters;

    /* Time stamp for benchmark start.*/
    start = chVTGetSystemTime();

    iters = 0;
    for (k = 0; k < NITERATIONS; k++) {

      /* Generate data array to process.*/
      memset(fdatas, 0, fasize);
      for (i = 0; i < faedge; i++) {
        for (j = 0; j < faedge; j++) {
          if (((i & 15) == 8) || ((j & 15) == 8)) {
            fdatas[1 + ((faedge * i) + j) * 2] = 128.0;
          }
        }
      }

      for (i = 0; i < NPASSES; i++) {
        /* Transform image to frequency domain.*/
        fourn_float(fdatas, nsize, 2, 1);

        /* Back-transform to image.*/
        fourn_float(fdatas, nsize, 2, -1);

        iters++;
      }
    }

    /* Time stamp for benchmark end.*/
    end = chVTGetSystemTime();
    msecs = chTimeI2MS(chTimeDiffX(start, end));
  }
  test_end_step(3);

  /* [1.2.4] Printing execution time.*/
  test_set_step(4);
  {
    test_print("--- Time  : ");
    test_printn(msecs);
    test_println(" milliseconds");
  }
  test_end_step(4);
}

static const testcase_t corebmk_test_001_002 = {
  "Two-dimensional FFT",
  corebmk_test_001_002_setup,
  corebmk_test_001_002_teardown,
  corebmk_test_001_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const corebmk_test_sequence_001_array[] = {
  &corebmk_test_001_001,
  &corebmk_test_001_002,
  NULL
};

/**
 * @brief   Information.
 */
const testsequence_t corebmk_test_sequence_001 = {
  "Information",
  corebmk_test_sequence_001_array
};

#endif /* CH_CFG_USE_HEAP == TRUE */
