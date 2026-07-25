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
#include "nil_test_root.h"

/**
 * @file    nil_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page nil_test_sequence_001 [1] Information
 *
 * File: @ref nil_test_sequence_001.c
 *
 * <h2>Description</h2>
 * This sequence reports configuration and version information about
 * the NIL kernel.
 *
 * <h2>Test Cases</h2>
 * - @subpage nil_test_001_001
 * - @subpage nil_test_001_002
 * - @subpage nil_test_001_003
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include "ch.h"

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page nil_test_001_001 [1.1] Port Info
 *
 * <h2>Description</h2>
 * Port-related info are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] Prints the version string.
 * .
 */

static void nil_test_001_001_execute(void) {

  /* [1.1.1] Prints the version string.*/
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

static const testcase_t nil_test_001_001 = {
  "Port Info",
  NULL,
  NULL,
  nil_test_001_001_execute
};

/**
 * @page nil_test_001_002 [1.2] Kernel Info
 *
 * <h2>Description</h2>
 * The version numbers are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] Prints the version string.
 * .
 */

static void nil_test_001_002_execute(void) {

  /* [1.2.1] Prints the version string.*/
  test_set_step(1);
  {
    test_println("--- Product:                            ChibiOS/NIL");
    test_print("--- Stable Flag:                        ");
    test_printn(CH_KERNEL_STABLE);
    test_println("");
    test_print("--- Version String:                     ");
    test_println(CH_KERNEL_VERSION);
    test_print("--- Major Number:                       ");
    test_printn(CH_KERNEL_MAJOR);
    test_println("");
    test_print("--- Minor Number:                       ");
    test_printn(CH_KERNEL_MINOR);
    test_println("");
    test_print("--- Patch Number:                       ");
    test_printn(CH_KERNEL_PATCH);
    test_println("");
  }
  test_end_step(1);
}

static const testcase_t nil_test_001_002 = {
  "Kernel Info",
  NULL,
  NULL,
  nil_test_001_002_execute
};

/**
 * @page nil_test_001_003 [1.3] Kernel Settings
 *
 * <h2>Description</h2>
 * The static kernel settings are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.3.1] Prints the configuration options settings.
 * .
 */

static void nil_test_001_003_execute(void) {

  /* [1.3.1] Prints the configuration options settings.*/
  test_set_step(1);
  {
    test_print("--- CH_CFG_MAX_THREADS:                 ");
    test_printn(CH_CFG_MAX_THREADS);
    test_println("");
    test_print("--- CH_CFG_AUTOSTART_THREADS:           ");
    test_printn(CH_CFG_AUTOSTART_THREADS);
    test_println("");
    test_print("--- CH_CFG_ST_RESOLUTION:               ");
    test_printn(CH_CFG_ST_RESOLUTION);
    test_println("");
    test_print("--- CH_CFG_ST_FREQUENCY:                ");
    test_printn(CH_CFG_ST_FREQUENCY);
    test_println("");
    test_print("--- CH_CFG_ST_TIMEDELTA:                ");
    test_printn(CH_CFG_ST_TIMEDELTA);
    test_println("");
    test_print("--- CH_CFG_USE_WAITEXIT:                ");
    test_printn(CH_CFG_USE_WAITEXIT);
    test_println("");
    test_print("--- CH_CFG_USE_SEMAPHORES:              ");
    test_printn(CH_CFG_USE_SEMAPHORES);
    test_println("");
    test_print("--- CH_CFG_USE_MUTEXES:                 ");
    test_printn(CH_CFG_USE_MUTEXES);
    test_println("");
    test_print("--- CH_CFG_USE_EVENTS:                  ");
    test_printn(CH_CFG_USE_EVENTS);
    test_println("");
    test_print("--- CH_CFG_USE_MESSAGES:                ");
    test_printn(CH_CFG_USE_MESSAGES);
    test_println("");
    test_print("--- CH_DBG_STATISTICS:                  ");
    test_printn(CH_DBG_STATISTICS);
    test_println("");
    test_print("--- CH_DBG_SYSTEM_STATE_CHECK:          ");
    test_printn(CH_DBG_SYSTEM_STATE_CHECK);
    test_println("");
    test_print("--- CH_DBG_ENABLE_CHECKS:               ");
    test_printn(CH_DBG_ENABLE_CHECKS);
    test_println("");
    test_print("--- CH_DBG_ENABLE_ASSERTS:              ");
    test_printn(CH_DBG_ENABLE_ASSERTS);
    test_println("");
    test_print("--- CH_DBG_ENABLE_STACK_CHECK:          ");
    test_printn(CH_DBG_ENABLE_STACK_CHECK);
    test_println("");
  }
  test_end_step(1);
}

static const testcase_t nil_test_001_003 = {
  "Kernel Settings",
  NULL,
  NULL,
  nil_test_001_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const nil_test_sequence_001_array[] = {
  &nil_test_001_001,
  &nil_test_001_002,
  &nil_test_001_003,
  NULL
};

/**
 * @brief   Information.
 */
const testsequence_t nil_test_sequence_001 = {
  "Information",
  nil_test_sequence_001_array
};
