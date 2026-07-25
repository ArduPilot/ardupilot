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
 * @file    oslib_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page oslib_test_sequence_001 [1] Information
 *
 * File: @ref oslib_test_sequence_001.c
 *
 * <h2>Description</h2>
 * This sequence reports configuration and version information about
 * the OS library.
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_001_001
 * - @subpage oslib_test_001_002
 * - @subpage oslib_test_001_003
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
 * @page oslib_test_001_001 [1.1] Port Info
 *
 * <h2>Description</h2>
 * Port-related info are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] Prints the version string.
 * .
 */

static void oslib_test_001_001_execute(void) {

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

static const testcase_t oslib_test_001_001 = {
  "Port Info",
  NULL,
  NULL,
  oslib_test_001_001_execute
};

/**
 * @page oslib_test_001_002 [1.2] OS Library Info
 *
 * <h2>Description</h2>
 * The version numbers are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] Prints the version string.
 * .
 */

static void oslib_test_001_002_execute(void) {

  /* [1.2.1] Prints the version string.*/
  test_set_step(1);
  {
    test_println("--- Product:                            ChibiOS/LIB");
    test_print("--- Stable Flag:                        ");
    test_printn(CH_OSLIB_STABLE);
    test_println("");
    test_print("--- Version String:                     ");
    test_println(CH_OSLIB_VERSION);
    test_print("--- Major Number:                       ");
    test_printn(CH_OSLIB_MAJOR);
    test_println("");
    test_print("--- Minor Number:                       ");
    test_printn(CH_OSLIB_MINOR);
    test_println("");
    test_print("--- Patch Number:                       ");
    test_printn(CH_OSLIB_PATCH);
    test_println("");
  }
  test_end_step(1);
}

static const testcase_t oslib_test_001_002 = {
  "OS Library Info",
  NULL,
  NULL,
  oslib_test_001_002_execute
};

/**
 * @page oslib_test_001_003 [1.3] OS Library Settings
 *
 * <h2>Description</h2>
 * The static OS Library settings are reported.
 *
 * <h2>Test Steps</h2>
 * - [1.3.1] Prints the configuration options settings.
 * .
 */

static void oslib_test_001_003_execute(void) {

  /* [1.3.1] Prints the configuration options settings.*/
  test_set_step(1);
  {
    test_print("--- CH_CFG_USE_MAILBOXES:               ");
    test_printn(CH_CFG_USE_MAILBOXES);
    test_println("");
    test_print("--- CH_CFG_USE_MEMCORE:                 ");
    test_printn(CH_CFG_USE_MEMCORE);
    test_println("");
    test_print("--- CH_CFG_USE_HEAP:                    ");
    test_printn(CH_CFG_USE_HEAP);
    test_println("");
    test_print("--- CH_CFG_USE_MEMPOOLS:                ");
    test_printn(CH_CFG_USE_MEMPOOLS);
    test_println("");
    test_print("--- CH_CFG_USE_OBJ_FIFOS:               ");
    test_printn(CH_CFG_USE_OBJ_FIFOS);
    test_println("");
    test_print("--- CH_CFG_USE_PIPES:                   ");
    test_printn(CH_CFG_USE_PIPES);
    test_println("");
    test_print("--- CH_CFG_USE_OBJ_CACHES:              ");
    test_printn(CH_CFG_USE_OBJ_CACHES);
    test_println("");
    test_print("--- CH_CFG_USE_DELEGATES:               ");
    test_printn(CH_CFG_USE_DELEGATES);
    test_println("");
    test_print("--- CH_CFG_USE_FACTORY:                 ");
    test_printn(CH_CFG_USE_FACTORY);
    test_println("");
    test_print("--- CH_CFG_FACTORY_MAX_NAMES_LENGTH:    ");
    test_printn(CH_CFG_FACTORY_MAX_NAMES_LENGTH);
    test_println("");
    test_print("--- CH_CFG_FACTORY_OBJECTS_REGISTRY:    ");
    test_printn(CH_CFG_FACTORY_OBJECTS_REGISTRY);
    test_println("");
    test_print("--- CH_CFG_FACTORY_GENERIC_BUFFERS:     ");
    test_printn(CH_CFG_FACTORY_GENERIC_BUFFERS);
    test_println("");
    test_print("--- CH_CFG_FACTORY_SEMAPHORES:          ");
    test_printn(CH_CFG_FACTORY_SEMAPHORES);
    test_println("");
    test_print("--- CH_CFG_FACTORY_MAILBOXES:           ");
    test_printn(CH_CFG_FACTORY_MAILBOXES);
    test_println("");
    test_print("--- CH_CFG_FACTORY_OBJ_FIFOS:           ");
    test_printn(CH_CFG_FACTORY_OBJ_FIFOS);
    test_println("");
    test_print("--- CH_CFG_FACTORY_PIPES:               ");
    test_printn(CH_CFG_FACTORY_PIPES);
    test_println("");
  }
  test_end_step(1);
}

static const testcase_t oslib_test_001_003 = {
  "OS Library Settings",
  NULL,
  NULL,
  oslib_test_001_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_001_array[] = {
  &oslib_test_001_001,
  &oslib_test_001_002,
  &oslib_test_001_003,
  NULL
};

/**
 * @brief   Information.
 */
const testsequence_t oslib_test_sequence_001 = {
  "Information",
  oslib_test_sequence_001_array
};
