/* Copyright statement.*/

/**
 * @mainpage Test Suite Specification
 * Test suite for NASA OSAL implementation over ChibiOS/RT. The purpose
 * of this suite is to perform unit tests on the OSAL module and to
 * converge to 100% code coverage through successive improvements.
 *
 * <h2>Test Sequences</h2>
 * - @subpage nasa_osal_test_sequence_001
 * - @subpage nasa_osal_test_sequence_002
 * - @subpage nasa_osal_test_sequence_003
 * - @subpage nasa_osal_test_sequence_004
 * - @subpage nasa_osal_test_sequence_005
 * - @subpage nasa_osal_test_sequence_006
 * .
 */

/**
 * @file    nasa_osal_test_root.c
 * @brief   Test Suite root structures code.
 */

#include "hal.h"
#include "nasa_osal_test_root.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Array of test sequences.
 */
const testsequence_t * const nasa_osal_test_suite_array[] = {
  &nasa_osal_test_sequence_001,
  &nasa_osal_test_sequence_002,
  &nasa_osal_test_sequence_003,
  &nasa_osal_test_sequence_004,
  &nasa_osal_test_sequence_005,
  &nasa_osal_test_sequence_006,
  NULL
};

/**
 * @brief   Test suite root structure.
 */
const testsuite_t nasa_osal_test_suite = {
  "NASA OSAL Test Suite",
  nasa_osal_test_suite_array
};

/*===========================================================================*/
/* Shared code.                                                              */
/*===========================================================================*/

THD_WORKING_AREA(wa_test1, TASKS_STACK_SIZE);
THD_WORKING_AREA(wa_test2, TASKS_STACK_SIZE);
THD_WORKING_AREA(wa_test3, TASKS_STACK_SIZE);
THD_WORKING_AREA(wa_test4, TASKS_STACK_SIZE);

#endif /* !defined(__DOXYGEN__) */
