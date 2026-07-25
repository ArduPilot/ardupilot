/* Copyright statement.*/

/**
 * @file    nasa_osal_test_root.h
 * @brief   Test Suite root structures header.
 */

#ifndef NASA_OSAL_TEST_ROOT_H
#define NASA_OSAL_TEST_ROOT_H

#include "ch_test.h"

#include "nasa_osal_test_sequence_001.h"
#include "nasa_osal_test_sequence_002.h"
#include "nasa_osal_test_sequence_003.h"
#include "nasa_osal_test_sequence_004.h"
#include "nasa_osal_test_sequence_005.h"
#include "nasa_osal_test_sequence_006.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern const testsuite_t nasa_osal_test_suite;

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Shared definitions.                                                       */
/*===========================================================================*/

#define TEST_SUITE_NAME                     "NASA OSAL over ChibiOS/RT Test Suite"

#define TASKS_BASE_PRIORITY                 200
#define TASKS_STACK_SIZE                    256

extern THD_WORKING_AREA(wa_test1, TASKS_STACK_SIZE);
extern THD_WORKING_AREA(wa_test2, TASKS_STACK_SIZE);
extern THD_WORKING_AREA(wa_test3, TASKS_STACK_SIZE);
extern THD_WORKING_AREA(wa_test4, TASKS_STACK_SIZE);

#endif /* !defined(__DOXYGEN__) */

#endif /* NASA_OSAL_TEST_ROOT_H */
