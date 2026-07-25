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

/**
 * @file    oslib_test_root.h
 * @brief   Test Suite root structures header.
 */

#ifndef OSLIB_TEST_ROOT_H
#define OSLIB_TEST_ROOT_H

#include "ch_test.h"

#include "oslib_test_sequence_001.h"
#include "oslib_test_sequence_002.h"
#include "oslib_test_sequence_003.h"
#include "oslib_test_sequence_004.h"
#include "oslib_test_sequence_005.h"
#include "oslib_test_sequence_006.h"
#include "oslib_test_sequence_007.h"
#include "oslib_test_sequence_008.h"
#include "oslib_test_sequence_009.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern const testsuite_t oslib_test_suite;

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Shared definitions.                                                       */
/*===========================================================================*/

#endif /* !defined(__DOXYGEN__) */

#endif /* OSLIB_TEST_ROOT_H */
