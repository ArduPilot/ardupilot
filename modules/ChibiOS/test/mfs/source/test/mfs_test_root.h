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
 * @file    mfs_test_root.h
 * @brief   Test Suite root structures header.
 */

#ifndef MFS_TEST_ROOT_H
#define MFS_TEST_ROOT_H

#include "ch_test.h"

#include "mfs_test_sequence_001.h"
#include "mfs_test_sequence_002.h"
#include "mfs_test_sequence_003.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern const testsuite_t mfs_test_suite;

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Shared definitions.                                                       */
/*===========================================================================*/

#include "hal_mfs.h"

#define TEST_SUITE_NAME "ChibiOS/HAL MFS Test Suite"

#define TEST_REPORT_HOOK_HEADER test_print_mfs_info();

extern mfs_nocache_buffer_t __nocache_mfsbuf1;
extern const MFSConfig mfscfg1;
extern MFSDriver mfs1;
extern uint8_t __nocache_mfs_buffer[512];
extern const uint8_t mfs_pattern16[16];
extern const uint8_t mfs_pattern32[32];
extern const uint8_t mfs_pattern10[10];
extern const uint8_t mfs_pattern512[512];

flash_error_t bank_erase(mfs_bank_t bank);
flash_error_t bank_verify_erased(mfs_bank_t bank);
void test_print_mfs_info(void);

#endif /* !defined(__DOXYGEN__) */

#endif /* MFS_TEST_ROOT_H */
