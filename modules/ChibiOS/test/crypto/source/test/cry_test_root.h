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
 * @file    cry_test_root.h
 * @brief   Test Suite root structures header.
 */

#ifndef CRY_TEST_ROOT_H
#define CRY_TEST_ROOT_H

#include "ch_test.h"

#include "cry_test_sequence_001.h"
#include "cry_test_sequence_002.h"
#include "cry_test_sequence_003.h"
#include "cry_test_sequence_004.h"
#include "cry_test_sequence_005.h"
#include "cry_test_sequence_006.h"
#include "cry_test_sequence_007.h"
#include "cry_test_sequence_008.h"
#include "cry_test_sequence_009.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern const testsuite_t cry_test_suite;

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Shared definitions.                                                       */
/*===========================================================================*/

extern void cryptoTest_setStream(BaseSequentialStream * s);
extern void cryptoTest_printArray32(bool isLE,const uint32_t *a,size_t len);
#ifdef LOG_CRYPTO_DATA
#define SHOW_ENCRYPDATA(w) 	cryptoTest_printArray32(true,msg_encrypted,w)
#define SHOW_DECRYPDATA(w)	cryptoTest_printArray32(true,msg_decrypted,w)
#define SHOW_DATA(d,w)      cryptoTest_printArray32(true,d,w)
#else
#define SHOW_ENCRYPDATA(w)
#define SHOW_DECRYPDATA(w)
#define SHOW_DATA(d,w)
#endif

#define TEST_DATA_BYTE_LEN 	640
#define TEST_DATA_WORD_LEN		(TEST_DATA_BYTE_LEN / 4)


#define TEST_MSG_DATA_BYTE_LEN		640
#define TEST_MSG_DATA_WORD_LEN		(TEST_MSG_DATA_BYTE_LEN / 4)

#define SHA_LEN_0    3
#define SHA_LEN_1    56
#define SHA_LEN_2    64
#define SHA_LEN_3    128

#define TEST_GCM_KEY1_LEN   32
#define TEST_P_LEN      60
#define TEST_A_LEN      20
#define TEST_IV1_LEN    12   
#define TEST_CL_LEN     60
#define TEST_TL_LEN     16  

extern const char test_plain_data[TEST_DATA_BYTE_LEN];
extern uint32_t msg_clear[TEST_MSG_DATA_WORD_LEN];
extern uint32_t msg_encrypted[TEST_MSG_DATA_WORD_LEN];
extern uint32_t msg_decrypted[TEST_MSG_DATA_WORD_LEN];
extern const uint32_t test_keys[8];
extern const uint32_t test_vectors[4];
extern const uint8_t sha_msg0[SHA_LEN_0];
extern const uint8_t sha_msg1[SHA_LEN_1];
extern const uint8_t sha_msg2[SHA_LEN_2];
extern const uint8_t sha_msg3[SHA_LEN_3];




#endif /* !defined(__DOXYGEN__) */

#endif /* CRY_TEST_ROOT_H */
