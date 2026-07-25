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
#include "cry_test_root.h"

/**
 * @file    cry_test_sequence_009.c
 * @brief   Test Sequence 009 code.
 *
 * @page cry_test_sequence_009 [9] HMAC
 *
 * File: @ref cry_test_sequence_009.c
 *
 * <h2>Description</h2>
 * HMAC testing.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_009_001
 * - @subpage cry_test_009_002
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "ref_hmac.h"
#define plaintext msg_clear
#define HMACOUT	  msg_encrypted
#define shabuffer msg_decrypted

#define MAX_SHA_BLOCK_SIZE			TEST_MSG_DATA_BYTE_LEN
#define MAX_SHA_BLOCK_SIZE_INWORD  (MAX_SHA_BLOCK_SIZE/4)

static const CRYConfig config_Polling=
{
		TRANSFER_POLLING,
		0
};

static const CRYConfig config_DMA=
{
		TRANSFER_DMA,
		0
};


static const int hmackeys_size[]=
{
 20,
 4,
 20
};

static const uint8_t hmackey_1[]=
{
     0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,
     0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,

};

//   "Hi There",
static const uint8_t hmacmsg1[8]= "Hi There";
static const size_t hmacmsg_size[]=
{
   8,
};
static void cryHMAC256(CRYDriver *cryp) {

    cryerror_t ret;
    HMACSHA256Context hmacshactxp;
    uint8_t *keyp;


    hmacshactxp.shacontext.sha.sha_buffer = (uint8_t*)&shabuffer[0];
    hmacshactxp.shacontext.sha.sha_buffer_size = MAX_SHA_BLOCK_SIZE;

    keyp =(uint8_t *)hmackey_1;
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_hmac,hmackeys_size[0], keyp);

    test_assert(ret == CRY_NOERROR, "failed load transient key");

    ret = cryHMACSHA256Init(cryp,&hmacshactxp);

	test_assert(ret == CRY_NOERROR, "failed init HMACSHA256");

    ret = cryHMACSHA256Update(cryp,&hmacshactxp,hmacmsg_size[0],(const uint8_t *)&hmacmsg1);

	test_assert(ret == CRY_NOERROR, "failed update HMACSHA256");

    ret = cryHMACSHA256Final(cryp,&hmacshactxp,(uint8_t *)HMACOUT);

	test_assert(ret == CRY_NOERROR, "failed final HMACSHA256");


    SHOW_DATA(HMACOUT,8);

}

static void cryHMAC512(CRYDriver *cryp) {

    cryerror_t ret;
    HMACSHA512Context hmacshactxp;
    uint8_t *keyp;


    hmacshactxp.shacontext.sha.sha_buffer = (uint8_t*)&shabuffer[0];
    hmacshactxp.shacontext.sha.sha_buffer_size = MAX_SHA_BLOCK_SIZE;

    keyp =(uint8_t *)hmackey_1;
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_hmac,hmackeys_size[0], keyp);

    test_assert(ret == CRY_NOERROR, "failed load transient key");

    ret = cryHMACSHA512Init(cryp,&hmacshactxp);

	test_assert(ret == CRY_NOERROR, "failed init HMACSHA512");

    ret = cryHMACSHA512Update(cryp,&hmacshactxp,hmacmsg_size[0],(const uint8_t *)&hmacmsg1);

	test_assert(ret == CRY_NOERROR, "failed update HMACSHA512");

    ret = cryHMACSHA512Final(cryp,&hmacshactxp,(uint8_t *)HMACOUT);

	test_assert(ret == CRY_NOERROR, "failed final HMACSHA512");


    SHOW_DATA(HMACOUT,16);

}



/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_009_001 [9.1] HMAC Polling
 *
 * <h2>Description</h2>
 * testing HMAC in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [9.1.1] hmac 256.
 * - [9.1.2] hmac 512.
 * .
 */

static void cry_test_009_001_setup(void) {
  memset(HMACOUT, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &config_Polling);


}

static void cry_test_009_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_009_001_execute(void) {

  /* [9.1.1] hmac 256.*/
  test_set_step(1);
  {
    cryHMAC256(&CRYD1);    

    for (int i = 0; i < 8; i++) {
       test_assert(HMACOUT[i] == ((uint32_t*) refHMAC_HMAC256_1)[i], "hmac mismatch");
    }
  }
  test_end_step(1);

  /* [9.1.2] hmac 512.*/
  test_set_step(2);
  {
    cryHMAC512(&CRYD1);    

    for (int i = 0; i < 16; i++) {
       test_assert(HMACOUT[i] == ((uint32_t*) refHMAC_HMAC512_1)[i], "hmac mismatch");
    }
  }
  test_end_step(2);
}

static const testcase_t cry_test_009_001 = {
  "HMAC Polling",
  cry_test_009_001_setup,
  cry_test_009_001_teardown,
  cry_test_009_001_execute
};

/**
 * @page cry_test_009_002 [9.2] HMAC DMA
 *
 * <h2>Description</h2>
 * testing HMAC in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [9.2.1] hmac 256.
 * - [9.2.2] hmac 512.
 * .
 */

static void cry_test_009_002_setup(void) {
  memset(HMACOUT, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &config_DMA);


}

static void cry_test_009_002_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_009_002_execute(void) {

  /* [9.2.1] hmac 256.*/
  test_set_step(1);
  {
    cryHMAC256(&CRYD1);    


    for (int i = 0; i < 8; i++) {
       test_assert(HMACOUT[i] == ((uint32_t*) refHMAC_HMAC256_1)[i], "hmac mismatch");
    }
  }
  test_end_step(1);

  /* [9.2.2] hmac 512.*/
  test_set_step(2);
  {
    cryHMAC512(&CRYD1);    

    for (int i = 0; i < 16; i++) {
       test_assert(HMACOUT[i] == ((uint32_t*) refHMAC_HMAC512_1)[i], "hmac mismatch");
    }
  }
  test_end_step(2);
}

static const testcase_t cry_test_009_002 = {
  "HMAC DMA",
  cry_test_009_002_setup,
  cry_test_009_002_teardown,
  cry_test_009_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_009_array[] = {
  &cry_test_009_001,
  &cry_test_009_002,
  NULL
};

/**
 * @brief   HMAC.
 */
const testsequence_t cry_test_sequence_009 = {
  "HMAC",
  cry_test_sequence_009_array
};
