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
 * @file    cry_test_sequence_007.c
 * @brief   Test Sequence 007 code.
 *
 * @page cry_test_sequence_007 [7] SHA
 *
 * File: @ref cry_test_sequence_007.c
 *
 * <h2>Description</h2>
 * SHA testing.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_007_001
 * - @subpage cry_test_007_002
 * - @subpage cry_test_007_003
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "ref_sha.h"

/* Buffer size for each SHA transfer, size should be multiple of block size
   (block size: 64 for SHA_1/SHA_256/SHA_224, 128 for SHA_384/SHA_512) */
#define MAX_SHA_BLOCK_SIZE			TEST_MSG_DATA_BYTE_LEN
#define MAX_SHA_BLOCK_SIZE_INWORD  (MAX_SHA_BLOCK_SIZE/4)

#define shabuffer	msg_decrypted


#define MAX_DIGEST_SIZE_INBYTE  TEST_MSG_DATA_BYTE_LEN
#define MAX_DIGEST_SIZE_INWORD  (MAX_DIGEST_SIZE_INBYTE/4)

#define digest		msg_encrypted
static const CRYConfig configSHA_DMA=
{
    TRANSFER_DMA,
    0
};

static cryerror_t crySHA1(CRYDriver *cryp, size_t size,const uint8_t *in, uint8_t *out) {

	cryerror_t ret;
	SHA1Context shactxp;

    shactxp.sha.sha_buffer = (uint8_t*)&shabuffer[0];
	shactxp.sha.sha_buffer_size = MAX_SHA_BLOCK_SIZE;

	ret = crySHA1Init(cryp,&shactxp);

	ret = crySHA1Update(cryp,&shactxp,size,in);

	ret = crySHA1Final(cryp,&shactxp,out);


	return ret;
}

static cryerror_t crySHA256(CRYDriver *cryp, size_t size,const uint8_t *in, uint8_t *out) {

	cryerror_t ret;
	SHA256Context shactxp;

    shactxp.sha.sha_buffer = (uint8_t*)&shabuffer[0];
	shactxp.sha.sha_buffer_size = MAX_SHA_BLOCK_SIZE;


	ret = crySHA256Init(cryp,&shactxp);

	ret = crySHA256Update(cryp,&shactxp,size,in);

	ret = crySHA256Final(cryp,&shactxp,out);


	return ret;
}

static cryerror_t crySHA512(CRYDriver *cryp, size_t size,const uint8_t *in, uint8_t *out) {

	cryerror_t ret;
	SHA512Context shactxp;

    shactxp.sha.sha_buffer = (uint8_t*)&shabuffer[0];
	shactxp.sha.sha_buffer_size = MAX_SHA_BLOCK_SIZE;

	ret = crySHA512Init(cryp,&shactxp);

	ret = crySHA512Update(cryp,&shactxp,size,in);

	ret = crySHA512Final(cryp,&shactxp,out);


	return ret;
}



/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_007_001 [7.1] SHA1 DMA
 *
 * <h2>Description</h2>
 * testing SHA1 in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [7.1.1] Digest.
 * .
 */

static void cry_test_007_001_setup(void) {
  memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
  memset(digest, 0, MAX_DIGEST_SIZE_INWORD);
  memcpy((char*) msg_clear, sha_msg0, SHA_LEN_0);
  cryStart(&CRYD1, &configSHA_DMA);


}

static void cry_test_007_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_007_001_execute(void) {
    cryerror_t ret;
    uint32_t *ref;

  /* [7.1.1] Digest.*/
  test_set_step(1);
  {
    //---- Empty Block Test   
    ret = crySHA1(&CRYD1,0,(uint8_t*)msg_clear,(uint8_t*)digest);    

    test_assert(ret == CRY_NOERROR, "sha1 failed");

    SHOW_DATA(digest,5);


    ref = (uint32_t*)refSHA_SHA1_EMPTY;
    for (int i = 0; i < 5; i++) {
      test_assert(digest[i] == ref[i], "sha1 digest mismatch");
    }
    //---- One Block Test
    ret = crySHA1(&CRYD1,SHA_LEN_0,(uint8_t*)msg_clear,(uint8_t*)digest);


    test_assert(ret == CRY_NOERROR, "sha1 failed");


    SHOW_DATA(digest,5);

    ref = (uint32_t*)refSHA_SHA1_3;
    for (int i = 0; i < 5; i++) {
      test_assert(digest[i] == ref[i], "sha1 digest mismatch");
    }

    //---- Multi Block Test 56 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg1, SHA_LEN_1);

    ret = crySHA1(&CRYD1,SHA_LEN_1,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha1 failed");

     SHOW_DATA(digest,5);


    ref = (uint32_t*)refSHA_SHA1_56;
    for (int i = 0; i < 5; i++) {
      test_assert(digest[i] == ref[i], "sha1 digest mismatch");
    }
    //---- Multi Block Test 64 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg2, SHA_LEN_2);

    ret = crySHA1(&CRYD1,SHA_LEN_2,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha1 failed");

     SHOW_DATA(digest,5);


    ref = (uint32_t*)refSHA_SHA1_64;
    for (int i = 0; i < 5; i++) {
      test_assert(digest[i] == ref[i], "sha1 digest mismatch");
    }

    //---- Multi Block Test 128 Byte

    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg3, SHA_LEN_3);

    ret = crySHA1(&CRYD1,SHA_LEN_3,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha1 failed");

    SHOW_DATA(digest,5);


    ref = (uint32_t*)refSHA_SHA1_128;
    for (int i = 0; i < 5; i++) {
        test_assert(digest[i] == ref[i], "sha1 digest mismatch");
    }

  }
  test_end_step(1);
}

static const testcase_t cry_test_007_001 = {
  "SHA1 DMA",
  cry_test_007_001_setup,
  cry_test_007_001_teardown,
  cry_test_007_001_execute
};

/**
 * @page cry_test_007_002 [7.2] SHA256 DMA
 *
 * <h2>Description</h2>
 * testing SHA256 in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [7.2.1] Digest.
 * .
 */

static void cry_test_007_002_setup(void) {
  memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
  memset(digest, 0, MAX_DIGEST_SIZE_INWORD);
  memcpy((char*) msg_clear, sha_msg0, SHA_LEN_0);
  cryStart(&CRYD1, &configSHA_DMA);


}

static void cry_test_007_002_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_007_002_execute(void) {
    cryerror_t ret;
    uint32_t *ref;

  /* [7.2.1] Digest.*/
  test_set_step(1);
  {

    //---- One Block Test
    ret = crySHA256(&CRYD1,SHA_LEN_0,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha256 failed");

    SHOW_DATA(digest,8);

    ref = (uint32_t*)refSHA_SHA256_3;
    for (int i = 0; i < 8; i++) {
      test_assert(digest[i] == ref[i], "sha256 digest mismatch");
    }

    //---- Multi Block Test 56 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg1, SHA_LEN_1);

    ret = crySHA256(&CRYD1,SHA_LEN_1,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha256 56 byte failed");

     SHOW_DATA(digest,8);


    ref = (uint32_t*)refSHA_SHA256_56;
    for (int i = 0; i < 8; i++) {
      test_assert(digest[i] == ref[i], "sha256 56 byte digest mismatch");
    }
    //---- Multi Block Test 64 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg2, SHA_LEN_2);

    ret = crySHA256(&CRYD1,SHA_LEN_2,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha256 64 byte failed");

     SHOW_DATA(digest,8);


    ref = (uint32_t*)refSHA_SHA256_64;
    for (int i = 0; i < 8; i++) {
      test_assert(digest[i] == ref[i], "sha256 64 byte digest mismatch");
    }

    //---- Multi Block Test 128 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg3, SHA_LEN_3);

    ret = crySHA256(&CRYD1,SHA_LEN_3,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha256 128 byte failed");

    SHOW_DATA(digest,8);


    ref = (uint32_t*)refSHA_SHA256_128;
    for (int i = 0; i < 8; i++) {
        test_assert(digest[i] == ref[i], "sha256 128 byte digest mismatch");
    }


  }
  test_end_step(1);
}

static const testcase_t cry_test_007_002 = {
  "SHA256 DMA",
  cry_test_007_002_setup,
  cry_test_007_002_teardown,
  cry_test_007_002_execute
};

/**
 * @page cry_test_007_003 [7.3] SHA512 DMA
 *
 * <h2>Description</h2>
 * testing SHA512 in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [7.3.1] Digest.
 * .
 */

static void cry_test_007_003_setup(void) {
  memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
  memset(digest, 0, MAX_DIGEST_SIZE_INWORD);
  memcpy((char*) msg_clear, sha_msg0, SHA_LEN_0);
  cryStart(&CRYD1, &configSHA_DMA);


}

static void cry_test_007_003_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_007_003_execute(void) {
    cryerror_t ret;
    uint32_t *ref;

  /* [7.3.1] Digest.*/
  test_set_step(1);
  {
    //---- One Block Test
    ret = crySHA512(&CRYD1,SHA_LEN_0,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha512 failed");

    SHOW_DATA(digest,16);

    ref = (uint32_t*)refSHA_SHA512_3;
    for (int i = 0; i < 16; i++) {
      test_assert(digest[i] == ref[i], "sha512 digest mismatch");
    }


    //---- Multi Block Test 56 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg1, SHA_LEN_1);

    ret = crySHA512(&CRYD1,SHA_LEN_1,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha512 56 byte failed");

     SHOW_DATA(digest,16);


    ref = (uint32_t*)refSHA_SHA512_56;
    for (int i = 0; i < 16; i++) {
      test_assert(digest[i] == ref[i], "sha512 56 byte digest mismatch");
    }
    //---- Multi Block Test 64 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg2, SHA_LEN_2);

    ret = crySHA512(&CRYD1,SHA_LEN_2,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha512 64 byte failed");

     SHOW_DATA(digest,16);


    ref = (uint32_t*)refSHA_SHA512_64;
    for (int i = 0; i < 16; i++) {
      test_assert(digest[i] == ref[i], "sha512 64 byte digest mismatch");
    }

    //---- Multi Block Test 128 Byte
    memset(msg_clear, 0, TEST_MSG_DATA_BYTE_LEN);
    memcpy((char*) msg_clear, sha_msg3, SHA_LEN_3);

    ret = crySHA512(&CRYD1,SHA_LEN_3,(uint8_t*)msg_clear,(uint8_t*)digest);

    test_assert(ret == CRY_NOERROR, "sha512 128 byte failed");

    SHOW_DATA(digest,16);


    ref = (uint32_t*)refSHA_SHA512_128;
    for (int i = 0; i < 16; i++) {
        test_assert(digest[i] == ref[i], "sha512 128 byte digest mismatch");
    }

  }
  test_end_step(1);
}

static const testcase_t cry_test_007_003 = {
  "SHA512 DMA",
  cry_test_007_003_setup,
  cry_test_007_003_teardown,
  cry_test_007_003_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_007_array[] = {
  &cry_test_007_001,
  &cry_test_007_002,
  &cry_test_007_003,
  NULL
};

/**
 * @brief   SHA.
 */
const testsequence_t cry_test_sequence_007 = {
  "SHA",
  cry_test_sequence_007_array
};
