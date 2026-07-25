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
 * @file    cry_test_sequence_008.c
 * @brief   Test Sequence 008 code.
 *
 * @page cry_test_sequence_008 [8] GCM
 *
 * File: @ref cry_test_sequence_008.c
 *
 * <h2>Description</h2>
 * GCM testing.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_008_001
 * - @subpage cry_test_008_002
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "ref_gcm.h"
#define plaintext msg_clear
#define cypher	  msg_encrypted
#define authtag	  msg_decrypted

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

struct test_el_t
{
		uint32_t size;
		const uint8_t * data;

};
struct test_gcm_t
{
	struct test_el_t key;
	struct test_el_t p;
	struct test_el_t iv;
	struct test_el_t aad;
	struct test_el_t c;
	struct test_el_t t;

};
#define TEST_GCM_LEN 3

const struct test_gcm_t test_gcm_k[TEST_GCM_LEN]={

	{ {K3_LEN,K3},{P3_LEN,P3},{IV3_LEN,IV3},{AAD3_LEN,A3},{C3_LEN,C3},{T3_LEN,T3}  },
	{ {K4_LEN,K4},{P4_LEN,P4},{IV4_LEN,IV4},{AAD4_LEN,A4},{C4_LEN,C4},{T4_LEN,T4}  },
	{ {K5_LEN,K5},{P5_LEN,P5},{IV5_LEN,IV5},{AAD5_LEN,A5},{C5_LEN,C5},{T5_LEN,T5}  }
};





/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_008_001 [8.1] GCM Polling
 *
 * <h2>Description</h2>
 * testing GCM in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [8.1.1] loading the key, encrypt and decrypt.
 * .
 */

static void cry_test_008_001_setup(void) {
    memset(cypher, 0xff, TEST_MSG_DATA_BYTE_LEN);
    memset(authtag, 0xff, TEST_MSG_DATA_BYTE_LEN);
    cryStart(&CRYD1, &config_Polling);


}

static void cry_test_008_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_008_001_execute(void) {
  cryerror_t ret;
  	uint32_t *ref;
  	uint8_t i,len1,len2;

  /* [8.1.1] loading the key, encrypt and decrypt.*/
  test_set_step(1);
  {
    for (i = 0;i<TEST_GCM_LEN;i++) {

            test_print("gcm test : ");
            test_printn(i);
            test_println("");
    		/* loading the key .*/

    		{
    			ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,test_gcm_k[i].key.size, (uint8_t *) test_gcm_k[i].key.data);

    			test_assert(ret == CRY_NOERROR, "failed load transient key");
    		}

    		/* Encrypt.*/

    		{

    			ret = cryEncryptAES_GCM(&CRYD1,
    									0,
    									test_gcm_k[i].p.size,
    									test_gcm_k[i].p.data,
    									(uint8_t*)cypher,
    									test_gcm_k[i].iv.data,
    									test_gcm_k[i].aad.size,
    									test_gcm_k[i].aad.data,
    									(uint8_t*)authtag);

    			test_assert(ret == CRY_NOERROR, "failed encryption");

    			len1 = test_gcm_k[i].c.size/4;
    			len2 = test_gcm_k[i].t.size/4;
    			SHOW_DATA(cypher,len1);
    			SHOW_DATA(authtag,len2);

    			ref = (uint32_t*)test_gcm_k[i].c.data;

    			for (uint8_t i = 0; i < len1; i++) {
    				test_assert(ref[i] == cypher[i], "c mismatch");
    			}
    			ref = (uint32_t*)test_gcm_k[i].t.data;

    			for (uint8_t i = 0; i < len2; i++) {
    				test_assert(ref[i] == authtag[i], "t mismatch");
    			}



    		}

    		/*  Decrypt.*/

    		{


    			memset(plaintext, 0, test_gcm_k[i].p.size);

    			ret = cryDecryptAES_GCM(&CRYD1,
    									0,
    									test_gcm_k[i].c.size,
    									(uint8_t*)cypher,
    									(uint8_t*)plaintext,
    									test_gcm_k[i].iv.data,
    									test_gcm_k[i].aad.size,
    									test_gcm_k[i].aad.data,
    									(uint8_t*)authtag);

    			test_assert(ret == CRY_NOERROR, "failed decryption");

    			len1 = test_gcm_k[i].p.size/4;
    			SHOW_DATA(plaintext,len1);

    			ref = (uint32_t*)test_gcm_k[i].p.data;

    			for (uint8_t i = 0; i < len1; i++) {
    				test_assert(ref[i] == plaintext[i], "decrypt plain mismatch");
    			}



    		}

    	}
  }
  test_end_step(1);
}

static const testcase_t cry_test_008_001 = {
  "GCM Polling",
  cry_test_008_001_setup,
  cry_test_008_001_teardown,
  cry_test_008_001_execute
};

/**
 * @page cry_test_008_002 [8.2] GCM DMA
 *
 * <h2>Description</h2>
 * testing GCM in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [8.2.1] loading the key, encrypt and decrypt.
 * .
 */

static void cry_test_008_002_setup(void) {
    memset(cypher, 0xff, TEST_MSG_DATA_BYTE_LEN);
    memset(authtag, 0xff, TEST_MSG_DATA_BYTE_LEN);
    cryStart(&CRYD1, &config_DMA);


}

static void cry_test_008_002_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_008_002_execute(void) {
  cryerror_t ret;
  	uint32_t *ref;
  	uint8_t i,len1,len2;

  /* [8.2.1] loading the key, encrypt and decrypt.*/
  test_set_step(1);
  {
    for (i = 0;i<TEST_GCM_LEN;i++) {

            test_print("gcm test : ");
            test_printn(i);
            test_println("");
    		/* loading the key .*/

    		{
    			ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,test_gcm_k[i].key.size, (uint8_t *) test_gcm_k[i].key.data);

    			test_assert(ret == CRY_NOERROR, "failed load transient key");
    		}

    		/* Encrypt.*/

    		{

    			ret = cryEncryptAES_GCM(&CRYD1,
    									0,
    									test_gcm_k[i].p.size,
    									test_gcm_k[i].p.data,
    									(uint8_t*)cypher,
    									test_gcm_k[i].iv.data,
    									test_gcm_k[i].aad.size,
    									test_gcm_k[i].aad.data,
    									(uint8_t*)authtag);

    			test_assert(ret == CRY_NOERROR, "failed encryption");

    			len1 = test_gcm_k[i].c.size/4;
    			len2 = test_gcm_k[i].t.size/4;
    			SHOW_DATA(cypher,len1);
    			SHOW_DATA(authtag,len2);

    			ref = (uint32_t*)test_gcm_k[i].c.data;

    			for (uint8_t i = 0; i < len1; i++) {
    				test_assert(ref[i] == cypher[i], "c mismatch");
    			}
    			ref = (uint32_t*)test_gcm_k[i].t.data;

    			for (uint8_t i = 0; i < len2; i++) {
    				test_assert(ref[i] == authtag[i], "t mismatch");
    			}



    		}

    		/*  Decrypt.*/

    		{


    			memset(plaintext, 0, test_gcm_k[i].p.size);

    			ret = cryDecryptAES_GCM(&CRYD1,
    									0,
    									test_gcm_k[i].c.size,
    									(uint8_t*)cypher,
    									(uint8_t*)plaintext,
    									test_gcm_k[i].iv.data,
    									test_gcm_k[i].aad.size,
    									test_gcm_k[i].aad.data,
    									(uint8_t*)authtag);

    			test_assert(ret == CRY_NOERROR, "failed decryption");

    			len1 = test_gcm_k[i].p.size/4;
    			SHOW_DATA(plaintext,len1);

    			ref = (uint32_t*)test_gcm_k[i].p.data;

    			for (uint8_t i = 0; i < len1; i++) {
    				test_assert(ref[i] == plaintext[i], "decrypt plain mismatch");
    			}



    		}

    	}
  }
  test_end_step(1);
}

static const testcase_t cry_test_008_002 = {
  "GCM DMA",
  cry_test_008_002_setup,
  cry_test_008_002_teardown,
  cry_test_008_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_008_array[] = {
  &cry_test_008_001,
  &cry_test_008_002,
  NULL
};

/**
 * @brief   GCM.
 */
const testsequence_t cry_test_sequence_008 = {
  "GCM",
  cry_test_sequence_008_array
};
