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
 * @mainpage Test Suite Specification
 * Test suite for ChibiOS Crypto Hal. The purpose of this suite is to
 * perform unit tests on the Hal Crypto and to converge to 100% code
 * coverage through successive improvements.
 *
 * <h2>Test Sequences</h2>
 * - @subpage cry_test_sequence_001
 * - @subpage cry_test_sequence_002
 * - @subpage cry_test_sequence_003
 * - @subpage cry_test_sequence_004
 * - @subpage cry_test_sequence_005
 * - @subpage cry_test_sequence_006
 * - @subpage cry_test_sequence_007
 * - @subpage cry_test_sequence_008
 * - @subpage cry_test_sequence_009
 * .
 */

/**
 * @file    cry_test_root.c
 * @brief   Test Suite root structures code.
 */

#include "hal.h"
#include "cry_test_root.h"

#if !defined(__DOXYGEN__)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Array of test sequences.
 */
const testsequence_t * const cry_test_suite_array[] = {
  &cry_test_sequence_001,
  &cry_test_sequence_002,
  &cry_test_sequence_003,
  &cry_test_sequence_004,
  &cry_test_sequence_005,
  &cry_test_sequence_006,
  &cry_test_sequence_007,
  &cry_test_sequence_008,
  &cry_test_sequence_009,
  NULL
};

/**
 * @brief   Test suite root structure.
 */
const testsuite_t cry_test_suite = {
  "Chibios Hal Crypto Test Suite",
  cry_test_suite_array
};

/*===========================================================================*/
/* Shared code.                                                              */
/*===========================================================================*/


#if CRYPTO_LOG_LEVEL == 1
#include "chprintf.h"
#endif



const uint32_t test_keys[8]=
{
    0x01234567,       //KEY 1 low part
    0x89ABCDEF,       //KEY 1 hi part
    0x76543210,       //KEY 2 low part
    0xFEDCBA98,       //KEY 2 hi part

    0x55AA55AA,
    0xAA55AA55,
    0x0000FFFF,
    0xFFFF0000

};

const uint32_t test_vectors[4]=
{
    0x11223344,   //VECTOR 1 low part
    0x55667788,   //VECTOR 1 hi part
    0x11112222,
    0x33334444

};

const char test_plain_data[TEST_DATA_BYTE_LEN] ="\
Lorem ipsum dolor sit amet, consectetur adipiscing elit. Praesen\
t et pellentesque risus. Sed id gravida elit. Proin eget accumsa\
n mi. Aliquam vitae dui porta, euismod velit viverra, elementum \
lacus. Nunc turpis orci, venenatis vel vulputate nec, luctus sit\
amet urna. Ut et nunc purus. Aliquam erat volutpat. Vestibulum n\
ulla dolor, cursus vitae cursus eget, dapibus eget sapien. Integ\
er justo eros, commodo ut massa eu, bibendum elementum tellus. N\
am quis dolor in libero placerat congue. Sed sodales urna sceler\
isque dui faucibus, vitae malesuada dui fermentum. Proin ultrici\
es sit amet justo at ornare. Suspendisse efficitur purus nullam.";


const uint8_t sha_msg0[SHA_LEN_0] = "abc";

const uint8_t sha_msg1[SHA_LEN_1] = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";

const uint8_t sha_msg2[SHA_LEN_2] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

const uint8_t sha_msg3[SHA_LEN_3] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";


ALIGNED_VAR(4) uint32_t msg_clear[TEST_MSG_DATA_WORD_LEN];
ALIGNED_VAR(4) uint32_t msg_encrypted[TEST_MSG_DATA_WORD_LEN];
ALIGNED_VAR(4) uint32_t msg_decrypted[TEST_MSG_DATA_WORD_LEN];
BaseSequentialStream * ts;

#if CRYPTO_LOG_LEVEL == 1
static uint32_t toBigEndian(uint32_t v)
{
  return (v & 0x000000ff) << 24u |
  ( (v & 0x0000ff00) << 8u ) |
  ( (v & 0x00ff0000) >> 8u ) |
  ( (v & 0xff000000) >> 24u );
}
#endif

void cryptoTest_setStream(BaseSequentialStream * s)
{
	ts = s;
}

void cryptoTest_printArray(const uint8_t *a,size_t len)
{
#if CRYPTO_LOG_LEVEL == 1
	for(size_t i=0;i<len;i++)
	{
		chprintf(ts,"%02X",a[i]);
	}
	chprintf(ts,"\r\n");
#else
	(void)a;
	(void)len;
#endif
}

void cryptoTest_printArray32(bool isLE,const uint32_t *a,size_t len)
{
#if CRYPTO_LOG_LEVEL == 1
	uint32_t data;

	for(size_t i=0;i<len;i++)
	{
		if (isLE)
			data = toBigEndian(a[i]);
		else
			data = a[i];

		chprintf(ts,"%08X ",data);

	}
	chprintf(ts,"\r\n");
#else
	(void)isLE;
	(void)a;
	(void)len;
#endif
}



#endif /* !defined(__DOXYGEN__) */
