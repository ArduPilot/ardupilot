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
 * @file    cry_test_sequence_001.c
 * @brief   Test Sequence 001 code.
 *
 * @page cry_test_sequence_001 [1] AES ECB
 *
 * File: @ref cry_test_sequence_001.c
 *
 * <h2>Description</h2>
 * AES ECB.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_001_001
 * - @subpage cry_test_001_002
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "ref_aes.h"
static const CRYConfig config_Polling = {
    TRANSFER_POLLING,
    AES_CFBS_128       //cfbs
};

static const CRYConfig config_DMA = {
    TRANSFER_DMA,
    AES_CFBS_128       //cfbs
};


/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_001_001 [1.1] AES ECB Polling
 *
 * <h2>Description</h2>
 * testing AES ECB with various Keys.
 *
 * <h2>Test Steps</h2>
 * - [1.1.1] loading the key with 16 byte size.
 * - [1.1.2] Encrypt.
 * - [1.1.3] Decrypt.
 * - [1.1.4] loading the key with 24 byte size.
 * - [1.1.5] Encrypt.
 * - [1.1.6] Decrypt.
 * - [1.1.7] loading the key with 32 byte size.
 * - [1.1.8] Encrypt.
 * - [1.1.9] Decrypt.
 * .
 */

static void cry_test_001_001_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &config_Polling);


}

static void cry_test_001_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_001_001_execute(void) {
    cryerror_t ret;

  /* [1.1.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [1.1.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_128)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [1.1.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [1.1.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [1.1.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_192)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [1.1.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);

  /* [1.1.7] loading the key with 32 byte size.*/
  test_set_step(7);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,32, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(7);

  /* [1.1.8] Encrypt.*/
  test_set_step(8);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_256)[i], "encrypt mismatch");
    }

  }
  test_end_step(8);

  /* [1.1.9] Decrypt.*/
  test_set_step(9);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(9);
}

static const testcase_t cry_test_001_001 = {
  "AES ECB Polling",
  cry_test_001_001_setup,
  cry_test_001_001_teardown,
  cry_test_001_001_execute
};

/**
 * @page cry_test_001_002 [1.2] AES ECB DMA
 *
 * <h2>Description</h2>
 * testing AES ECB with various Keys.
 *
 * <h2>Test Steps</h2>
 * - [1.2.1] loading the key with 16 byte size.
 * - [1.2.2] Encrypt.
 * - [1.2.3] Decrypt.
 * - [1.2.4] loading the key with 24 byte size.
 * - [1.2.5] Encrypt.
 * - [1.2.6] Decrypt.
 * - [1.2.7] loading the key with 32 byte size.
 * - [1.2.8] Encrypt.
 * - [1.2.9] Decrypt.
 * .
 */

static void cry_test_001_002_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &config_DMA);


}

static void cry_test_001_002_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_001_002_execute(void) {
    cryerror_t ret;

  /* [1.2.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [1.2.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_128)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [1.2.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [1.2.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [1.2.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_192)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [1.2.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);

  /* [1.2.7] loading the key with 32 byte size.*/
  test_set_step(7);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_aes,32, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(7);

  /* [1.2.8] Encrypt.*/
  test_set_step(8);
  {
    ret = cryEncryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refAES_ECB_256)[i], "encrypt mismatch");
    }

  }
  test_end_step(8);

  /* [1.2.9] Decrypt.*/
  test_set_step(9);
  {
    ret = cryDecryptAES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(9);
}

static const testcase_t cry_test_001_002 = {
  "AES ECB DMA",
  cry_test_001_002_setup,
  cry_test_001_002_teardown,
  cry_test_001_002_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_001_array[] = {
  &cry_test_001_001,
  &cry_test_001_002,
  NULL
};

/**
 * @brief   AES ECB.
 */
const testsequence_t cry_test_sequence_001 = {
  "AES ECB",
  cry_test_sequence_001_array
};
