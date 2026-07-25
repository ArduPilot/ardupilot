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
 * @file    cry_test_sequence_004.c
 * @brief   Test Sequence 004 code.
 *
 * @page cry_test_sequence_004 [4] (T)DES
 *
 * File: @ref cry_test_sequence_004.c
 *
 * <h2>Description</h2>
 * (T)DES testing.
 *
 * <h2>Test Cases</h2>
 * - @subpage cry_test_004_001
 * - @subpage cry_test_004_002
 * - @subpage cry_test_004_003
 * - @subpage cry_test_004_004
 * - @subpage cry_test_004_005
 * .
 */

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>
#include "ref_des.h"
static const CRYConfig configDES_Polling=
{
		TRANSFER_POLLING,
		0
};

static const CRYConfig configDES_DMA=
{
		TRANSFER_DMA,
		0
};



/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page cry_test_004_001 [4.1] DES Polling
 *
 * <h2>Description</h2>
 * testing DES in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [4.1.1] loading the key with 8 byte size.
 * - [4.1.2] Encrypt.
 * - [4.1.3] Decrypt.
 * .
 */

static void cry_test_004_001_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &configDES_Polling);


}

static void cry_test_004_001_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_004_001_execute(void) {
    cryerror_t ret;

  /* [4.1.1] loading the key with 8 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,8, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [4.1.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptDES(&CRYD1, 0, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(2);

    for (int i = 0; i < 2; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refDES_ECB_8)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [4.1.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptDES(&CRYD1, 0, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(2);

    for (int i = 0; i < 2; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);
}

static const testcase_t cry_test_004_001 = {
  "DES Polling",
  cry_test_004_001_setup,
  cry_test_004_001_teardown,
  cry_test_004_001_execute
};

/**
 * @page cry_test_004_002 [4.2] TDES CBC Polling
 *
 * <h2>Description</h2>
 * testing TDES CBC in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [4.2.1] loading the key with 16 byte size.
 * - [4.2.2] Encrypt.
 * - [4.2.3] Decrypt.
 * - [4.2.4] loading the key with 24 byte size.
 * - [4.2.5] Encrypt.
 * - [4.2.6] Decrypt.
 * .
 */

static void cry_test_004_002_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &configDES_Polling);


}

static void cry_test_004_002_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_004_002_execute(void) {
    cryerror_t ret;

  /* [4.2.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [4.2.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_CBC_16)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [4.2.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [4.2.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [4.2.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_CBC_24)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [4.2.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);
}

static const testcase_t cry_test_004_002 = {
  "TDES CBC Polling",
  cry_test_004_002_setup,
  cry_test_004_002_teardown,
  cry_test_004_002_execute
};

/**
 * @page cry_test_004_003 [4.3] TDES ECB Polling
 *
 * <h2>Description</h2>
 * testing TDES ECB in polling mode.
 *
 * <h2>Test Steps</h2>
 * - [4.3.1] loading the key with 16 byte size.
 * - [4.3.2] Encrypt.
 * - [4.3.3] Decrypt.
 * - [4.3.4] loading the key with 24 byte size.
 * - [4.3.5] Encrypt.
 * - [4.3.6] Decrypt.
 * .
 */

static void cry_test_004_003_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &configDES_Polling);


}

static void cry_test_004_003_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_004_003_execute(void) {
    cryerror_t ret;

  /* [4.3.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [4.3.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_ECB_16)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [4.3.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [4.3.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [4.3.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_ECB_24)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [4.3.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);
}

static const testcase_t cry_test_004_003 = {
  "TDES ECB Polling",
  cry_test_004_003_setup,
  cry_test_004_003_teardown,
  cry_test_004_003_execute
};

/**
 * @page cry_test_004_004 [4.4] TDES CBC DMA
 *
 * <h2>Description</h2>
 * testing TDES CBC in polled mode.
 *
 * <h2>Test Steps</h2>
 * - [4.4.1] loading the key with 16 byte size.
 * - [4.4.2] Encrypt.
 * - [4.4.3] Decrypt.
 * - [4.4.4] loading the key with 24 byte size.
 * - [4.4.5] Encrypt.
 * - [4.4.6] Decrypt.
 * .
 */

static void cry_test_004_004_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &configDES_DMA);


}

static void cry_test_004_004_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_004_004_execute(void) {
    cryerror_t ret;

  /* [4.4.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [4.4.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_CBC_16)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [4.4.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [4.4.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [4.4.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_CBC_24)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [4.4.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptDES_CBC(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted,(uint8_t*)test_vectors);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);
}

static const testcase_t cry_test_004_004 = {
  "TDES CBC DMA",
  cry_test_004_004_setup,
  cry_test_004_004_teardown,
  cry_test_004_004_execute
};

/**
 * @page cry_test_004_005 [4.5] TDES ECB DMA
 *
 * <h2>Description</h2>
 * testing TDES ECB in DMA mode.
 *
 * <h2>Test Steps</h2>
 * - [4.5.1] loading the key with 16 byte size.
 * - [4.5.2] Encrypt.
 * - [4.5.3] Decrypt.
 * - [4.5.4] loading the key with 24 byte size.
 * - [4.5.5] Encrypt.
 * - [4.5.6] Decrypt.
 * .
 */

static void cry_test_004_005_setup(void) {
  memcpy((char*) msg_clear, test_plain_data, TEST_DATA_BYTE_LEN);
  memset(msg_encrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  memset(msg_decrypted, 0xff, TEST_MSG_DATA_BYTE_LEN);
  cryStart(&CRYD1, &configDES_DMA);


}

static void cry_test_004_005_teardown(void) {
  cryStop(&CRYD1);
}

static void cry_test_004_005_execute(void) {
    cryerror_t ret;

  /* [4.5.1] loading the key with 16 byte size.*/
  test_set_step(1);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,16, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(1);

  /* [4.5.2] Encrypt.*/
  test_set_step(2);
  {
    ret = cryEncryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_ECB_16)[i], "encrypt mismatch");
    }

  }
  test_end_step(2);

  /* [4.5.3] Decrypt.*/
  test_set_step(3);
  {
    ret = cryDecryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(3);

  /* [4.5.4] loading the key with 24 byte size.*/
  test_set_step(4);
  {
    ret = cryLoadTransientKey(&CRYD1, (cryalgorithm_t) cry_algo_des,24, (uint8_t *) test_keys);

    test_assert(ret == CRY_NOERROR, "failed load transient key");
  }
  test_end_step(4);

  /* [4.5.5] Encrypt.*/
  test_set_step(5);
  {
    ret = cryEncryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_clear, (uint8_t*) msg_encrypted);

    test_assert(ret == CRY_NOERROR, "encrypt failed");

    SHOW_ENCRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_encrypted[i] == ((uint32_t*) refTDES_ECB_24)[i], "encrypt mismatch");
    }

  }
  test_end_step(5);

  /* [4.5.6] Decrypt.*/
  test_set_step(6);
  {
    ret = cryDecryptDES_ECB(&CRYD1, 0,TEST_DATA_BYTE_LEN, (uint8_t*) msg_encrypted, (uint8_t*) msg_decrypted);

    test_assert(ret == CRY_NOERROR, "decrypt failed");

    SHOW_DECRYPDATA(TEST_DATA_WORD_LEN);

    for (int i = 0; i < TEST_DATA_WORD_LEN; i++) {
      test_assert(msg_decrypted[i] == msg_clear[i], "decrypt mismatch");
    }

  }
  test_end_step(6);
}

static const testcase_t cry_test_004_005 = {
  "TDES ECB DMA",
  cry_test_004_005_setup,
  cry_test_004_005_teardown,
  cry_test_004_005_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const cry_test_sequence_004_array[] = {
  &cry_test_004_001,
  &cry_test_004_002,
  &cry_test_004_003,
  &cry_test_004_004,
  &cry_test_004_005,
  NULL
};

/**
 * @brief   (T)DES.
 */
const testsequence_t cry_test_sequence_004 = {
  "(T)DES",
  cry_test_sequence_004_array
};
