/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

BaseSequentialStream *chp = (BaseSequentialStream *) &SD1;

  /* Key to use during the Encryption. */
  uint8_t key[AES_BLOCK_SIZE]   = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                    0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
  /* Data to encrypt. */
  uint8_t data[AES_BLOCK_SIZE]  = { 0x08, 0x04, 0x02, 0x01, 0x08, 0x04, 0x02, 0x01,
                                    0x08, 0x04, 0x12, 0x11, 0x18, 0x14, 0x12, 0x11};
  uint8_t data_encrypted[AES_BLOCK_SIZE] = {0};
  uint8_t data_decrypted[AES_BLOCK_SIZE] = {0};
  cryerror_t res;
  uint8_t tkey = 0; /* Transient key. */

static const CRYConfig cryConfig = {
  FALSE,  /* No Auto start feature. */
  FALSE   /* No XOR feature.        */
};

/**
 * @brief   Blink thread.
 */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");

  while (true) {
    palClearPad(IOPORT5, PORTE_LED);
    chThdSleepMilliseconds(1000);
    palSetPad(IOPORT5, PORTE_LED);
    chThdSleepMilliseconds(100);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  uint8_t i;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Turn off the LED.
   */
  palClearPad(IOPORT5, PORTE_LED);

  /*
   * Configure TX (PINC3) and RX (PIN2) for the USART1.
   */
  palSetPadMode(IOPORT3, PIN3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT3, PIN2, PAL_MODE_INPUT_PULLUP);

  /*
   * Start the Serial driver 1 by using the default configuration.
   */
  sdStart(&SD1, NULL);

  chprintf(chp, "\r\n");
  chprintf(chp, "***********************************************\r\n");
  chprintf(chp, "*** ChibiOS/RT testhal/AVR/XMEGA/CRYPTO.\r\n");
  chprintf(chp, "*** Test data Encryption/Decryption.\r\n");

  chprintf(chp, "--- Start Crypto driver.\r\n");
  cryStart(&CRYD1, &cryConfig);

  chprintf(chp, "--- Load transient key for encrypt/decrypt.\r\n");
  res = cryLoadAESTransientKey(&CRYD1, 16, key);

  chprintf(chp, "--- Start data encryption.\r\n");
  res = cryEncryptAES(&CRYD1, tkey, data, data_encrypted);

  if (res != CRY_NOERROR) {
    chprintf(chp, "--- Encryption failed.\r\n");
    while (true) {
      chThdSleepMilliseconds(100);
    }
  }
  else {
    chprintf(chp, "--- Encryption done.\r\n");
  }

  chprintf(chp, "--- Start data decryption.\r\n");
  res = cryDecryptAES(&CRYD1, tkey, data_encrypted, data_decrypted);
  if (res != CRY_NOERROR) {
      chprintf(chp, "--- Decryption failed.\r\n");
    while (true) {
      chThdSleepMilliseconds(100);
    }
  }
  else {
    chprintf(chp, "--- Decryption done.\r\n");
  }

  chprintf(chp, "--- Compare decrypted data to original data.\r\n");

  /*
   * Check if decrypted answer is equal to plaintext.
   */
  for (i = 0; i < AES_BLOCK_SIZE; i++) {
    if (data[i] != data_decrypted[i]) {
      chprintf(chp, "--- Decrypted data is different to original data.\r\n");
      while (true) {
        chThdSleepMilliseconds(100);
      }
    }
  }

  chprintf(chp, "--- Decrypted data is equal to original data.\r\n");

  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (true) {
    chThdSleepMilliseconds(100);
  }

  return 0;
}
