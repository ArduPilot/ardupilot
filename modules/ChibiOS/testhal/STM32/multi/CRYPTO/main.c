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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "portab.h"

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Board-dependent GPIO setup code.*/
  portab_setup();

  /* Starting a serial port for test report output.*/
  sdStart(&PORTAB_SD1, NULL);

  /* Starting Crypto driver.*/
  cryStart(&CRYD1, NULL);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Normal main() thread activity, in this demo it does nothing.*/
  while (true) {
    uint8_t digest[32];
    static uint8_t iv[16]   = {00, 00, 00, 00, 00, 00, 00, 00,
                               00, 00, 00, 00, 00, 00, 00, 00};
    static uint8_t key[16]  = {00, 00, 00, 00, 00, 00, 00, 00,
                               00, 00, 00, 00, 00, 00, 00, 00};
    static uint8_t data[16] = {00, 00, 00, 00, 00, 00, 00, 00,
                               00, 00, 00, 00, 00, 00, 00, 00};
    uint8_t out[16];

    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
      SHA256Context ctx256;

      crySHA256Init(&CRYD1, &ctx256);
      crySHA256Update(&CRYD1, &ctx256, 0U, (const uint8_t *)"");
      crySHA256Final(&CRYD1, &ctx256, digest);

      crySHA256Init(&CRYD1, &ctx256);
      crySHA256Update(&CRYD1, &ctx256, 3U, (const uint8_t *)"abc");
      crySHA256Final(&CRYD1, &ctx256, digest);

      crySHA256Init(&CRYD1, &ctx256);
      crySHA256Update(&CRYD1, &ctx256, 56U, (const uint8_t *)"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq");
      crySHA256Final(&CRYD1, &ctx256, digest);

      cryLoadAESTransientKey(&CRYD1, sizeof (key), key);
      cryEncryptAES(&CRYD1, (crykey_t)0, data, out);
      cryDecryptAES(&CRYD1, (crykey_t)0, data, out);
      cryEncryptAES_ECB(&CRYD1, (crykey_t)0, 16U, data, out);
      cryDecryptAES_ECB(&CRYD1, (crykey_t)0, 16U, data, out);
      cryEncryptAES_CBC(&CRYD1, (crykey_t)0, 16U, data, out, iv);
      cryDecryptAES_CBC(&CRYD1, (crykey_t)0, 16U, data, out, iv);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
