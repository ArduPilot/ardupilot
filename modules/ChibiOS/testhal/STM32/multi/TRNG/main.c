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

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Normal main() thread activity, in this demo it does nothing.*/
  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
      bool err;
      uint8_t rand[32];

      trngStart(&TRNGD1, NULL);

      err = trngGenerate(&TRNGD1, 32, rand);
      if (err) {
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "Error!\r\n");
      }
      else {
        unsigned i;

        for (i = 0; i < 32; i++) {
          chprintf((BaseSequentialStream *)&PORTAB_SD1, "%02x", rand[i]);
        }
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "\r\n");
      }

      err = trngGenerate(&TRNGD1, 15, rand);
      if (err) {
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "Error!\r\n");
      }
      else {
        unsigned i;

        for (i = 0; i < 15; i++) {
          chprintf((BaseSequentialStream *)&PORTAB_SD1, "%02x", rand[i]);
        }
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "\r\n");
      }

      err = trngGenerate(&TRNGD1, 2, rand);
      if (err) {
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "Error!\r\n");
      }
      else {
        unsigned i;

        for (i = 0; i < 2; i++) {
          chprintf((BaseSequentialStream *)&PORTAB_SD1, "%02x", rand[i]);
        }
        chprintf((BaseSequentialStream *)&PORTAB_SD1, "\r\n");
      }

      trngStop(&TRNGD1);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
