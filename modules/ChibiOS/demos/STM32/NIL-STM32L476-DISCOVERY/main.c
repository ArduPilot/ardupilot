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
#include "ch.h"
#include "nil_test_root.h"
#include "oslib_test_root.h"

/*
 * Blinker thread #1.
 */
THD_WORKING_AREA(waThread1, 128);
THD_FUNCTION(Thread1, arg) {

  (void)arg;

  while (true) {
    palSetLine(LINE_LED_RED);
    chThdSleepMilliseconds(250);
    palClearLine(LINE_LED_RED);
    chThdSleepMilliseconds(250);
  }
}

/*
 * Blinker thread #2.
 */
THD_WORKING_AREA(waThread2, 128);
THD_FUNCTION(Thread2, arg) {

  (void)arg;

  while (true) {
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(500);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Tester thread.
 */
THD_WORKING_AREA(waThread3, 256);
THD_FUNCTION(Thread3, arg) {

  (void)arg;

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  /* Welcome message.*/
  chnWrite(&SD2, (const uint8_t *)"Hello World!\r\n", 14);

  /* Waiting for button push and activation of the test suite.*/
  while (true) {
    if (palReadLine(LINE_JOY_CENTER)) {
      test_execute((BaseSequentialStream *)&SD2, &nil_test_suite);
      test_execute((BaseSequentialStream *)&SD2, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}

/*
 * Threads creation table, one entry per thread.
 */
THD_TABLE_BEGIN
  THD_TABLE_THREAD(0, "blinker1",     waThread1,       Thread1,      NULL)
  THD_TABLE_THREAD(1, "blinker2",     waThread2,       Thread2,      NULL)
  THD_TABLE_THREAD(4, "tester",       waThread3,       Thread3,      NULL)
THD_TABLE_END

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

  /* This is now the idle thread loop, you may perform here a low priority
     task but you must never try to sleep or wait in this loop. Note that
     this tasks runs at the lowest priority level so any instruction added
     here will be executed after all other tasks have been started.*/
  while (true) {
  }
}
