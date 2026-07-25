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
#include "rt_test_root.h"
#include "oslib_test_root.h"

/*
 * Blinker thread #1.
 */
THD_WORKING_AREA(waThread1, 128);
THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker 1");
  while (true) {
    palToggleLine(LINE_LED3_RED);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED7_GREEN);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED10_RED);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED6_GREEN);
    chThdSleepMilliseconds(100);
  }
}

/*
 * Blinker thread #2.
 */
THD_WORKING_AREA(waThread2, 128);
THD_FUNCTION(Thread2, arg) {

  (void)arg;

  chRegSetThreadName("blinker 2");
  while (true) {
    chThdSleepMilliseconds(50);
    palToggleLine(LINE_LED5_ORANGE);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED9_BLUE);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED8_ORANGE);
    chThdSleepMilliseconds(100);
    palToggleLine(LINE_LED4_BLUE);
    chThdSleepMilliseconds(50);
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

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));       /* USART1 TX.       */
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));      /* USART1 RX.       */

  /*
   * Creates the example threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
      test_execute((BaseSequentialStream *)&SD1, &rt_test_suite);
      test_execute((BaseSequentialStream *)&SD1, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}
