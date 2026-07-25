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
/*
    Concepts and parts of this file have been contributed by Ilya Kharin.
*/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

/*
 * LEDs blinker thread, LED1 always blinks and LED2 and LED3 blink only when
 * corresponding SW2 and SW3 buttons are pressed, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");

  palClearLine(LINE_LED_BLUE);
  palClearLine(LINE_LED_GREEN);
  palClearLine(LINE_LED_RED);

  while (true) {
    palToggleLine(LINE_LED_BLUE);

    if (PAL_LOW == palReadLine(LINE_BUTTON_2)) {
      palToggleLine(LINE_LED_GREEN);
    }

    if (PAL_LOW == palReadLine(LINE_BUTTON_3)) {
      palToggleLine(LINE_LED_RED);
    }

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

  /*
   * Activates the serial driver using the driver default configuration.
   */
  sdStart(&SD2, NULL);
  sdWrite(&SD2, (uint8_t*)"Initialized\r\n", 13);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the SW1 button state.
   */
  while (true) {
    if (PAL_LOW == palReadLine(LINE_BUTTON_1)) {
      test_execute((BaseSequentialStream *)&SD2, &rt_test_suite);
      test_execute((BaseSequentialStream *)&SD2, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}
