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

#define BOTH_BUTTONS (PAL_PORT_BIT(PA_BUTTON1) | PAL_PORT_BIT(PA_BUTTON2))

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker1");
  while (true) {
    palClearPort(IOPORT1, PAL_PORT_BIT(PA_LED2));
    chThdSleepMilliseconds(200);
    palSetPort(IOPORT1, PAL_PORT_BIT(PA_LED1) | PAL_PORT_BIT(PA_LED2));
    chThdSleepMilliseconds(800);
    palClearPort(IOPORT1, PAL_PORT_BIT(PA_LED1));
    chThdSleepMilliseconds(200);
    palSetPort(IOPORT1, PAL_PORT_BIT(PA_LED1) | PAL_PORT_BIT(PA_LED2));
    chThdSleepMilliseconds(800);
  }
}

/*
 * Yellow LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("blinker2");
  while (true) {
    palClearPad(IOPORT1, PA_LEDUSB);
    chThdSleepMilliseconds(200);
    palSetPad(IOPORT1, PA_LEDUSB);
    chThdSleepMilliseconds(300);
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

  /*
   * Creating blinkers threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the buttons state and run test procedure.
   */
  while (true) {
    if (!palReadPad(IOPORT1, PA_BUTTON1))
      sdWrite(&SD1, (uint8_t *)"Hello World!\r\n", 14);
    if (!palReadPad(IOPORT1, PA_BUTTON2)) {
      test_execute((BaseSequentialStream *)&SD1, &rt_test_suite);
      test_execute((BaseSequentialStream *)&SD1, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
