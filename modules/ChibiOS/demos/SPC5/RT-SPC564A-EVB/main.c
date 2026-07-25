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
#include "shell.h"
#include "chprintf.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD1,
  commands
};

/*
 * LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");

  while (true) {
    unsigned i;

    for (i = 0; i < 4; i++) {
      palClearPad(PORT11, P11_LED1);
      chThdSleepMilliseconds(100);
      palClearPad(PORT11, P11_LED2);
      chThdSleepMilliseconds(100);
      palClearPad(PORT11, P11_LED3);
      chThdSleepMilliseconds(100);
      palClearPad(PORT11, P11_LED4);
      chThdSleepMilliseconds(100);
      palSetPad(PORT11, P11_LED1);
      chThdSleepMilliseconds(100);
      palSetPad(PORT11, P11_LED2);
      chThdSleepMilliseconds(100);
      palSetPad(PORT11, P11_LED3);
      chThdSleepMilliseconds(100);
      palSetPad(PORT11, P11_LED4);
      chThdSleepMilliseconds(300);
    }

    for (i = 0; i < 4; i++) {
      palTogglePad(PORT11, P11_LED1);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT11, P11_LED1);
      palTogglePad(PORT11, P11_LED2);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT11, P11_LED2);
      palTogglePad(PORT11, P11_LED3);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT11, P11_LED3);
      palTogglePad(PORT11, P11_LED4);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT11, P11_LED4);
    }

    palSetPort(PORT11,
               PAL_PORT_BIT(P11_LED1) | PAL_PORT_BIT(P11_LED2) |
               PAL_PORT_BIT(P11_LED3) | PAL_PORT_BIT(P11_LED4));
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
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, spawning shells.
   */
  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                            "shell", NORMALPRIO + 1,
                                            shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp);               /* Waiting termination.             */
    chThdSleepMilliseconds(1000);
  }
  return 0;
}
