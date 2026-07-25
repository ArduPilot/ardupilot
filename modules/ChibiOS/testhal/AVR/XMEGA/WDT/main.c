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

/*
 * Watchdog dealine set to one second.
 */
static const WDGConfig wdgcfg = {
  false,              /* WDT use period mode, set true for window mode. */
  WDT_PER_1KCLK_gc,   /* Normal timeout period of 1 second.             */
  WDT_WPER_1KCLK_gc   /* Closed timeout period of 1 second.             */
};

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
   * Configure TX (PINC3) and RX (PIN2) for the USART1.
   */
  palSetPadMode(IOPORT3, PIN3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT3, PIN2, PAL_MODE_INPUT_PULLUP);

  /*
   * Start the Serial driver 1.
   */
  sdStart(&SD1, NULL);

  /*
   * Starting the watchdog driver.
   */
  wdgStart(&WDTD1, &wdgcfg);

  chprintf(chp, "Watchdog driver test program.\r\n");

  /*
   * Normal main() thread activity, it resets the watchdog.
   */
  while (TRUE) {
    chprintf(chp, "Watchdog reset by software.\r\n");
    wdgReset(&WDTD1); /* TODO: Rebuild whit this line commented.      */
                      /* It will shows you if the board is rebooted.  */
                      /* By the watchdog.                             */
    palTogglePad(IOPORT5, PORTE_LED);

    /*
     * Use for example 2 second of delay to see if the watchdog reset the
     * board every second.
     */
    chThdSleepMilliseconds(500);
  }

  return 0;
}
