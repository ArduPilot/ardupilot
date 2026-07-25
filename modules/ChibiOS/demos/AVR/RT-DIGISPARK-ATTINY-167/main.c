/*
    ChibiOS - Copyright (C) 2017 Theodore Ateba

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

/*
 * UART 1 configuration structure.
 */
const UARTConfig uartConf = {
  NULL,   /* UART transmission buffer callback.           */
  NULL,   /* UART physical end of transmission callback.  */
  NULL,   /* UART Receiver receiver filled callback.      */
  NULL,   /* UART caracter received callback.             */
  NULL,   /* UART received error callback.                */
  38400,  /* UART baudrate.                               */
};

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chThdSleepMilliseconds(1000);
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
   * Initialize the UART interface.
   */
   uartInit();

  /*
   * Start the Uart 1 interface.
   */
  uartStart(&UARTD1, &uartConf);

  /*
   * Send an message via the UART 1 interface.
   */
  uartStartSend(&UARTD1, 15, (const void *) "Hello world!.\n\r");

  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (true) {
    chThdSleepMilliseconds(1000);
  }
}
