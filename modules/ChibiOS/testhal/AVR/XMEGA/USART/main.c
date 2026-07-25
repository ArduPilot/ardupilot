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

/**
 * @brief   USARTC0 configuration structure.
 */
const UARTConfig usart1cfg = {
  NULL,                     /* Transmission buffer callback.          */
  NULL,                     /* End of physical transmission callback. */
  NULL,                     /* Received buffer filled  callback.      */
  NULL,                     /* Caractere received while output.       */
  NULL,                     /* Received error callback.               */
  115200,                   /* Usart baud rate.                       */
  false,                    /* Double transmission speed.             */
  false,                    /* Multiprocessor communication mode bit. */
  false,                    /* Transmission bit 8.                    */
  USART_CMODE_ASYNCHRONOUS, /* Communication mode.                    */
  USART_PMODE_DISABLE,      /* Parity mode.                           */
  false,                    /* False=1bit stop, true=2bit stop.       */
  USART_CHSIZE_8BIT,        /* Caractere size.                        */
};

static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");

  while (true) {
    palClearPad(IOPORT5, PORTE_LED);
    chThdSleepMilliseconds(2000);
    palSetPad(IOPORT5, PORTE_LED);
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
   * Set the TX and the RX pin to use the USARTC0 module.
   * PIN3 (TXD0) as output for USARTC0.
   * PIN2 (RXD0) as input for USARTC0.
   */
  palSetPadMode(IOPORT3, PIN3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT3, PIN2, PAL_MODE_INPUT_PULLUP);

  /* Start the USART1 (USARTC0). */
  uartStart(&USART1D, &usart1cfg);

  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (TRUE) {
    uartStartSend(&USART1D, 14, (const uint8_t *)"Hello world!\r\n");
    chThdSleepMilliseconds(2000);
  }
}

