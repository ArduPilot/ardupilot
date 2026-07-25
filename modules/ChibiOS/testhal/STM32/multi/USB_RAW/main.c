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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "portab.h"
#include "usbcfg.h"

static const uint8_t txbuf[] =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcde\n"
    "123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\n"
    "23456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0\n"
    "3456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef01\n"
    "456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef012\n"
    "56789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123\n"
    "6789abcdef0123456789abcdef0123456789abcdef0123456789abcdef01234\n"
    "789abcdef0123456789abcdef0123456789abcdef0123456789abcdef012345\n"
    "89abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456\n"
    "9abcdef0123456789abcdef0123456789abcdef0123456789abcdef01234567\n"
    "abcdef0123456789abcdef0123456789abcdef0123456789abcdef012345678\n"
    "bcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789\n"
    "cdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789a\n"
    "def0123456789abcdef0123456789abcdef0123456789abcdef0123456789ab\n"
    "ef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abc\n"
    "f0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcd\n";

static uint8_t rxbuf[1024 + 1];

/*
 * USB writer. This thread writes data to the USB at maximum rate.
 * Can be measured using:
 *   dd if=/dev/xxxx of=/dev/null bs=512 count=10000
 */
static THD_WORKING_AREA(waWriter, 128);
static THD_FUNCTION(Writer, arg) {

  (void)arg;
  chRegSetThreadName("writer");
  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
      msg_t msg = usbTransmit(&PORTAB_USB1, USBD2_DATA_REQUEST_EP,
                              txbuf, sizeof (txbuf));
      if (msg == MSG_RESET)
        chThdSleepMilliseconds(500);
    }
    else {
      chThdSleepMilliseconds(50);
    }
  }
}

/*
 * USB reader. This thread reads data from the USB at maximum rate.
 * Can be measured using:
 *   dd if=bigfile of=/dev/xxx bs=512 count=10000
 */
static THD_WORKING_AREA(waReader, 128);
static THD_FUNCTION(Reader, arg) {

  (void)arg;
  chRegSetThreadName("reader");
  while (true) {
    msg_t msg = usbReceive(&PORTAB_USB1, USBD2_DATA_AVAILABLE_EP,
                           rxbuf, sizeof (rxbuf));
    if (msg == MSG_RESET)
      chThdSleepMilliseconds(500);
    else
      asm volatile ("nop");
  }
}

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time;

    time = PORTAB_USB1.state == USB_ACTIVE ? 250 : 500;
    palClearLine(PORTAB_BLINK_LED1);
    chThdSleepMilliseconds(time);
    palSetLine(PORTAB_BLINK_LED1);
    chThdSleepMilliseconds(time);
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
   * Board-dependent initialization.
   */
  portab_setup();

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&PORTAB_USB1);
  chThdSleepMilliseconds(1500);
  usbStart(&PORTAB_USB1, &usbcfg);
  usbConnectBus(&PORTAB_USB1);

  /*
   * Starting threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waWriter, sizeof(waWriter), NORMALPRIO, Writer, NULL);
  chThdCreateStatic(waReader, sizeof(waReader), NORMALPRIO, Reader, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    chThdSleepMilliseconds(1000);
  }
}
