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
#include "portab.h"

void txend1(UARTDriver *uartp) {

  (void)uartp;
}

void txend2(UARTDriver *uartp) {

  (void)uartp;
}

void rxend(UARTDriver *uartp) {

  (void)uartp;
}

void rxchar(UARTDriver *uartp, uint16_t c) {

  (void)uartp;
  (void)c;
}

void rxerr(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
}

/*
 * Application entry point.
 */
int main(void) {
  static const char teststr[] = "0123456789ABCDEF";
  msg_t msg;
  size_t sz;

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
   * Board-dependent setup code.
   */
  portab_setup();

  /*
   * Activates the UART driver 2.
   */
  uartStart(&PORTAB_UART1, &uart_cfg_1);

  /*
   * Blocking API test, send and restart send.
   */
  sz = 3;
  msg = uartSendTimeout(&PORTAB_UART1, &sz, teststr, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");

  sz = 12;
  msg = uartSendTimeout(&PORTAB_UART1, &sz, teststr, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");

  sz = 2;
  msg = uartSendTimeout(&PORTAB_UART1, &sz, teststr, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");

  sz = 16;
  msg = uartSendFullTimeout(&PORTAB_UART1, &sz, teststr, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");

  sz = 7;
  msg = uartSendFullTimeout(&PORTAB_UART1, &sz, teststr, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");

  /*
   * Normal main() thread activity.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
