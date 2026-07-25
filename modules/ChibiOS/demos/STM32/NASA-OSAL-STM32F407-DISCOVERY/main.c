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
#include "nasa_osal_test_root.h"
#include "osapi.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 * Note, the working area is created using ChibiOS/RT macro because alignment
 * constraints.
 */
static THD_WORKING_AREA(wa_blinker, 128);
static void blinker(void) {

  OS_TaskRegister();

  while (true) {
    palSetLine(LINE_LED3);       /* Orange.  */
    OS_TaskDelay(500);
    palClearLine(LINE_LED3);     /* Orange.  */
    OS_TaskDelay(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  uint32 blinker_id;

  /* HAL initialization, this also initializes the configured device drivers
     and performs the board-specific initializations.*/
  halInit();

  /* OS initialization.*/
  (void) OS_API_Init();

  /* Activates the serial driver 2 using the driver default configuration.
    PA2(TX) and PA3(RX) are routed to USART2.*/
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /* Starting the blinker thread.*/
  (void) OS_TaskCreate(&blinker_id, "blinker", blinker,
                       (uint32 *)wa_blinker, sizeof wa_blinker,
                       128, 0);

  /* In the ChibiOS/RT OSAL implementation the main() function is an
     usable thread with priority 128 (NORMALPRIO), here we just sleep
     waiting for a button event, then the test suite is executed.*/
  while (true) {
    if (palReadLine(LINE_BUTTON))
      test_execute((BaseSequentialStream *)&SD2, &nasa_osal_test_suite);
    OS_TaskDelay(500);
  }
}
