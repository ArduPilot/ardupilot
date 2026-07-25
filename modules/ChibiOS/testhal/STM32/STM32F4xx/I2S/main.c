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

#define I2S_BUF_SIZE            256

static uint16_t i2s_rx_buf[I2S_BUF_SIZE];

static void i2scallback(I2SDriver *i2sp);

static const I2SConfig i2scfg = {
  NULL,
  i2s_rx_buf,
  I2S_BUF_SIZE,
  i2scallback,
  0,
  16
};

static void i2scallback(I2SDriver *i2sp) {

  if (i2sIsBufferComplete(i2sp)) {
    /* 2nd buffer half processing.*/
  }
  else {
    /* 1st buffer half processing.*/
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
   * Starting and configuring the I2S driver 2.
   */
  i2sStart(&I2SD2, &i2scfg);
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOC, 3, PAL_MODE_ALTERNATE(5));

  /*
   * Starting continuous I2S transfer.
   */
  i2sStartExchange(&I2SD2);

  /*
   * Normal main() thread activity, if the button is pressed then the I2S
   * transfer is stopped.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_BUTTON))
      i2sStopExchange(&I2SD2);
    chThdSleepMilliseconds(500);
  }
  return 0;
}
