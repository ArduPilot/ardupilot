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

/*
 * Timing values are taken from the RM except the PRESC set to 9 because
 * the input clock is 72MHz.
 * The timings are critical, please always refer to the STM32 Reference
 * Manual before attempting changes.
 */
#if 0
static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(8U)  |            /* 72MHz/9 = 8MHz I2CCLK.           */
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  0,
  0
};
#endif
static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(15U) |
  STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
  STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(21U),
  0,
  0
};

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(blinker_wa, 128);
static THD_FUNCTION(blinker, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOC, GPIOC_LED1);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOC, GPIOC_LED1);
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
   * Starting the I2C driver 2.
   */
  i2cStart(&I2CD2, &i2cconfig);

  /*
   * Starting the blinker thread.
   */
  chThdCreateStatic(blinker_wa, sizeof(blinker_wa),
                    NORMALPRIO-1, blinker, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    unsigned i;
    msg_t msg;
    static const uint8_t cmd[] = {0, 0};
    uint8_t data[16];

    msg = i2cMasterTransmitTimeout(&I2CD2, 0x52, cmd, sizeof(cmd),
                                   data, sizeof(data), TIME_INFINITE);
    if (msg != MSG_OK)
      palTogglePad(GPIOC, GPIOC_LED3);
    for (i = 0; i < 256; i++) {
      msg = i2cMasterReceiveTimeout(&I2CD2, 0x52,
                                    data, sizeof(data), TIME_INFINITE);
      if (msg != MSG_OK)
        palTogglePad(GPIOC, GPIOC_LED3);
    }
    chThdSleepMilliseconds(500);
    palTogglePad(GPIOC, GPIOC_LED2);
  }
  return 0;
}
