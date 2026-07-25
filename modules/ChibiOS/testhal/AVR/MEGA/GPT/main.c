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
#include "chprintf.h"

BaseSequentialStream * chp = (BaseSequentialStream *) &SD1;

static void gpt3cb(GPTDriver *gptp) {
  palTogglePad(IOPORT2, 7);
}

static GPTConfig gpt3cfg = {
  1000,         /* Timer clock.    */
  gpt3cb        /* Timer callback. */
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

  palSetPadMode(IOPORT2, 7, PAL_MODE_OUTPUT_PUSHPULL);

  sdStart(&SD1, NULL);
  gptStart(&GPTD3, &gpt3cfg);

  gptStartContinuous(&GPTD3, 500);

  while (1) {
    chprintf(chp, "OCR3A: %d, TCCR3B: %x, period: %d, counter: %d , TCNT3: %d\r\n",
                   OCR3A,
                   TCCR3B,
                   GPTD3.period,
                   GPTD3.counter,
                   TCNT3);
    chThdSleepMilliseconds(100);
  }
}
