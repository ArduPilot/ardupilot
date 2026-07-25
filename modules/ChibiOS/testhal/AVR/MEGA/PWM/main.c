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
#include "ch.h"

#define PERIOD  0x7FFF

/**
 * @brief   PWM configuration structure.
 */
static PWMConfig pwm3cfg = {
  F_CPU,                            /* PWM frequency.         */
  PERIOD,                           /* PWM period.            */
  NULL,                             /* TODO: comment.         */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 1 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 2 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 3 actived. */
  },
};

/**
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

  /* PE3-5 are timer 3 pwm channel outputs. */
  palSetPadMode(IOPORT5, 3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT5, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT5, 5, PAL_MODE_OUTPUT_PUSHPULL);

  pwmStart(&PWMD3, &pwm3cfg);
  /* Channel 0 with 50% duty cycle, 1 with 25% and 2 with 75% */
  pwmEnableChannel(&PWMD3, 0, PERIOD >> 1);
  pwmEnableChannel(&PWMD3, 1, PERIOD >> 2);
  pwmEnableChannel(&PWMD3, 2, (PERIOD >> 2)*3);

  while (TRUE) {
  }
}

