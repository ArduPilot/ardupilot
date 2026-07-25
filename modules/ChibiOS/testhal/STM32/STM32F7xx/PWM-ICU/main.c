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

/*===========================================================================*/
/* PWM driver related.                                                       */
/*===========================================================================*/

static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;
}

static void pwmc1cb(PWMDriver *pwmp) {

  (void)pwmp;
}

static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
  0
};

/*===========================================================================*/
/* ICU driver related.                                                       */
/*===========================================================================*/

icucnt_t last_width, last_period;

static void icuwidthcb(ICUDriver *icup) {

  palSetLine(LINE_ARD_D13);
  last_width = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) {

  palClearLine(LINE_ARD_D13);
  last_period = icuGetPeriodX(icup);
}

static void icuovfcb(ICUDriver *icup) {

  (void)icup;
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  10000,                                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  icuovfcb,
  ICU_CHANNEL_1,
  0,
  0xFFFFFFFFU
};

/*===========================================================================*/
/* Application code.                                                         */
/*===========================================================================*/

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
   * Starting PWM driver 1 and enabling the notifications.
   * GPIOA8 is programmed as PWM output (channel 1 of TIM1).
   */
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetLineMode(LINE_ARD_D5, PAL_MODE_ALTERNATE(1));

  /*
   * Starting ICU driver 2.
   * GPIOA15 is programmed as ICU input (channel 1 of TIM2).
   */
  icuStart(&ICUD2, &icucfg);
  palSetLineMode(LINE_ARD_D9, PAL_MODE_ALTERNATE(1));

  /*
   * GPIOI1 is programmed as output (board LED).
   */
  palClearLine(LINE_ARD_D13);
  palSetLineMode(LINE_ARD_D13, PAL_MODE_OUTPUT_PUSHPULL);
  chThdSleepMilliseconds(1000);

  /*
   * Starting ICU capture and enabling the notifications.
   */
  icuStartCapture(&ICUD2);
  icuEnableNotifications(&ICUD2);

  /*
   * Normal main() thread activity, various PWM patterns are generated
   * cyclically, if the ICU input is connected to the PWM output the
   * board LED mirrors the PWM output.
   */
  while (true) {
    /*
     * Starts the PWM channel 0 using 75% duty cycle.
     */
    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
    pwmEnableChannelNotification(&PWMD1, 0);
    chThdSleepMilliseconds(5000);

    /*
     * Changes the PWM channel 0 to 50% duty cycle.
     */
    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
    chThdSleepMilliseconds(5000);

    /*
     * Changes the PWM channel 0 to 25% duty cycle.
     */
    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
    chThdSleepMilliseconds(5000);

    /*
     * Changes PWM period to half second the duty cycle becomes 50%
     * implicitly.
     */
    pwmChangePeriod(&PWMD1, 5000);
    chThdSleepMilliseconds(5000);

    /*
     * Disables channel 0.
     */
    pwmDisableChannel(&PWMD1, 0);
  }
}
