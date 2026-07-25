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

/*
 * Buzzer driver for Olimex LPC-P2148.
 * Uses the timer 1 for wave generation and a Virtual Timer for the sound
 * duration.
 * The driver also generates an event when the sound is done and the buzzer
 * goes silent.
 */

#include "hal.h"

#include "buzzer.h"

EventSource BuzzerSilentEventSource;

#define StartCounter(t) ((t)->TC_EMR = 0xF1, (t)->TC_TCR = 1)
#define StopCounter(t)  ((t)->TC_EMR = 0, (t)->TC_TCR = 2)

/**
 * @brief Buzzer driver initialization.
 */
void buzzInit(void) {

  chEvtInit(&BuzzerSilentEventSource);

  /*
   * Switches P0.12 and P0.13 to MAT1.0 and MAT1.1 functions.
   * Enables Timer1 clock.
   */
  PINSEL0 &= 0xF0FFFFFF;
  PINSEL0 |= 0x0A000000;
  PCONP = (PCONP & PCALL) | PCTIM1;

  /*
   * Timer setup.
   */
  TC *tc = T1Base;
  StopCounter(tc);
  tc->TC_CTCR = 0;                      /* Clock source is PCLK.        */
  tc->TC_PR   = 0;                      /* Prescaler disabled.          */
  tc->TC_MCR  = 2;                      /* Clear TC on match MR0.       */
}

/**
 * @brief Stops the sound.
 *
 * @param[in] p pointer to the timer
 */
static void stop(void *p) {

  StopCounter((TC *)p);
  chSysLockFromIsr();
  chEvtBroadcastI(&BuzzerSilentEventSource);
  chSysUnlockFromIsr();
}

/**
 * @brief Plays a tone asynchronously.
 *
 * @param[in] freq approximated tone frequency
 * @param[in] duration tone duration in systicks
 */
void buzzPlay(uint32_t freq, systime_t duration) {
  static VirtualTimer bvt;
  TC *tc = T1Base;

  chSysLock();

  if (chVTIsArmedI(&bvt)) {             /* If a sound is already being  */
    chVTResetI(&bvt);                   /* played then aborts it.       */
    StopCounter(tc);
  }

  tc->TC_MR0 = tc->TC_MR1 = (PCLK / (freq * 2));
  StartCounter(tc);
  chVTSetI(&bvt, duration, stop, tc);

  chSysUnlock();
}

/**
 * @brief Plays a tone.
 *
 * @param[in] freq approximated tone frequency
 * @param[in] duration tone duration in systicks
 */
void buzzPlayWait(uint32_t freq, systime_t duration) {
  TC *tc = T1Base;

  StopCounter(tc);
  tc->TC_MR0 = tc->TC_MR1 = (PCLK / (freq * 2));
  StartCounter(tc);
  chThdSleep(duration);
  StopCounter(tc);
}
