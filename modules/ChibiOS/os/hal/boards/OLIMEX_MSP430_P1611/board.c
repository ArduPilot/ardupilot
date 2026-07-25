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

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
#if defined(__MSP430_HAS_PORT1__) || defined(__MSP430_HAS_PORT1_R__)
  {VAL_P1OUT, VAL_P1DIR},
#endif
#if defined(__MSP430_HAS_PORT2__) || defined(__MSP430_HAS_PORT2_R__)
  {VAL_P2OUT, VAL_P2DIR},
#endif
#if defined(__MSP430_HAS_PORT3__) || defined(__MSP430_HAS_PORT3_R__)
  {VAL_P3OUT, VAL_P3DIR},
#endif
#if defined(__MSP430_HAS_PORT4__) || defined(__MSP430_HAS_PORT4_R__)
  {VAL_P4OUT, VAL_P4DIR},
#endif
#if defined(__MSP430_HAS_PORT5__) || defined(__MSP430_HAS_PORT5_R__)
  {VAL_P5OUT, VAL_P5DIR},
#endif
#if defined(__MSP430_HAS_PORT6__) || defined(__MSP430_HAS_PORT6_R__)
  {VAL_P6OUT, VAL_P6DIR},
#endif
};
#endif

CH_IRQ_HANDLER(TIMERA0) {

  CH_IRQ_PROLOGUE();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

#if USE_MSP430_USART0
  P3SEL |= (1 << 4) | (1 << 5);
#endif

#if USE_MSP430_USART1
  P3SEL |= (1 << 6) | (1 << 7);
#endif

  /*
   * Timer 0 setup, uses SMCLK as source.
   */
  TACCR0 = SMCLK / 4 / CH_FREQUENCY - 1;/* Counter limit.               */
  TACTL = TACLR;                        /* Clean start.                 */
  TACTL = TASSEL_2 | ID_2 | MC_1;       /* Src=SMCLK, ID=4, cmp=TACCR0. */
  TACCTL0 = CCIE;                       /* Interrupt on compare.        */
}
