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
#if defined(PORTA)
  {VAL_PORTA, VAL_DDRA},
#endif
#if defined(PORTB)
  {VAL_PORTB, VAL_DDRB},
#endif
#if defined(PORTC)
  {VAL_PORTC, VAL_DDRC},
#endif
#if defined(PORTD)
  {VAL_PORTD, VAL_DDRD},
#endif
#if defined(PORTE)
  {VAL_PORTE, VAL_DDRE},
#endif
#if defined(PORTF)
  {VAL_PORTF, VAL_DDRF},
#endif
#if defined(PORTG)
  {VAL_PORTG, VAL_DDRG},
#endif
};
#endif /* HAL_USE_PAL */

CH_IRQ_HANDLER(TIMER0_COMP_vect) {

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

  /*
   * External interrupts setup, all disabled initially.
   */
  EICRA  = 0x00;
  EICRB  = 0x00;
  EIMSK  = 0x00;

  /*
   * Enables Idle mode for SLEEP instruction.
   */
  SMCR   = (1 << SE);

  /*
   * Timer 0 setup.
   */
  TCCR0A = (1 << WGM01)  | (0 << WGM00)  |              /* CTC mode.        */
           (0 << COM0A1) | (0 << COM0A0) |              /* OC0A disabled.   */
           (0 << CS02)   | (1 << CS01)   | (1 << CS00); /* CLK/64 clock.    */
  OCR0A  = F_CPU / 64 / CH_FREQUENCY - 1;
  TCNT0  = 0;                                           /* Reset counter.   */
  TIFR0  = (1 << OCF0A);                                /* Reset pending.   */
  TIMSK0 = (1 << OCIE0A);                               /* IRQ on compare.  */
}
