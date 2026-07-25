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

/**
 * @file    TIMv1/hal_st_lld.c
 * @brief   AVR Tiny ST subsystem low level driver source file.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief  Timer maximum value
 */
#define AVR_TIMER_COUNTER_MAX 255

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) || defined(__DOXYGEN__)

/* Work out what the timer interrupt is called on this MCU */
#ifdef TIMER0_COMPA_vect
  #define AVR_TIMER_VECT TIMER0_COMPA_vect
#elif defined(TIMER_COMPA_vect)
  #define AVR_TIMER_VECT TIMER_COMPA_vect
#elif defined(TIMER0_COMP_vect)
  #define AVR_TIMER_VECT TIMER0_COMP_vect
#else
  #error "Cannot find interrupt vector name for timer"
#endif

/* Find the most suitable prescaler setting for the desired OSAL_ST_FREQUENCY */
#if ((F_CPU / OSAL_ST_FREQUENCY) <= AVR_TIMER_COUNTER_MAX)

  #define AVR_TIMER_PRESCALER 1
  #define AVR_TIMER_PRESCALER_BITS ((0<<CS02)|(0<<CS01)|(1<<CS00)) /* CLK      */

#elif ((F_CPU / OSAL_ST_FREQUENCY / 8) <= AVR_TIMER_COUNTER_MAX)

  #define AVR_TIMER_PRESCALER 8
  #define AVR_TIMER_PRESCALER_BITS ((0<<CS02)|(1<<CS01)|(0<<CS00)) /* CLK/8    */

#elif ((F_CPU / OSAL_ST_FREQUENCY / 64) <= AVR_TIMER_COUNTER_MAX)

  #define AVR_TIMER_PRESCALER 64

  #ifdef __AVR_ATmega128__
    #define AVR_TIMER_PRESCALER_BITS ((1<<CS02)|(0<<CS01)|(0<<CS00)) /* CLK/64   */
  #else
    #define AVR_TIMER_PRESCALER_BITS ((0<<CS02)|(1<<CS01)|(1<<CS00)) /* CLK/64   */
  #endif

#elif ((F_CPU / OSAL_ST_FREQUENCY / 256) <= AVR_TIMER_COUNTER_MAX)

  #define AVR_TIMER_PRESCALER 256

  #ifdef __AVR_ATmega128__
    #define AVR_TIMER_PRESCALER_BITS ((1<<CS02)|(1<<CS01)|(0<<CS00)) /* CLK/256  */
  #else
    #define AVR_TIMER_PRESCALER_BITS ((1<<CS02)|(0<<CS01)|(0<<CS00)) /* CLK/256  */
  #endif

#elif ((F_CPU / OSAL_ST_FREQUENCY / 1024) <= AVR_TIMER_COUNTER_MAX)

  #define AVR_TIMER_PRESCALER 1024

  #ifdef __AVR_ATmega128__
    #define AVR_TIMER_PRESCALER_BITS (1<<CS02)|(1<<CS01)|(1<<CS00); /* CLK/1024 */
  #else
    #define AVR_TIMER_PRESCALER_BITS (1<<CS02)|(0<<CS01)|(1<<CS00); /* CLK/1024 */
  #endif

#else
  #error "Frequency too low for timer, please set OSAL_ST_FREQUENCY to a higher value"
#endif

#define AVR_TIMER_COUNTER (F_CPU / OSAL_ST_FREQUENCY / AVR_TIMER_PRESCALER)

/* Test if OSAL_ST_FREQUENCY can be matched exactly using this timer */
#define F_CPU_ (AVR_TIMER_COUNTER * AVR_TIMER_PRESCALER * OSAL_ST_FREQUENCY)
#if (F_CPU_ != F_CPU)
  #warning "OSAL_ST_FREQUENCY cannot be generated exactly using timer"
#endif
#undef F_CPU_

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)

/* FIXME: Prescaler is now fixed in 1024.
 *        Should add support for calculating best value according to
 *        user requested configuration.
 */
#define PRESCALER (_BV(CS12) | _BV(CS10))

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) || defined(__DOXYGEN__)

/**
 * @brief Timer handler for periodic mode.
 */
OSAL_IRQ_HANDLER(AVR_TIMER_VECT) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  osalOsTimerHandlerI();
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)

/**
 * @brief Timer handler for free running mode.
 */
OSAL_IRQ_HANDLER(TIMER1_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();

  /* TODO: reset status if required. */

  osalSysLockFromISR();
  osalOsTimerHandlerI();
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)

  /*
   * Periodic mode uses Timer 1 (16 bit).
   */

  /* CTC mode, no clock source. */
  TCCR1A     = 0;
  TCCR1B     = _BV(WGM12);

  /* Start disabled. */
  TCCR1C     = 0;
  OCR1A      = 0;
  TCNT1      = 0;
  TIFR_REG   = _BV(OCF1A);                              /* Reset pending.   */
  TIMSK_REG  = 0;
  TCCR1B     = PRESCALER;

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if (OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) || defined(__DOXYGEN__)

  /*
   * Periodic mode uses Timer 0 (8 bit).
   */
#if defined(TCCR0B) /* Timer has multiple output comparators.               */
  TCCR0A  = (1 << WGM01) | (0 << WGM00) |               /* CTC mode.        */
            (0 << COM0A1) | (0 << COM0A0);              /* OC0A disabled.   */
            //(0 << COM0B1) | (0 << COM0B0);            /* OC0B disabled.   */
  /* FIXME: See if the line bellow must be delate or recoded.               */
  //TCCR0B  = (0 << WGM02) | AVR_TIMER_PRESCALER_BITS;  /* CTC mode.        */
  OCR0A   = AVR_TIMER_COUNTER - 1;
  TCNT0   = 0;                                          /* Reset counter.   */
#if defined(__AVR_ATtiny85__)
  TIFR    = (1 << OCF0A);                               /* Reset pending.   */
  TIMSK   = (1 << OCIE0A);                              /* IRQ on compare.  */
#else
  TIFR0   = (1 << OCF0A);                               /* Reset pending.   */
  TIMSK0  = (1 << OCIE0A);                              /* IRQ on compare.  */
#endif

#elif defined(TCCR0A) /* AT90CAN doesn't have TCCR0B and slightly different */
                      /* TCCR0A.                                            */
  TCCR0A  = (1 << WGM01) | (0 << WGM00) |               /* CTC mode.        */
            (0 << COM0A1) | (0 << COM0A0);              /* OC0A disabled.   */
  OCR0A   = AVR_TIMER_COUNTER - 1;
  TCNT0   = 0;                                          /* Reset counter.   */
  TIFR0   = (1 << OCF0A);                               /* Reset pending.   */
  TIMSK0  = (1 << OCIE0A);                              /* IRQ on compare.  */

#elif defined(TCCR0) /* Timer has single output comparator                  */
  TCCR0   = (1 << WGM01) | (0 << WGM00) |               /* CTC mode.        */
            (0 << COM01) | (0 << COM00) |               /* OC0A disabled.   */
            AVR_TIMER_PRESCALER_BITS;
  OCR0    = AVR_TIMER_COUNTER - 1;
  TCNT0   = 0;                                          /* Reset counter.   */
  TIFR    = (1 << OCF0);                                /* Reset pending.   */
  TIMSK   = (1 << OCIE0);                               /* IRQ on compare.  */
#else
  #error "Neither TCCR0A nor TCCR0 registers are defined"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
