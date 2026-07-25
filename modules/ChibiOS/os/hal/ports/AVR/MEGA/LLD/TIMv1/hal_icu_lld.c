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
 * @file    TIMv1/hal_icu_lld.c
 * @brief   AVR/MEGA ICU subsystem low level driver source.
 *
 * @addtogroup ICU
 * @{
 */

#include "hal.h"

#if HAL_USE_ICU || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/**
 * @brief   ICU driver type.
 */
typedef struct {
  volatile uint8_t  *tccra;
  volatile uint8_t  *tccrb;
  volatile uint16_t *tcnt;
  volatile uint8_t  *timsk;
} icu_registers_t;

static icu_registers_t regs_table[] =
{
#if AVR_ICU_USE_TIM1 || defined(__DOXYGEN__)
  {&TCCR1A, &TCCR1B, &TCNT1, &TIMSK1},
#endif
#if AVR_ICU_USE_TIM3 || defined(__DOXYGEN__)
  {&TCCR3A, &TCCR3B, &TCNT3, &TIMSK3},
#endif
#if AVR_ICU_USE_TIM4 || defined(__DOXYGEN__)
  {&TCCR4A, &TCCR4B, &TCNT4, &TIMSK4},
#endif
#if AVR_ICU_USE_TIM5 || defined(__DOXYGEN__)
  {&TCCR5A, &TCCR5B, &TCNT5, &TIMSK5},
#endif
};

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief   ICU1 driver identifier.
 */
#if AVR_ICU_USE_TIM1 || defined(__DOXYGEN__)
ICUDriver ICUD1;
#endif
/**
 * @brief   ICU3 driver identifier.
 */
#if AVR_ICU_USE_TIM3 || defined(__DOXYGEN__)
ICUDriver ICUD3;
#endif
/**
 * @brief   ICU4 driver identifier.
 */
#if AVR_ICU_USE_TIM4 || defined(__DOXYGEN__)
ICUDriver ICUD4;
#endif
/**
 * @brief   ICU5 driver identifier.
 */
#if AVR_ICU_USE_TIM5 || defined(__DOXYGEN__)
ICUDriver ICUD5;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/* TODO: Comment. */
static inline void handle_capture_isr(ICUDriver *icup,
                                      volatile uint16_t *icr,
                                      volatile uint8_t  *tccrb,
                                      volatile uint16_t *tcnt) {

  uint16_t value = *icr;
  uint8_t rising = (*tccrb & (1 << ICES1)) ? 1 : 0;
  *tccrb ^= (1 << ICES1);
  if ((icup->config->mode == ICU_INPUT_ACTIVE_HIGH && rising) ||
      (icup->config->mode == ICU_INPUT_ACTIVE_LOW  && !rising)) {
   icup->width = value;
   if (icup->config->width_cb != NULL)
     icup->config->width_cb(icup);
  } else {
   icup->period = value;
   if (icup->config->period_cb != NULL)
     icup->config->period_cb(icup);
   /* Reset counter at the end of every cycle. */
   *tcnt = 0;
  }
}

/* TODO: Comment. */
static uint8_t tmrIndex(ICUDriver *icup) {

  uint8_t index = 0;
#if AVR_ICU_USE_TIM1 || defined(__DOXYGEN__)
  if (icup == &ICUD1) return index;
  else index++;
#endif
#if AVR_ICU_USE_TIM3 || defined(__DOXYGEN__)
  if (icup == &ICUD3) return index;
  else index++;
#endif
#if AVR_ICU_USE_TIM4 || defined(__DOXYGEN__)
  if (icup == &ICUD4) return index;
  else index++;
#endif
#if AVR_ICU_USE_TIM5 || defined(__DOXYGEN__)
  if (icup == &ICUD5) return index;
  else index++;
#endif
  return 255;
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_ICU_USE_TIM1 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(TIMER1_CAPT_vect) {

  OSAL_IRQ_PROLOGUE();
  handle_capture_isr(&ICUD1, &ICR1, &TCCR1B, &TCNT1);
  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(TIMER1_OVF_vect) {

  OSAL_IRQ_PROLOGUE();
  ICUD1.config->overflow_cb(&ICUD1);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_ICU_USE_TIM3 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(TIMER3_CAPT_vect) {

  OSAL_IRQ_PROLOGUE();
  handle_capture_isr(&ICUD3, &ICR3, &TCCR3B, &TCNT3);
  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(TIMER3_OVF_vect) {

  OSAL_IRQ_PROLOGUE();
  ICUD3.config->overflow_cb(&ICUD3);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_ICU_USE_TIM4 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(TIMER4_CAPT_vect) {

  OSAL_IRQ_PROLOGUE();
  handle_capture_isr(&ICUD4, &ICR4, &TCCR4B, &TCNT4);
  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(TIMER4_OVF_vect) {

  OSAL_IRQ_PROLOGUE();
  ICUD4.config->overflow_cb(&ICUD4);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_ICU_USE_TIM5 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(TIMER5_CAPT_vect) {

  OSAL_IRQ_PROLOGUE();
  handle_capture_isr(&ICUD5, &ICR5, &TCCR5B, &TCNT5);
  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(TIMER5_OVF_vect) {

  OSAL_IRQ_PROLOGUE();
  ICUD5.config->overflow_cb(&ICUD5);
  OSAL_IRQ_EPILOGUE();
}
#endif

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level ICU driver initialization.
 *
 * @notapi
 */
void icu_lld_init(void) {

#if AVR_ICU_USE_TIM1
  icuObjectInit(&ICUD1);
#endif
#if AVR_ICU_USE_TIM3
  icuObjectInit(&ICUD3);
#endif
#if AVR_ICU_USE_TIM4
  icuObjectInit(&ICUD4);
#endif
#if AVR_ICU_USE_TIM5
  icuObjectInit(&ICUD5);
#endif
}

/**
 * @brief   Configures and activates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_start(ICUDriver *icup) {

  if (icup->state == ICU_STOP) {
    uint8_t i = tmrIndex(icup);
    /* Normal waveform generation (counts from 0 to 0xFFFF). */
    *regs_table[i].tccra &= ~((1 << WGM11) | (1 << WGM10));
    *regs_table[i].tccrb &= ~((1 << WGM13) | (1 << WGM12));
    /* Enable noise canceler, set prescale to CLK/1024. */
    *regs_table[i].tccrb |= (1 << ICNC1) | (1 << CS12) | (1 << CS10);
    if (icup->config->mode == ICU_INPUT_ACTIVE_HIGH)
      *regs_table[i].tccrb |= (1 << ICES1);
    else
      *regs_table[i].tccrb &= ~(1 << ICES1);
  }
}

/**
 * @brief   Deactivates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_stop(ICUDriver *icup) {

  if (icup->state == ICU_READY) {
    /* Resets the peripheral. */

    /* Disables the peripheral. */
#if AVR_ICU_USE_TIM1
    if (&ICUD1 == icup) {

    }
#endif /* AVR_ICU_USE_TIM1 */
  }
}

/**
 * @brief   Enables the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_start_capture(ICUDriver *icup) {

  uint8_t i = tmrIndex(icup);
  icup->width = icup->period = 0;
  *regs_table[i].tcnt = 0;
  *regs_table[i].timsk |= (1 << ICIE1);
  if (icup->config->overflow_cb != NULL)
    *regs_table[i].timsk |= (1 << TOIE1);
}

/**
 * @brief   Disables the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_stop_capture(ICUDriver *icup) {

  uint8_t i = tmrIndex(icup);
  *regs_table[i].timsk &= ~((1 << ICIE1) | (1 << TOIE1));
}

/**
 * @brief   Returns the width of the latest pulse.
 * @details The pulse width is defined as number of ticks between the start
 *          edge and the stop edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
icucnt_t icu_lld_get_width(ICUDriver *icup) {

  return icup->width;
}

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
icucnt_t icu_lld_get_period(ICUDriver *icup) {

  return icup->period;
}

#endif /* HAL_USE_ICU */

/** @} */
