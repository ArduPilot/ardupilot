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
   This driver is based on the work done by Matteo Serva available at
   http://github.com/matteoserva/ChibiOS-AVR
*/

/**
 * @file    TIMv1/hal_gpt_lld.c
 * @brief   AVR/MEGA GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

#define PRESCALER_SIZE_BASE       5
#define PRESCALER_SIZE_EXTENDED   7

/* FIXME: could use better names here! */
typedef struct {
  volatile uint8_t *tccra;
  volatile uint8_t *tccrb;
  volatile uint8_t *ocr1;
  volatile uint8_t *ocr2;
  volatile uint8_t *tcnt1;
  volatile uint8_t *tcnt2;
  volatile uint8_t *tifr;
  volatile uint8_t *timsk;
} timer_registers_t;

const timer_registers_t regs_table[] = {
#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
  { &TCCR1A, &TCCR1B, &OCR1AH, &OCR1AL, &TCNT1H, &TCNT1L, &TIFR1, &TIMSK1 },
#endif
#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
  { &TCCR2A, &TCCR2B, &OCR2A, &OCR2A, &TCNT2, &TCNT2, &TIFR2, &TIMSK2 },
#endif
#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
  { &TCCR3A, &TCCR3B, &OCR3AH, &OCR3AL, &TCNT3H, &TCNT3L, &TIFR3, &TIMSK3 },
#endif
#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
  { &TCCR4A, &TCCR4B, &OCR4AH, &OCR4AL, &TCNT4H, &TCNT4L, &TIFR4, &TIMSK4 },
#endif
#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
  { &TCCR5A, &TCCR5B, &OCR5AH, &OCR5AL, &TCNT5H, &TCNT5L, &TIFR5, &TIMSK5 },
#endif
};

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif
#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif
#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif
#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif
#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif

/*==========================================================================*/
/* Driver local variables.                                                  */
/*==========================================================================*/

static uint16_t ratio_base[] = { 1024, 256, 64, 8, 1 };
static uint8_t  clock_source_base[] = { 5, 4, 3, 2, 1 };

/* Extended tables.
 * static uint16_t ratio_extended[] = { 1024, 256, 128, 64, 32, 8, 1 };
 * static uint8_t  clock_source_extended[] = { 7, 6, 5, 4, 3, 2, 1 };
 */

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   TODO: Comment this function.
 *
 * @param[in] freq
 * @param[in] ratio   pointer to the ratio used to calculate the prescaler
 * @param[in] n       ....
 * @return            ....
 */
static uint8_t prescaler(uint16_t freq, uint16_t *ratio, uint8_t n) {

  uint8_t i;

  for (i = 0; i < n; ++i) {
    uint32_t result = F_CPU / ratio[i] / freq;
    if (result > 256UL)
       return i - 1;
    if ((result * ratio[i] * freq) == F_CPU)
      return i;
  }
  return -1;
}

/**
 * @brief   TODO: Comment this function.
 *
 * @param[in] gptp  pointer to the General Purpose Timer driver.
 */
static void gpt_lld_serve_interrupt(GPTDriver *gptp) {

  gptp->counter++;
  if (gptp->counter == gptp->period) {
    gptp->counter = 0;
    if (gptp->state == GPT_ONESHOT) {
      gptp->state = GPT_READY;             /* Back in GPT_READY state.     */
      gpt_lld_stop_timer(gptp);            /* Timer automatically stopped. */
    }
    gptp->callback(gptp);
  }
}

/**
 * @brief   GPT low level driver dummy callback.
 *
 * @param[in] gptp  pointer to the General Purpose Timer driver.
 */
static void gpt_lld_dummy_callback(GPTDriver *gptp) {
}

/**
 * @brief   Get the timer index.
 *
 * @param[in] gptp  pointer to the General Purpose Timer driver.
 */
static uint8_t getTimerIndex(GPTDriver *gptp) {

  uint8_t index = 0;

#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
  if (gptp == &GPTD1) return index;
  else index++;
#endif
#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
  if (gptp == &GPTD2) return index;
  else index++;
#endif
#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
  if (gptp == &GPTD3) return index;
  else index++;
#endif
#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
  if (gptp == &GPTD4) return index;
  else index++;
#endif
#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
  if (gptp == &GPTD5) return index;
  else index++;
#endif
  return -1;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TIMER1_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD1);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TIMER2_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD2);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TIMER3_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD3);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
/**
 * @brief   TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TIMER4_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD4);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(TIMER5_COMPA_vect) {

  OSAL_IRQ_PROLOGUE();
  gpt_lld_serve_interrupt(&GPTD5);
  OSAL_IRQ_EPILOGUE();
}
#endif

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if AVR_GPT_USE_TIM1 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD1);
#endif
#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD2);
#endif
#if AVR_GPT_USE_TIM3 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD3);
#endif
#if AVR_GPT_USE_TIM4 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD4);
#endif
#if AVR_GPT_USE_TIM5 || defined(__DOXYGEN__)
  gptObjectInit(&GPTD5);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp    pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {

  uint8_t psc;

  if (gptp->state == GPT_STOP) {
    /* Clock activation.*/
  }

  /* Configuration.*/

#if AVR_GPT_USE_TIM2 || defined(__DOXYGEN__)
  if (gptp == &GPTD2) {
    psc = prescaler(gptp->config->frequency, ratio_extended, PRESCALER_SIZE_EXTENDED);
    gptp->clock_source = clock_source_extended[psc] & 0x07;
    TCCR2A  = (1 << WGM21) | (0 << WGM20);
    TCCR2B  = (0 << WGM22);
    OCR2A = F_CPU / ratio_extended[psc] /gptp->config->frequency - 1;
    return;
  }
#endif

  uint8_t i = getTimerIndex(gptp);
  psc = prescaler(gptp->config->frequency, ratio_base, PRESCALER_SIZE_BASE);
  gptp->clock_source = clock_source_base[psc] & 0x07;
  *regs_table[i].tccra = (0 << WGM11)  |
                         (0 << WGM10)  |
                         (0 << COM1A1) |
                         (0 << COM1A0) |
                         (0 << COM1B1) |
                         (0 << COM1B0);
  *regs_table[i].tccrb = (1 << WGM12);
  *regs_table[i].ocr1 = 0;
  *regs_table[i].ocr2 = F_CPU / ratio_base[psc] / gptp->config->frequency - 1;
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp    pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  /* Nothing to be done */
  if (gptp->state == GPT_READY) {
    /* Clock de-activation.*/
  }
  gpt_lld_stop_timer(gptp);
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] period    period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t period) {

  gptp->callback = gptp->config->callback;
  gptp->period = period;
  gptp->counter = 0;

  uint8_t i = getTimerIndex(gptp);
  *regs_table[i].tcnt1 = 0;
  *regs_table[i].tcnt2 = 0;
  *regs_table[i].tifr  = (1 << OCF1A);
  *regs_table[i].timsk = (1 << OCIE1A);
  *regs_table[i].tccrb |= (gptp->clock_source << CS10);
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp    pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  uint8_t i = getTimerIndex(gptp);

  *regs_table[i].tccrb &= ~((7 << CS10) | (1 << OCIE1A));
  *regs_table[i].tifr = (1 << OCF1A);
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  gptp->callback = gpt_lld_dummy_callback;
  gpt_lld_start_timer(gptp, interval);
  /* FIX */
  while (gptp->state != GPT_READY) {}
}

#endif /* HAL_USE_GPT */

/** @} */
