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
   Concepts and parts of this file have been contributed by Fabio Utzig and
   Xo Wang.
 */

/**
 * @file    TIMv1/hal_icu_lld.c
 * @brief   STM32 ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#include "hal.h"

#if HAL_USE_ICU || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ICUD1 driver identifier.
 * @note    The driver ICUD1 allocates the complex timer TIM1 when enabled.
 */
#if STM32_ICU_USE_TIM1 || defined(__DOXYGEN__)
ICUDriver ICUD1;
#endif

/**
 * @brief   ICUD2 driver identifier.
 * @note    The driver ICUD1 allocates the timer TIM2 when enabled.
 */
#if STM32_ICU_USE_TIM2 || defined(__DOXYGEN__)
ICUDriver ICUD2;
#endif

/**
 * @brief   ICUD3 driver identifier.
 * @note    The driver ICUD1 allocates the timer TIM3 when enabled.
 */
#if STM32_ICU_USE_TIM3 || defined(__DOXYGEN__)
ICUDriver ICUD3;
#endif

/**
 * @brief   ICUD4 driver identifier.
 * @note    The driver ICUD4 allocates the timer TIM4 when enabled.
 */
#if STM32_ICU_USE_TIM4 || defined(__DOXYGEN__)
ICUDriver ICUD4;
#endif

/**
 * @brief   ICUD5 driver identifier.
 * @note    The driver ICUD5 allocates the timer TIM5 when enabled.
 */
#if STM32_ICU_USE_TIM5 || defined(__DOXYGEN__)
ICUDriver ICUD5;
#endif

/**
 * @brief   ICUD8 driver identifier.
 * @note    The driver ICUD8 allocates the timer TIM8 when enabled.
 */
#if STM32_ICU_USE_TIM8 || defined(__DOXYGEN__)
ICUDriver ICUD8;
#endif

/**
 * @brief   ICUD9 driver identifier.
 * @note    The driver ICUD9 allocates the timer TIM9 when enabled.
 */
#if STM32_ICU_USE_TIM9 || defined(__DOXYGEN__)
ICUDriver ICUD9;
#endif

/**
 * @brief   ICUD10 driver identifier.
 * @note    The driver ICUD10 allocates the timer TIM10 when enabled.
 */
#if STM32_ICU_USE_TIM10 || defined(__DOXYGEN__)
ICUDriver ICUD10;
#endif

/**
 * @brief   ICUD11 driver identifier.
 * @note    The driver ICUD11 allocates the timer TIM11 when enabled.
 */
#if STM32_ICU_USE_TIM11 || defined(__DOXYGEN__)
ICUDriver ICUD11;
#endif

/**
 * @brief   ICUD12 driver identifier.
 * @note    The driver ICUD12 allocates the timer TIM12 when enabled.
 */
#if STM32_ICU_USE_TIM12 || defined(__DOXYGEN__)
ICUDriver ICUD12;
#endif

/**
 * @brief   ICUD13 driver identifier.
 * @note    The driver ICUD13 allocates the timer TIM13 when enabled.
 */
#if STM32_ICU_USE_TIM13 || defined(__DOXYGEN__)
ICUDriver ICUD13;
#endif

/**
 * @brief   ICUD14 driver identifier.
 * @note    The driver ICUD14 allocates the timer TIM14 when enabled.
 */
#if STM32_ICU_USE_TIM14 || defined(__DOXYGEN__)
ICUDriver ICUD14;
#endif

/**
 * @brief   ICUD15 driver identifier.
 * @note    The driver ICUD15 allocates the timer TIM15 when enabled.
 */
#if STM32_ICU_USE_TIM15 || defined(__DOXYGEN__)
ICUDriver ICUD15;
#endif

/**
 * @brief   ICUD20 driver identifier.
 * @note    The driver ICUD20 allocates the timer TIM20 when enabled.
 */
#if STM32_ICU_USE_TIM20 || defined(__DOXYGEN__)
ICUDriver ICUD20;
#endif

/**
 * @brief   ICUD21 driver identifier.
 * @note    The driver ICUD21 allocates the timer TIM21 when enabled.
 */
#if STM32_ICU_USE_TIM21 || defined(__DOXYGEN__)
ICUDriver ICUD21;
#endif

/**
 * @brief   ICUD22 driver identifier.
 * @note    The driver ICUD22 allocates the timer TIM22 when enabled.
 */
#if STM32_ICU_USE_TIM22 || defined(__DOXYGEN__)
ICUDriver ICUD22;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool icu_lld_wait_edge(ICUDriver *icup) {
  uint32_t sr;
  bool result;

  /* Polled mode so re-enabling the interrupts while the operation is
     performed.*/
  osalSysUnlock();

  /* Polling the right bit depending on the input channel.*/
  if (icup->config->channel == ICU_CHANNEL_1) {
    /* Waiting for an edge.*/
    while (((sr = icup->tim->SR) &
            (STM32_TIM_SR_CC1IF | STM32_TIM_SR_UIF)) == 0)
      ;
  }
  else {
    /* Waiting for an edge.*/
    while (((sr = icup->tim->SR) &
            (STM32_TIM_SR_CC2IF | STM32_TIM_SR_UIF)) == 0)
      ;
  }

  /* Edge or overflow?*/
  result = (sr & STM32_TIM_SR_UIF) != 0 ? true : false;

  /* Done, disabling interrupts again.*/
  osalSysLock();

  /* Resetting all flags.*/
  icup->tim->SR &= ~(STM32_TIM_SR_CC1IF |
                     STM32_TIM_SR_CC2IF |
                     STM32_TIM_SR_UIF);

  return result;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_ICU_USE_TIM1 || defined(__DOXYGEN__)
#if !defined(STM32_TIM1_SUPPRESS_ISR)
#if !defined(STM32_TIM1_UP_HANDLER)
#error "STM32_TIM1_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM1_CC_HANDLER)
#error "STM32_TIM1_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM1_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM1 */

#if STM32_ICU_USE_TIM2 || defined(__DOXYGEN__)
#if !defined(STM32_TIM2_SUPPRESS_ISR)
#if !defined(STM32_TIM2_HANDLER)
#error "STM32_TIM2_HANDLER not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM2_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM2 */

#if STM32_ICU_USE_TIM3 || defined(__DOXYGEN__)
#if !defined(STM32_TIM3_SUPPRESS_ISR)
#if !defined(STM32_TIM3_HANDLER)
#error "STM32_TIM3_HANDLER not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM3_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM3 */

#if STM32_ICU_USE_TIM4 || defined(__DOXYGEN__)
#if !defined(STM32_TIM4_SUPPRESS_ISR)
#if !defined(STM32_TIM4_HANDLER)
#error "STM32_TIM4_HANDLER not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM4_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM4 */

#if STM32_ICU_USE_TIM5 || defined(__DOXYGEN__)
#if !defined(STM32_TIM5_SUPPRESS_ISR)
#if !defined(STM32_TIM5_HANDLER)
#error "STM32_TIM5_HANDLER not defined"
#endif
/**
 * @brief   TIM5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM5_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM5 */

#if STM32_ICU_USE_TIM8 || defined(__DOXYGEN__)
#if !defined(STM32_TIM8_SUPPRESS_ISR)
#if !defined(STM32_TIM8_UP_HANDLER)
#error "STM32_TIM8_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM8 update interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD8);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM8_CC_HANDLER)
#error "STM32_TIM8_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD8);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM8_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM8 */

#if STM32_ICU_USE_TIM9 || defined(__DOXYGEN__)
#if !defined(STM32_TIM9_SUPPRESS_ISR)
#error "TIM9 ISR not defined by platform"
#endif /* !defined(STM32_TIM9_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM9 */

#if STM32_ICU_USE_TIM10 || defined(__DOXYGEN__)
#if !defined(STM32_TIM10_SUPPRESS_ISR)
#error "TIM10 ISR not defined by platform"
#endif /* !defined(STM32_TIM10_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM10 */

#if STM32_ICU_USE_TIM11 || defined(__DOXYGEN__)
#if !defined(STM32_TIM11_SUPPRESS_ISR)
#error "TIM11 ISR not defined by platform"
#endif /* !defined(STM32_TIM11_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM11 */

#if STM32_ICU_USE_TIM12 || defined(__DOXYGEN__)
#if !defined(STM32_TIM12_SUPPRESS_ISR)
#error "TIM12 ISR not defined by platform"
#endif /* !defined(STM32_TIM12_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM12 */

#if STM32_ICU_USE_TIM13 || defined(__DOXYGEN__)
#if !defined(STM32_TIM13_SUPPRESS_ISR)
#error "TIM13 ISR not defined by platform"
#endif /* !defined(STM32_TIM13_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM13 */

#if STM32_ICU_USE_TIM14 || defined(__DOXYGEN__)
#if !defined(STM32_TIM14_SUPPRESS_ISR)
#error "TIM14 ISR not defined by platform"
#endif /* !defined(STM32_TIM14_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM14 */

#if STM32_ICU_USE_TIM15 || defined(__DOXYGEN__)
#if !defined(STM32_TIM15_SUPPRESS_ISR)
#error "TIM15 ISR not defined by platform"
#endif /* !defined(STM32_TIM15_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM15 */

#if STM32_ICU_USE_TIM20 || defined(__DOXYGEN__)
#if !defined(STM32_TIM20_SUPPRESS_ISR)
#if !defined(STM32_TIM20_UP_HANDLER)
#error "STM32_TIM20_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM20 update interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM20_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD20);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM20_CC_HANDLER)
#error "STM32_TIM20_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM20 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM20_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD20);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM20_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM20 */

#if STM32_ICU_USE_TIM21 || defined(__DOXYGEN__)
#if !defined(STM32_TIM21_SUPPRESS_ISR)
#error "TIM21 ISR not defined by platform"
#endif /* !defined(STM32_TIM21_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM21 */

#if STM32_ICU_USE_TIM22 || defined(__DOXYGEN__)
#if !defined(STM32_TIM22_SUPPRESS_ISR)
#error "TIM22 ISR not defined by platform"
#endif /* !defined(STM32_TIM22_SUPPRESS_ISR) */
#endif /* STM32_ICU_USE_TIM22 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ICU driver initialization.
 *
 * @notapi
 */
void icu_lld_init(void) {

#if STM32_ICU_USE_TIM1
  /* Driver initialization.*/
  icuObjectInit(&ICUD1);
  ICUD1.tim = STM32_TIM1;
#endif

#if STM32_ICU_USE_TIM2
  /* Driver initialization.*/
  icuObjectInit(&ICUD2);
  ICUD2.tim = STM32_TIM2;
#endif

#if STM32_ICU_USE_TIM3
  /* Driver initialization.*/
  icuObjectInit(&ICUD3);
  ICUD3.tim = STM32_TIM3;
#endif

#if STM32_ICU_USE_TIM4
  /* Driver initialization.*/
  icuObjectInit(&ICUD4);
  ICUD4.tim = STM32_TIM4;
#endif

#if STM32_ICU_USE_TIM5
  /* Driver initialization.*/
  icuObjectInit(&ICUD5);
  ICUD5.tim = STM32_TIM5;
#endif

#if STM32_ICU_USE_TIM8
  /* Driver initialization.*/
  icuObjectInit(&ICUD8);
  ICUD8.tim = STM32_TIM8;
#endif

#if STM32_ICU_USE_TIM9
  /* Driver initialization.*/
  icuObjectInit(&ICUD9);
  ICUD9.tim = STM32_TIM9;
#endif

#if STM32_ICU_USE_TIM10
  /* Driver initialization.*/
  icuObjectInit(&ICUD10);
  ICUD10.tim = STM32_TIM10;
#endif

#if STM32_ICU_USE_TIM11
  /* Driver initialization.*/
  icuObjectInit(&ICUD11);
  ICUD11.tim = STM32_TIM11;
#endif

#if STM32_ICU_USE_TIM12
  /* Driver initialization.*/
  icuObjectInit(&ICUD12);
  ICUD12.tim = STM32_TIM12;
#endif

#if STM32_ICU_USE_TIM13
  /* Driver initialization.*/
  icuObjectInit(&ICUD13);
  ICUD13.tim = STM32_TIM13;
#endif

#if STM32_ICU_USE_TIM14
  /* Driver initialization.*/
  icuObjectInit(&ICUD14);
  ICUD14.tim = STM32_TIM14;
#endif

#if STM32_ICU_USE_TIM15
  /* Driver initialization.*/
  icuObjectInit(&ICUD15);
  ICUD15.tim = STM32_TIM15;
#endif

#if STM32_ICU_USE_TIM20
  /* Driver initialization.*/
  icuObjectInit(&ICUD20);
  ICUD20.tim = STM32_TIM20;
#endif

#if STM32_ICU_USE_TIM21
  /* Driver initialization.*/
  icuObjectInit(&ICUD21);
  ICUD21.tim = STM32_TIM21;
#endif

#if STM32_ICU_USE_TIM22
  /* Driver initialization.*/
  icuObjectInit(&ICUD22);
  ICUD22.tim = STM32_TIM22;
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
  uint32_t psc;

  osalDbgAssert((icup->config->channel == ICU_CHANNEL_1) ||
                (icup->config->channel == ICU_CHANNEL_2),
                "invalid input");

  if (icup->state == ICU_STOP) {
    /* Clock activation and timer reset.*/
#if STM32_ICU_USE_TIM1
    if (&ICUD1 == icup) {
      rccEnableTIM1(true);
      rccResetTIM1();
#if !defined(STM32_TIM1_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM1_UP_NUMBER, STM32_ICU_TIM1_IRQ_PRIORITY);
      nvicEnableVector(STM32_TIM1_CC_NUMBER, STM32_ICU_TIM1_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM1CLK)
      icup->clock = STM32_TIM1CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM2
    if (&ICUD2 == icup) {
      rccEnableTIM2(true);
      rccResetTIM2();
#if !defined(STM32_TIM2_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM2_NUMBER, STM32_ICU_TIM2_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM2CLK)
      icup->clock = STM32_TIM2CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM3
    if (&ICUD3 == icup) {
      rccEnableTIM3(true);
      rccResetTIM3();
#if !defined(STM32_TIM3_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM3_NUMBER, STM32_ICU_TIM3_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM3CLK)
      icup->clock = STM32_TIM3CLK;
#else
     icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM4
    if (&ICUD4 == icup) {
      rccEnableTIM4(true);
      rccResetTIM4();
#if !defined(STM32_TIM4_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM4_NUMBER, STM32_ICU_TIM4_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM4CLK)
      icup->clock = STM32_TIM4CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM5
    if (&ICUD5 == icup) {
      rccEnableTIM5(true);
      rccResetTIM5();
#if !defined(STM32_TIM5_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM5_NUMBER, STM32_ICU_TIM5_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM5CLK)
      icup->clock = STM32_TIM5CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM8
    if (&ICUD8 == icup) {
      rccEnableTIM8(true);
      rccResetTIM8();
#if !defined(STM32_TIM8_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM8_UP_NUMBER, STM32_ICU_TIM8_IRQ_PRIORITY);
      nvicEnableVector(STM32_TIM8_CC_NUMBER, STM32_ICU_TIM8_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM8CLK)
      icup->clock = STM32_TIM8CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM9
    if (&ICUD9 == icup) {
      rccEnableTIM9(true);
      rccResetTIM9();
#if defined(STM32_TIM9CLK)
      icup->clock = STM32_TIM9CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM10
    if (&ICUD10 == icup) {
      rccEnableTIM10(true);
      rccResetTIM10();
#if defined(STM32_TIM10CLK)
      icup->clock = STM32_TIM10CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM11
    if (&ICUD11 == icup) {
      rccEnableTIM11(true);
      rccResetTIM11();
#if defined(STM32_TIM11CLK)
      icup->clock = STM32_TIM11CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM12
    if (&ICUD12 == icup) {
      rccEnableTIM12(true);
      rccResetTIM12();
#if defined(STM32_TIM12CLK)
      icup->clock = STM32_TIM12CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM13
    if (&ICUD13 == icup) {
      rccEnableTIM13(true);
      rccResetTIM13();
#if defined(STM32_TIM13CLK)
      icup->clock = STM32_TIM13CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM14
    if (&ICUD14 == icup) {
      rccEnableTIM14(true);
      rccResetTIM14();
#if defined(STM32_TIM14CLK)
      icup->clock = STM32_TIM14CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM15
    if (&ICUD15 == icup) {
      rccEnableTIM15(true);
      rccResetTIM15();
#if defined(STM32_TIM15CLK)
      icup->clock = STM32_TIM15CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM20
    if (&ICUD20 == icup) {
      rccEnableTIM20(true);
      rccResetTIM20();
#if defined(STM32_TIM20CLK)
      icup->clock = STM32_TIM20CLK;
#else
      icup->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_ICU_USE_TIM21
    if (&ICUD21 == icup) {
      rccEnableTIM21(true);
      rccResetTIM21();
#if defined(STM32_TIM21CLK)
      icup->clock = STM32_TIM21CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_ICU_USE_TIM22
    if (&ICUD22 == icup) {
      rccEnableTIM22(true);
      rccResetTIM22();
#if defined(STM32_TIM22CLK)
      icup->clock = STM32_TIM22CLK;
#else
      icup->clock = STM32_TIMCLK1;
#endif
    }
#endif
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    icup->tim->CR1    = 0;                  /* Timer disabled.              */
    icup->tim->CCR[0] = 0;                  /* Comparator 1 disabled.       */
    icup->tim->CCR[1] = 0;                  /* Comparator 2 disabled.       */
    icup->tim->CNT    = 0;                  /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  icup->tim->SR   = 0;                      /* Clear eventual pending IRQs. */
  icup->tim->DIER = icup->config->dier &    /* DMA-related DIER settings.   */
                    ~STM32_TIM_DIER_IRQ_MASK;
  psc = (icup->clock / icup->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFFFF) &&
                ((psc + 1) * icup->config->frequency) == icup->clock,
                "invalid frequency");
  icup->tim->PSC = psc;
  if (icup->config->arr == 0U) {
    /* Zero is an invalid value and is turned in maximum value, also for
       legacy configurations compatibility.*/
    icup->tim->ARR = 0xFFFFFFFFU;
  }
  else {
    icup->tim->ARR = icup->config->arr;
  }

  if (icup->config->channel == ICU_CHANNEL_1) {
    /* Selected input 1.
       CCMR1_CC1S = 01 = CH1 Input on TI1.
       CCMR1_CC2S = 10 = CH2 Input on TI1.*/
    icup->tim->CCMR1 = STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(2);

    /* SMCR_TS  = 101, input is TI1FP1.
       SMCR_SMS = 100, reset on rising edge.*/
    icup->tim->SMCR  = STM32_TIM_SMCR_TS(5) | STM32_TIM_SMCR_SMS(4);

    /* The CCER settings depend on the selected trigger mode.
       ICU_INPUT_ACTIVE_HIGH: Active on rising edge, idle on falling edge.
       ICU_INPUT_ACTIVE_LOW:  Active on falling edge, idle on rising edge.*/
    if (icup->config->mode == ICU_INPUT_ACTIVE_HIGH)
      icup->tim->CCER = STM32_TIM_CCER_CC1E |
                        STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2P;
    else
      icup->tim->CCER = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1P |
                        STM32_TIM_CCER_CC2E;

    /* Direct pointers to the capture registers in order to make reading
       data faster from within callbacks.*/
    icup->wccrp = &icup->tim->CCR[1];
    icup->pccrp = &icup->tim->CCR[0];
  }
  else {
    /* Selected input 2.
       CCMR1_CC1S = 10 = CH1 Input on TI2.
       CCMR1_CC2S = 01 = CH2 Input on TI2.*/
    icup->tim->CCMR1 = STM32_TIM_CCMR1_CC1S(2) | STM32_TIM_CCMR1_CC2S(1);

    /* SMCR_TS  = 110, input is TI2FP2.
       SMCR_SMS = 100, reset on rising edge.*/
    icup->tim->SMCR  = STM32_TIM_SMCR_TS(6) | STM32_TIM_SMCR_SMS(4);

    /* The CCER settings depend on the selected trigger mode.
       ICU_INPUT_ACTIVE_HIGH: Active on rising edge, idle on falling edge.
       ICU_INPUT_ACTIVE_LOW:  Active on falling edge, idle on rising edge.*/
    if (icup->config->mode == ICU_INPUT_ACTIVE_HIGH)
      icup->tim->CCER = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1P |
                        STM32_TIM_CCER_CC2E;
    else
      icup->tim->CCER = STM32_TIM_CCER_CC1E |
                        STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2P;

    /* Direct pointers to the capture registers in order to make reading
       data faster from within callbacks.*/
    icup->wccrp = &icup->tim->CCR[0];
    icup->pccrp = &icup->tim->CCR[1];
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
    /* Clock deactivation.*/
    icup->tim->CR1  = 0;                    /* Timer disabled.              */
    icup->tim->DIER = 0;                    /* All IRQs disabled.           */
    icup->tim->SR   = 0;                    /* Clear eventual pending IRQs. */

#if STM32_ICU_USE_TIM1
    if (&ICUD1 == icup) {
#if !defined(STM32_TIM1_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM1_UP_NUMBER);
      nvicDisableVector(STM32_TIM1_CC_NUMBER);
#endif
      rccDisableTIM1();
    }
#endif

#if STM32_ICU_USE_TIM2
    if (&ICUD2 == icup) {
#if !defined(STM32_TIM2_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM2_NUMBER);
#endif
      rccDisableTIM2();
    }
#endif

#if STM32_ICU_USE_TIM3
    if (&ICUD3 == icup) {
#if !defined(STM32_TIM3_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM3_NUMBER);
#endif
      rccDisableTIM3();
    }
#endif

#if STM32_ICU_USE_TIM4
    if (&ICUD4 == icup) {
#if !defined(STM32_TIM4_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM4_NUMBER);
#endif
      rccDisableTIM4();
    }
#endif

#if STM32_ICU_USE_TIM5
    if (&ICUD5 == icup) {
#if !defined(STM32_TIM5_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM5_NUMBER);
#endif
      rccDisableTIM5();
    }
#endif

#if STM32_ICU_USE_TIM8
    if (&ICUD8 == icup) {
#if !defined(STM32_TIM8_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM8_UP_NUMBER);
      nvicDisableVector(STM32_TIM8_CC_NUMBER);
#endif
      rccDisableTIM8();
    }
#endif

#if STM32_ICU_USE_TIM9
    if (&ICUD9 == icup) {
      rccDisableTIM9();
    }
#endif

#if STM32_ICU_USE_TIM10
    if (&ICUD10 == icup) {
      rccDisableTIM10();
    }
#endif

#if STM32_ICU_USE_TIM11
    if (&ICUD11 == icup) {
      rccDisableTIM11();
    }
#endif

#if STM32_ICU_USE_TIM12
    if (&ICUD12 == icup) {
      rccDisableTIM12();
    }
#endif

#if STM32_ICU_USE_TIM13
    if (&ICUD13 == icup) {
      rccDisableTIM13();
    }
#endif

#if STM32_ICU_USE_TIM14
    if (&ICUD14 == icup) {
      rccDisableTIM14();
    }
#endif

#if STM32_ICU_USE_TIM15
    if (&ICUD15 == icup) {
      rccDisableTIM15();
    }
#endif

#if STM32_ICU_USE_TIM20
    if (&ICUD20 == icup) {
      rccDisableTIM20();
    }
#endif

#if STM32_ICU_USE_TIM21
    if (&ICUD21 == icup) {
      rccDisableTIM21();
    }
#endif

#if STM32_ICU_USE_TIM22
    if (&ICUD22 == icup) {
      rccDisableTIM22();
    }
#endif
  }
}

/**
 * @brief   Starts the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_start_capture(ICUDriver *icup) {

  /* Triggering an UG and clearing the IRQ status.*/
  icup->tim->EGR |= STM32_TIM_EGR_UG;
  icup->tim->SR = 0;

  /* Timer is started.*/
  icup->tim->CR1 = STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}

/**
 * @brief   Waits for a completed capture.
 * @note    The operation is performed in polled mode.
 * @note    In order to use this function notifications must be disabled.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The capture status.
 * @retval false        if the capture is successful.
 * @retval true         if a timer overflow occurred.
 *
 * @notapi
 */
bool icu_lld_wait_capture(ICUDriver *icup) {

  /* If the driver is still in the ICU_WAITING state then we need to wait
     for the first activation edge.*/
  if (icup->state == ICU_WAITING)
    if (icu_lld_wait_edge(icup))
      return true;

  /* This edge marks the availability of a capture result.*/
  return icu_lld_wait_edge(icup);
}

/**
 * @brief   Stops the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_stop_capture(ICUDriver *icup) {

  /* Timer stopped.*/
  icup->tim->CR1   = 0;

  /* All interrupts disabled.*/
  icup->tim->DIER &= ~STM32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Enables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_enable_notifications(ICUDriver *icup) {
  uint32_t dier = icup->tim->DIER;

  /* If interrupts were already enabled then the operation is skipped.
     This is done in order to avoid clearing the SR and risk losing
     pending interrupts.*/
  if ((dier & STM32_TIM_DIER_IRQ_MASK) == 0) {
    /* Previously triggered IRQs are ignored, status cleared.*/
    icup->tim->SR = 0;

    if (icup->config->channel == ICU_CHANNEL_1) {
      /* Enabling periodic callback on CC1.*/
      dier |= STM32_TIM_DIER_CC1IE;

      /* Optionally enabling width callback on CC2.*/
      if (icup->config->width_cb != NULL)
        dier |= STM32_TIM_DIER_CC2IE;
    }
    else {
      /* Enabling periodic callback on CC2.*/
      dier |= STM32_TIM_DIER_CC2IE;

      /* Optionally enabling width callback on CC1.*/
      if (icup->config->width_cb != NULL)
        dier |= STM32_TIM_DIER_CC1IE;
    }

    /* If an overflow callback is defined then also the overflow callback
       is enabled.*/
    if (icup->config->overflow_cb != NULL)
      dier |= STM32_TIM_DIER_UIE;

    /* One single atomic write.*/
    icup->tim->DIER = dier;
  }
}

/**
 * @brief   Disables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_disable_notifications(ICUDriver *icup) {

  /* All interrupts disabled.*/
  icup->tim->DIER &= ~STM32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_serve_interrupt(ICUDriver *icup) {
  uint32_t sr;

  sr  = icup->tim->SR;
  sr &= icup->tim->DIER & STM32_TIM_DIER_IRQ_MASK;
  icup->tim->SR = ~sr;
  if (icup->config->channel == ICU_CHANNEL_1) {
    if ((sr & STM32_TIM_SR_CC2IF) != 0)
      _icu_isr_invoke_width_cb(icup);
    if ((sr & STM32_TIM_SR_CC1IF) != 0)
      _icu_isr_invoke_period_cb(icup);
  }
  else {
    if ((sr & STM32_TIM_SR_CC1IF) != 0)
      _icu_isr_invoke_width_cb(icup);
    if ((sr & STM32_TIM_SR_CC2IF) != 0)
      _icu_isr_invoke_period_cb(icup);
  }
  if ((sr & STM32_TIM_SR_UIF) != 0)
    _icu_isr_invoke_overflow_cb(icup);
}

#endif /* HAL_USE_ICU */

/** @} */
