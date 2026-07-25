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
 * @file    TIMv1/hal_gpt_lld.c
 * @brief   STM32 GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 * @note    The driver GPTD1 allocates the complex timer TIM1 when enabled.
 */
#if STM32_GPT_USE_TIM1 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif

/**
 * @brief   GPTD2 driver identifier.
 * @note    The driver GPTD2 allocates the timer TIM2 when enabled.
 */
#if STM32_GPT_USE_TIM2 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif

/**
 * @brief   GPTD3 driver identifier.
 * @note    The driver GPTD3 allocates the timer TIM3 when enabled.
 */
#if STM32_GPT_USE_TIM3 || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif

/**
 * @brief   GPTD4 driver identifier.
 * @note    The driver GPTD4 allocates the timer TIM4 when enabled.
 */
#if STM32_GPT_USE_TIM4 || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif

/**
 * @brief   GPTD5 driver identifier.
 * @note    The driver GPTD5 allocates the timer TIM5 when enabled.
 */
#if STM32_GPT_USE_TIM5 || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif

/**
 * @brief   GPTD6 driver identifier.
 * @note    The driver GPTD6 allocates the timer TIM6 when enabled.
 */
#if STM32_GPT_USE_TIM6 || defined(__DOXYGEN__)
GPTDriver GPTD6;
#endif

/**
 * @brief   GPTD7 driver identifier.
 * @note    The driver GPTD7 allocates the timer TIM7 when enabled.
 */
#if STM32_GPT_USE_TIM7 || defined(__DOXYGEN__)
GPTDriver GPTD7;
#endif

/**
 * @brief   GPTD8 driver identifier.
 * @note    The driver GPTD8 allocates the timer TIM8 when enabled.
 */
#if STM32_GPT_USE_TIM8 || defined(__DOXYGEN__)
GPTDriver GPTD8;
#endif

/**
 * @brief   GPTD9 driver identifier.
 * @note    The driver GPTD9 allocates the timer TIM9 when enabled.
 */
#if STM32_GPT_USE_TIM9 || defined(__DOXYGEN__)
GPTDriver GPTD9;
#endif

/**
 * @brief   GPTD10 driver identifier.
 * @note    The driver GPTD10 allocates the timer TIM10 when enabled.
 */
#if STM32_GPT_USE_TIM10 || defined(__DOXYGEN__)
GPTDriver GPTD10;
#endif

/**
 * @brief   GPTD11 driver identifier.
 * @note    The driver GPTD11 allocates the timer TIM11 when enabled.
 */
#if STM32_GPT_USE_TIM11 || defined(__DOXYGEN__)
GPTDriver GPTD11;
#endif

/**
 * @brief   GPTD12 driver identifier.
 * @note    The driver GPTD12 allocates the timer TIM12 when enabled.
 */
#if STM32_GPT_USE_TIM12 || defined(__DOXYGEN__)
GPTDriver GPTD12;
#endif

/**
 * @brief   GPTD13 driver identifier.
 * @note    The driver GPTD13 allocates the timer TIM13 when enabled.
 */
#if STM32_GPT_USE_TIM13 || defined(__DOXYGEN__)
GPTDriver GPTD13;
#endif

/**
 * @brief   GPTD14 driver identifier.
 * @note    The driver GPTD14 allocates the timer TIM14 when enabled.
 */
#if STM32_GPT_USE_TIM14 || defined(__DOXYGEN__)
GPTDriver GPTD14;
#endif

/**
 * @brief   GPTD15 driver identifier.
 * @note    The driver GPTD15 allocates the timer TIM15 when enabled.
 */
#if STM32_GPT_USE_TIM15 || defined(__DOXYGEN__)
GPTDriver GPTD15;
#endif

/**
 * @brief   GPTD16 driver identifier.
 * @note    The driver GPTD16 allocates the timer TIM16 when enabled.
 */
#if STM32_GPT_USE_TIM16 || defined(__DOXYGEN__)
GPTDriver GPTD16;
#endif

/**
 * @brief   GPTD17 driver identifier.
 * @note    The driver GPTD17 allocates the timer TIM17 when enabled.
 */
#if STM32_GPT_USE_TIM17 || defined(__DOXYGEN__)
GPTDriver GPTD17;
#endif

/**
 * @brief   GPTD20 driver identifier.
 * @note    The driver GPTD20 allocates the timer TIM20 when enabled.
 */
#if STM32_GPT_USE_TIM20 || defined(__DOXYGEN__)
GPTDriver GPTD20;
#endif

/**
 * @brief   GPTD21 driver identifier.
 * @note    The driver GPTD21 allocates the timer TIM21 when enabled.
 */
#if STM32_GPT_USE_TIM21 || defined(__DOXYGEN__)
GPTDriver GPTD21;
#endif

/**
 * @brief   GPTD22 driver identifier.
 * @note    The driver GPTD22 allocates the timer TIM22 when enabled.
 */
#if STM32_GPT_USE_TIM22 || defined(__DOXYGEN__)
GPTDriver GPTD22;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_GPT_USE_TIM1 || defined(__DOXYGEN__)
#if !defined(STM32_TIM1_SUPPRESS_ISR)
#if !defined(STM32_TIM1_UP_HANDLER)
#error "STM32_TIM1_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM1_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM1 */

#if STM32_GPT_USE_TIM2 || defined(__DOXYGEN__)
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

  gpt_lld_serve_interrupt(&GPTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM2_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM2 */

#if STM32_GPT_USE_TIM3 || defined(__DOXYGEN__)
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

  gpt_lld_serve_interrupt(&GPTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM3_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM3 */

#if STM32_GPT_USE_TIM4 || defined(__DOXYGEN__)
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

  gpt_lld_serve_interrupt(&GPTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM4_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM4 */

#if STM32_GPT_USE_TIM5 || defined(__DOXYGEN__)
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

  gpt_lld_serve_interrupt(&GPTD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM5_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM5 */

#if STM32_GPT_USE_TIM6 || defined(__DOXYGEN__)
#if !defined(STM32_TIM6_SUPPRESS_ISR)
#if !defined(STM32_TIM6_HANDLER)
#error "STM32_TIM6_HANDLER not defined"
#endif
/**
 * @brief   TIM6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD6);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM6_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM6 */

#if STM32_GPT_USE_TIM7 || defined(__DOXYGEN__)
#if !defined(STM32_TIM7_SUPPRESS_ISR)
#if !defined(STM32_TIM7_HANDLER)
#error "STM32_TIM7_HANDLER not defined"
#endif
/**
 * @brief   TIM7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD7);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM7_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM7 */

#if STM32_GPT_USE_TIM8 || defined(__DOXYGEN__)
#if !defined(STM32_TIM8_SUPPRESS_ISR)
#if !defined(STM32_TIM8_UP_HANDLER)
#error "STM32_TIM8_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM8 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD8);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM8_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM8 */

#if STM32_GPT_USE_TIM9 || defined(__DOXYGEN__)
#if !defined(STM32_TIM9_SUPPRESS_ISR)
#error "TIM9 ISR not defined by platform"
#endif /* !defined(STM32_TIM9_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM9 */

#if STM32_GPT_USE_TIM10 || defined(__DOXYGEN__)
#if !defined(STM32_TIM10_SUPPRESS_ISR)
#error "TIM10 ISR not defined by platform"
#endif /* !defined(STM32_TIM10_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM10 */

#if STM32_GPT_USE_TIM11 || defined(__DOXYGEN__)
#if !defined(STM32_TIM11_SUPPRESS_ISR)
#error "TIM11 ISR not defined by platform"
#endif /* !defined(STM32_TIM11_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM11 */

#if STM32_GPT_USE_TIM12 || defined(__DOXYGEN__)
#if !defined(STM32_TIM12_SUPPRESS_ISR)
#error "TIM12 ISR not defined by platform"
#endif /* !defined(STM32_TIM12_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM12 */

#if STM32_GPT_USE_TIM13 || defined(__DOXYGEN__)
#if !defined(STM32_TIM13_SUPPRESS_ISR)
#error "TIM13 ISR not defined by platform"
#endif /* !defined(STM32_TIM13_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM13 */

#if STM32_GPT_USE_TIM14 || defined(__DOXYGEN__)
#if !defined(STM32_TIM14_SUPPRESS_ISR)
#error "TIM14 ISR not defined by platform"
#endif /* !defined(STM32_TIM14_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM14 */

#if STM32_GPT_USE_TIM15 || defined(__DOXYGEN__)
#if !defined(STM32_TIM15_SUPPRESS_ISR)
#error "TIM15 ISR not defined by platform"
#endif /* !defined(STM32_TIM15_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM15 */

#if STM32_GPT_USE_TIM16 || defined(__DOXYGEN__)
#if !defined(STM32_TIM16_SUPPRESS_ISR)
#error "TIM16 ISR not defined by platform"
#endif /* !defined(STM32_TIM16_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM16 */

#if STM32_GPT_USE_TIM17 || defined(__DOXYGEN__)
#if !defined(STM32_TIM17_SUPPRESS_ISR)
#error "TIM17 ISR not defined by platform"
#endif /* !defined(STM32_TIM17_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM17 */

#if STM32_GPT_USE_TIM20 || defined(__DOXYGEN__)
#if !defined(STM32_TIM20_SUPPRESS_ISR)
#if !defined(STM32_TIM20_UP_HANDLER)
#error "STM32_TIM20_UP_HANDLER not defined"
#endif /* !defined(STM32_TIM20_SUPPRESS_ISR) */
/**
 * @brief   TIM20 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM20_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD20);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* STM32_GPT_USE_TIM20 */

#if STM32_GPT_USE_TIM21 || defined(__DOXYGEN__)
#if !defined(STM32_TIM21_SUPPRESS_ISR)
#if !defined(STM32_TIM21_HANDLER)
#error "STM32_TIM21_HANDLER not defined"
#endif
/**
 * @brief   TIM21 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM21_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD21);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM21_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM21 */

#if STM32_GPT_USE_TIM22 || defined(__DOXYGEN__)
#if !defined(STM32_TIM22_SUPPRESS_ISR)
#if !defined(STM32_TIM22_HANDLER)
#error "STM32_TIM22_HANDLER not defined"
#endif
/**
 * @brief   TIM22 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM22_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD22);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_TIM22_SUPPRESS_ISR) */
#endif /* STM32_GPT_USE_TIM22 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if STM32_GPT_USE_TIM1
  /* Driver initialization.*/
  GPTD1.tim = STM32_TIM1;
  gptObjectInit(&GPTD1);
#endif

#if STM32_GPT_USE_TIM2
  /* Driver initialization.*/
  GPTD2.tim = STM32_TIM2;
  gptObjectInit(&GPTD2);
#endif

#if STM32_GPT_USE_TIM3
  /* Driver initialization.*/
  GPTD3.tim = STM32_TIM3;
  gptObjectInit(&GPTD3);
#endif

#if STM32_GPT_USE_TIM4
  /* Driver initialization.*/
  GPTD4.tim = STM32_TIM4;
  gptObjectInit(&GPTD4);
#endif

#if STM32_GPT_USE_TIM5
  /* Driver initialization.*/
  GPTD5.tim = STM32_TIM5;
  gptObjectInit(&GPTD5);
#endif

#if STM32_GPT_USE_TIM6
  /* Driver initialization.*/
  GPTD6.tim = STM32_TIM6;
  gptObjectInit(&GPTD6);
#endif

#if STM32_GPT_USE_TIM7
  /* Driver initialization.*/
  GPTD7.tim = STM32_TIM7;
  gptObjectInit(&GPTD7);
#endif

#if STM32_GPT_USE_TIM8
  /* Driver initialization.*/
  GPTD8.tim = STM32_TIM8;
  gptObjectInit(&GPTD8);
#endif

#if STM32_GPT_USE_TIM9
  /* Driver initialization.*/
  GPTD9.tim = STM32_TIM9;
  gptObjectInit(&GPTD9);
#endif

#if STM32_GPT_USE_TIM10
  /* Driver initialization.*/
  GPTD10.tim = STM32_TIM10;
  gptObjectInit(&GPTD10);
#endif

#if STM32_GPT_USE_TIM11
  /* Driver initialization.*/
  GPTD11.tim = STM32_TIM11;
  gptObjectInit(&GPTD11);
#endif

#if STM32_GPT_USE_TIM12
  /* Driver initialization.*/
  GPTD12.tim = STM32_TIM12;
  gptObjectInit(&GPTD12);
#endif

#if STM32_GPT_USE_TIM13
  /* Driver initialization.*/
  GPTD13.tim = STM32_TIM13;
  gptObjectInit(&GPTD13);
#endif

#if STM32_GPT_USE_TIM14
  /* Driver initialization.*/
  GPTD14.tim = STM32_TIM14;
  gptObjectInit(&GPTD14);
#endif

#if STM32_GPT_USE_TIM15
  /* Driver initialization.*/
  GPTD15.tim = STM32_TIM15;
  gptObjectInit(&GPTD15);
#endif

#if STM32_GPT_USE_TIM16
  /* Driver initialization.*/
  GPTD16.tim = STM32_TIM16;
  gptObjectInit(&GPTD16);
#endif

#if STM32_GPT_USE_TIM17
  /* Driver initialization.*/
  GPTD17.tim = STM32_TIM17;
  gptObjectInit(&GPTD17);
#endif

#if STM32_GPT_USE_TIM20
  /* Driver initialization.*/
  GPTD20.tim = STM32_TIM20;
  gptObjectInit(&GPTD20);
#endif

#if STM32_GPT_USE_TIM21
  /* Driver initialization.*/
  GPTD21.tim = STM32_TIM21;
  gptObjectInit(&GPTD21);
#endif

#if STM32_GPT_USE_TIM22
  /* Driver initialization.*/
  GPTD22.tim = STM32_TIM22;
  gptObjectInit(&GPTD22);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
  uint16_t psc;

  if (gptp->state == GPT_STOP) {
    /* Clock activation.*/
#if STM32_GPT_USE_TIM1
    if (&GPTD1 == gptp) {
      rccEnableTIM1(true);
      rccResetTIM1();
#if !defined(STM32_TIM1_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM1_UP_NUMBER, STM32_GPT_TIM1_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM1CLK)
      gptp->clock = STM32_TIM1CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM2
    if (&GPTD2 == gptp) {
      rccEnableTIM2(true);
      rccResetTIM2();
#if !defined(STM32_TIM2_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM2_NUMBER, STM32_GPT_TIM2_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM2CLK)
      gptp->clock = STM32_TIM2CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM3
    if (&GPTD3 == gptp) {
      rccEnableTIM3(true);
      rccResetTIM3();
#if !defined(STM32_TIM3_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM3_NUMBER, STM32_GPT_TIM3_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM3CLK)
      gptp->clock = STM32_TIM3CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM4
    if (&GPTD4 == gptp) {
      rccEnableTIM4(true);
      rccResetTIM4();
#if !defined(STM32_TIM4_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM4_NUMBER, STM32_GPT_TIM4_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM4CLK)
      gptp->clock = STM32_TIM4CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM5
    if (&GPTD5 == gptp) {
      rccEnableTIM5(true);
      rccResetTIM5();
#if !defined(STM32_TIM5_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM5_NUMBER, STM32_GPT_TIM5_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM5CLK)
      gptp->clock = STM32_TIM5CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM6
    if (&GPTD6 == gptp) {
      rccEnableTIM6(true);
      rccResetTIM6();
#if !defined(STM32_TIM6_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM6_NUMBER, STM32_GPT_TIM6_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM6CLK)
      gptp->clock = STM32_TIM6CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM7
    if (&GPTD7 == gptp) {
      rccEnableTIM7(true);
      rccResetTIM7();
#if !defined(STM32_TIM7_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM7_NUMBER, STM32_GPT_TIM7_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM7CLK)
      gptp->clock = STM32_TIM7CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM8
    if (&GPTD8 == gptp) {
      rccEnableTIM8(true);
      rccResetTIM8();
#if !defined(STM32_TIM8_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM8_UP_NUMBER, STM32_GPT_TIM8_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM8CLK)
      gptp->clock = STM32_TIM8CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM9
    if (&GPTD9 == gptp) {
      rccEnableTIM9(true);
      rccResetTIM9();
#if !defined(STM32_TIM9_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM9_NUMBER, STM32_GPT_TIM9_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM9CLK)
      gptp->clock = STM32_TIM9CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM10
    if (&GPTD10 == gptp) {
      rccEnableTIM10(true);
      rccResetTIM10();
#if !defined(STM32_TIM10_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM10_NUMBER, STM32_GPT_TIM10_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM10CLK)
      gptp->clock = STM32_TIM10CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM11
    if (&GPTD11 == gptp) {
      rccEnableTIM11(true);
      rccResetTIM11();
#if !defined(STM32_TIM11_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM11_NUMBER, STM32_GPT_TIM11_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM11CLK)
      gptp->clock = STM32_TIM11CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM12
    if (&GPTD12 == gptp) {
      rccEnableTIM12(true);
      rccResetTIM12();
#if !defined(STM32_TIM12_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM12_NUMBER, STM32_GPT_TIM12_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM12CLK)
      gptp->clock = STM32_TIM12CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM13
    if (&GPTD13 == gptp) {
      rccEnableTIM13(true);
      rccResetTIM13();
#if !defined(STM32_TIM13_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM13_NUMBER, STM32_GPT_TIM13_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM13CLK)
      gptp->clock = STM32_TIM13CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM14
    if (&GPTD14 == gptp) {
      rccEnableTIM14(true);
      rccResetTIM14();
#if !defined(STM32_TIM14_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM14_NUMBER, STM32_GPT_TIM14_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM14CLK)
      gptp->clock = STM32_TIM14CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM15
    if (&GPTD15 == gptp) {
      rccEnableTIM15(true);
      rccResetTIM15();
#if defined(STM32_TIM15CLK)
      gptp->clock = STM32_TIM15CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM16
    if (&GPTD16 == gptp) {
      rccEnableTIM16(true);
      rccResetTIM16();
#if defined(STM32_TIM16CLK)
      gptp->clock = STM32_TIM16CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM17
    if (&GPTD17 == gptp) {
      rccEnableTIM17(true);
      rccResetTIM17();
    }
#endif

#if STM32_GPT_USE_TIM20
    if (&GPTD20 == gptp) {
      rccEnableTIM20(true);
      rccResetTIM20();
#if !defined(STM32_TIM20_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM20_UP_NUMBER, STM32_GPT_TIM20_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM20CLK)
      gptp->clock = STM32_TIM20CLK;
#else
      gptp->clock = STM32_TIMCLK2;
#endif
    }
#endif

#if STM32_GPT_USE_TIM21
    if (&GPTD21 == gptp) {
      rccEnableTIM21(true);
      rccResetTIM21();
#if !defined(STM32_TIM21_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM21_NUMBER, STM32_GPT_TIM21_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM21CLK)
      gptp->clock = STM32_TIM21CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif

#if STM32_GPT_USE_TIM22
    if (&GPTD22 == gptp) {
      rccEnableTIM22(true);
      rccResetTIM22();
#if !defined(STM32_TIM22_SUPPRESS_ISR)
      nvicEnableVector(STM32_TIM22_NUMBER, STM32_GPT_TIM22_IRQ_PRIORITY);
#endif
#if defined(STM32_TIM22CLK)
      gptp->clock = STM32_TIM22CLK;
#else
      gptp->clock = STM32_TIMCLK1;
#endif
    }
#endif
  }

  /* Prescaler value calculation.*/
  psc = (uint16_t)((gptp->clock / gptp->config->frequency) - 1);
  osalDbgAssert(((uint32_t)(psc + 1) * gptp->config->frequency) == gptp->clock,
                "invalid frequency");

  /* Timer configuration.*/
  gptp->tim->CR1  = 0U;                         /* Initially stopped.       */
  gptp->tim->CR2  = gptp->config->cr2;
  gptp->tim->PSC  = psc;                        /* Prescaler value.         */
  gptp->tim->SR   = 0U;                         /* Clear pending IRQs.      */
  gptp->tim->DIER = gptp->config->dier &        /* DMA-related DIER bits.   */
                    ~STM32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  if (gptp->state == GPT_READY) {
    gptp->tim->CR1  = 0U;                       /* Timer disabled.          */
    gptp->tim->DIER = 0U;                       /* All IRQs disabled.       */
    gptp->tim->SR   = 0U;                       /* Clear pending IRQs.      */

#if STM32_GPT_USE_TIM1
    if (&GPTD1 == gptp) {
#if !defined(STM32_TIM1_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM1_UP_NUMBER);
#endif
      rccDisableTIM1();
    }
#endif

#if STM32_GPT_USE_TIM2
    if (&GPTD2 == gptp) {
#if !defined(STM32_TIM2_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM2_NUMBER);
#endif
      rccDisableTIM2();
    }
#endif

#if STM32_GPT_USE_TIM3
    if (&GPTD3 == gptp) {
#if !defined(STM32_TIM3_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM3_NUMBER);
#endif
      rccDisableTIM3();
    }
#endif

#if STM32_GPT_USE_TIM4
    if (&GPTD4 == gptp) {
#if !defined(STM32_TIM4_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM4_NUMBER);
#endif
      rccDisableTIM4();
    }
#endif

#if STM32_GPT_USE_TIM5
    if (&GPTD5 == gptp) {
#if !defined(STM32_TIM5_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM5_NUMBER);
#endif
      rccDisableTIM5();
    }
#endif

#if STM32_GPT_USE_TIM6
    if (&GPTD6 == gptp) {
#if !defined(STM32_TIM6_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM6_NUMBER);
#endif
      rccDisableTIM6();
    }
#endif

#if STM32_GPT_USE_TIM7
    if (&GPTD7 == gptp) {
#if !defined(STM32_TIM7_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM7_NUMBER);
#endif
      rccDisableTIM7();
    }
#endif

#if STM32_GPT_USE_TIM8
    if (&GPTD8 == gptp) {
#if !defined(STM32_TIM8_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM8_UP_NUMBER);
#endif
      rccDisableTIM8();
    }
#endif

#if STM32_GPT_USE_TIM9
    if (&GPTD9 == gptp) {
#if !defined(STM32_TIM9_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM9_NUMBER);
#endif
      rccDisableTIM9();
    }
#endif

#if STM32_GPT_USE_TIM10
    if (&GPTD10 == gptp) {
#if !defined(STM32_TIM10_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM10_NUMBER);
#endif
      rccDisableTIM10();
    }
#endif

#if STM32_GPT_USE_TIM11
    if (&GPTD11 == gptp) {
#if !defined(STM32_TIM11_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM11_NUMBER);
#endif
      rccDisableTIM11();
    }
#endif

#if STM32_GPT_USE_TIM12
    if (&GPTD12 == gptp) {
#if !defined(STM32_TIM12_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM12_NUMBER);
#endif
      rccDisableTIM12();
    }
#endif

#if STM32_GPT_USE_TIM13
    if (&GPTD13 == gptp) {
#if !defined(STM32_TIM13_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM13_NUMBER);
#endif
      rccDisableTIM13();
    }
#endif

#if STM32_GPT_USE_TIM14
    if (&GPTD14 == gptp) {
#if !defined(STM32_TIM14_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM14_NUMBER);
#endif
      rccDisableTIM14();
    }
#endif

#if STM32_GPT_USE_TIM15
    if (&GPTD15 == gptp) {
      rccDisableTIM15();
    }
#endif

#if STM32_GPT_USE_TIM16
    if (&GPTD16 == gptp) {
      rccDisableTIM16();
    }
#endif

#if STM32_GPT_USE_TIM17
    if (&GPTD17 == gptp) {
      rccDisableTIM17();
    }
#endif

#if STM32_GPT_USE_TIM20
    if (&GPTD20 == gptp) {
#if !defined(STM32_TIM20_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM20_NUMBER);
#endif
      rccDisableTIM20();
    }
#endif

#if STM32_GPT_USE_TIM21
    if (&GPTD21 == gptp) {
#if !defined(STM32_TIM21_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM21_NUMBER);
#endif
      rccDisableTIM21();
    }
#endif

#if STM32_GPT_USE_TIM22
    if (&GPTD22 == gptp) {
#if !defined(STM32_TIM22_SUPPRESS_ISR)
      nvicDisableVector(STM32_TIM22_NUMBER);
#endif
      rccDisableTIM22();
    }
#endif
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 * @note    Interval values 0 and 1 are invalid on this architecture.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgAssert(interval > (gptcnt_t)0, "invalid interval");

  gptp->tim->ARR = (uint32_t)(interval - 1U);   /* Time constant.           */
  gptp->tim->EGR = STM32_TIM_EGR_UG;            /* Update event.            */
  gptp->tim->CNT = 0U;                          /* Reset counter.           */

  /* NOTE: After generating the UG event it takes several clock cycles before
     SR bit 0 goes to 1. This is why the clearing of CNT has been inserted
     before the clearing of SR, to give it some time.*/
  gptp->tim->SR  = 0U;                          /* Clear pending IRQs.      */
  if (NULL != gptp->config->callback)
    gptp->tim->DIER |= STM32_TIM_DIER_UIE;      /* Update Event IRQ enabled.*/
  gptp->tim->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  gptp->tim->CR1 = 0U;                          /* Initially stopped.       */
  gptp->tim->SR  = 0U;                          /* Clear pending IRQs.      */

  /* All interrupts disabled.*/
  gptp->tim->DIER &= ~STM32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 * @note    Interval values 0 and 1 are invalid on this architecture.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  osalDbgAssert(interval > (gptcnt_t)0, "invalid interval");

  gptp->tim->CR1 = STM32_TIM_CR1_UDIS;          /* Immediate update.        */
  gptp->tim->ARR = (uint32_t)(interval - 1U);   /* Time constant.           */
  gptp->tim->EGR = STM32_TIM_EGR_UG;            /* Update event.            */
  gptp->tim->SR  = 0U;                          /* Clear pending IRQs.      */
  gptp->tim->CR1 = STM32_TIM_CR1_OPM | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
  while (!(gptp->tim->SR & STM32_TIM_SR_UIF))
    ;
  gptp->tim->SR = 0U;                           /* Clear pending IRQs.      */
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_serve_interrupt(GPTDriver *gptp) {
  uint32_t sr;

  sr  = gptp->tim->SR;
  sr &= gptp->tim->DIER & STM32_TIM_DIER_IRQ_MASK;
  gptp->tim->SR = ~sr;
  if ((sr & STM32_TIM_SR_UIF) != 0) {
    _gpt_isr_invoke_cb(gptp);
  }
}

#endif /* HAL_USE_GPT */

/** @} */
