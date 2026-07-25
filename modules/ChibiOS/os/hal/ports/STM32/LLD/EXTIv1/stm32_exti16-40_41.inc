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
 * @file    EXTIv1/stm32_exti16-40_41.inc
 * @brief   Shared EXTI16-40_41 handler.
 *
 * @addtogroup STM32_EXTI164041_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Priority settings checks.*/
#if !defined(STM32_IRQ_EXTI164041_PRIORITY)
#error "STM32_IRQ_EXTI164041_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_DISABLE_EXTI164041_HANDLER) &&                           \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI164041_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI164041_PRIORITY"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void exti16_exti40_exti41_irq_init(void) {
#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI40_IS_USED) ||       \
    defined(STM32_EXTI41_IS_USED)
  nvicEnableVector(STM32_EXTI164041_NUMBER, STM32_IRQ_EXTI164041_PRIORITY);
#endif
}

static inline void exti16_exti40_exti41_irq_deinit(void) {
#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI40_IS_USED) ||       \
    defined(STM32_EXTI41_IS_USED)
  nvicDisableVector(STM32_EXTI164041_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI40_IS_USED) ||       \
    defined(STM32_EXTI41_IS_USED) || defined(__DOXYGEN__)
#if !defined(STM32_DISABLE_EXTI164041_HANDLER)
/**
 * @brief   EXTI[16], EXTI[40], EXTI[41] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_EXTI164041_HANDLER) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

#if defined(STM32_EXTI16_IS_USED)
  extiGetAndClearGroup1(1U << 16, pr);

  /* Could be unused.*/
  (void)pr;

#if defined(STM32_EXTI16_ISR)
  STM32_EXTI16_ISR(pr, 16);
#endif
#endif

#if defined(STM32_EXTI40_IS_USED) || defined(STM32_EXTI41_IS_USED)
  extiGetAndClearGroup2((1U << (40 - 32)) | (1U << (41 - 32)), pr);

  /* Could be unused.*/
  (void)pr;

#if defined(STM32_EXTI40_ISR)
  STM32_EXTI40_ISR(pr, 40);
#endif
#if defined(STM32_EXTI41_ISR)
  STM32_EXTI41_ISR(pr, 41);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
