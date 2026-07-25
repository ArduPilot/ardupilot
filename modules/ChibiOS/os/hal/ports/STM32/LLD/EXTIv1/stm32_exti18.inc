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
 * @file    EXTIv1/stm32_exti18.inc
 * @brief   Shared EXTI18 handler.
 *
 * @addtogroup STM32_EXTI18_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Priority settings checks.*/
#if !defined(STM32_IRQ_EXTI18_PRIORITY)
#error "STM32_IRQ_EXTI18_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_DISABLE_EXTI18_HANDLER) &&                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI18_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI18_PRIORITY"
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

static inline void exti18_irq_init(void) {
#if defined(STM32_EXTI18_IS_USED)
  nvicEnableVector(STM32_EXTI18_NUMBER, STM32_IRQ_EXTI18_PRIORITY);
#endif
}

static inline void exti18_irq_deinit(void) {
#if defined(STM32_EXTI18_IS_USED)
  nvicDisableVector(STM32_EXTI18_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_EXTI18_IS_USED) || defined(__DOXYGEN__)
#if !defined(STM32_DISABLE_EXTI18_HANDLER)
/**
 * @brief   EXTI[18] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_EXTI18_HANDLER) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  extiGetAndClearGroup1(1U << 18, pr);

  /* Could be unused.*/
  (void)pr;

#if defined(STM32_EXTI18_ISR)
  STM32_EXTI18_ISR(pr, 18);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
