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
 * @file    EXTIv1/stm32_exti16-35_38.inc
 * @brief   Shared EXTI16-35_38 handler.
 *
 * @addtogroup STM32_EXTI1635_38_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Priority settings checks.*/
#if !defined(STM32_IRQ_EXTI1635_38_PRIORITY)
#error "STM32_IRQ_EXTI1635_38_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_DISABLE_EXTI1635_38_HANDLER) &&                          \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI1635_38_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI1635_38_PRIORITY"
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

static inline void exti16_exti35_38_irq_init(void) {
#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI35_IS_USED) ||       \
    defined(STM32_EXTI36_IS_USED) || defined(STM32_EXTI37_IS_USED) ||       \
    defined(STM32_EXTI38_IS_USED)
  nvicEnableVector(STM32_EXTI1635_38_NUMBER, STM32_IRQ_EXTI1635_38_PRIORITY);
#endif
}

static inline void exti16_exti35_38_irq_deinit(void) {
#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI35_IS_USED) ||       \
    defined(STM32_EXTI36_IS_USED) || defined(STM32_EXTI37_IS_USED) ||       \
    defined(STM32_EXTI38_IS_USED)
  nvicDisableVector(STM32_EXTI1635_38_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_EXTI16_IS_USED) || defined(STM32_EXTI35_IS_USED) ||       \
    defined(STM32_EXTI36_IS_USED) || defined(STM32_EXTI37_IS_USED) ||       \
    defined(STM32_EXTI38_IS_USED) || defined(__DOXYGEN__)
#if !defined(STM32_DISABLE_EXTI1635_38_HANDLER)
/**
 * @brief   EXTI[16], EXTI[35], EXTI[38] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_EXTI1635_38_HANDLER) {
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

#if defined(STM32_EXTI35_IS_USED) || defined(STM32_EXTI36_IS_USED) ||       \
    defined(STM32_EXTI37_IS_USED) || defined(STM32_EXTI38_IS_USED)
  extiGetAndClearGroup2((1U << (35 - 32)) | (1U << (36 - 32)) |
                        (1U << (37 - 32)) | (1U << (38 - 32)), pr);

  /* Could be unused.*/
  (void)pr;

#if defined(STM32_EXTI35_ISR)
  STM32_EXTI35_ISR(pr, 35);
#endif
#if defined(STM32_EXTI36_ISR)
  STM32_EXTI36_ISR(pr, 36);
#endif
#if defined(STM32_EXTI37_ISR)
  STM32_EXTI37_ISR(pr, 37);
#endif
#if defined(STM32_EXTI38_ISR)
  STM32_EXTI38_ISR(pr, 38);
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
