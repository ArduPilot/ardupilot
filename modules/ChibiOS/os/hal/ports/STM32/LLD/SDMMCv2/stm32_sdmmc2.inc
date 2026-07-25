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
 * @file    SDMMCv2/stm32_sdmmc2.inc
 * @brief   Shared SDMMC2 handler.
 *
 * @addtogroup STM32_SDMMC2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_SDMMC2)
#error "STM32_HAS_SDMMC2 not defined in registry"
#endif

#if STM32_HAS_SDMMC2

/* Priority settings checks.*/
#if !defined(STM32_IRQ_SDMMC2_PRIORITY)
#error "STM32_IRQ_SDMMC2_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_SDMMC2_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_SDMMC2_PRIORITY"
#endif

#endif /* STM32_HAS_SDMMC2 */

/* Other checks.*/
#if (HAL_USE_SDC && STM32_SDC_USE_SDMMC2)
#define STM32_SDMMC2_IS_USED        TRUE
#else
#define STM32_SDMMC2_IS_USED        FALSE
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

static inline void sdmmc2_irq_init(void) {
#if STM32_SDMMC2_IS_USED
  nvicEnableVector(STM32_SDMMC2_NUMBER, STM32_IRQ_SDMMC2_PRIORITY);
#endif
}

static inline void sdmmc2_irq_deinit(void) {
#if STM32_SDMMC2_IS_USED
  nvicDisableVector(STM32_SDMMC2_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_SDMMC2_IS_USED || defined(__DOXYGEN__)
/**
 * @brief   SDMMC2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SDMMC2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_SDC
#if STM32_SDC_USE_SDMMC2
  sdc_lld_serve_interrupt(&SDCD2);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
