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
 * @file    SPI/stm32_spi3.inc
 * @brief   Shared SPI3 handler.
 *
 * @addtogroup STM32_SPI3_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_SPI3)
#error "STM32_HAS_SPI3 not defined in registry"
#endif

#if STM32_HAS_SPI3

/* Checks on vector names and numbers definitions.*/
#if !defined(STM32_SPI3_NUMBER)
#error "STM32_SPI3_NUMBER not defined in stm32_isr.h"
#endif
#if !defined(STM32_SPI3_HANDLER)
#error "STM32_SPI3_HANDLER not defined in stm32_isr.h"
#endif

/* Checks for mcuconf.h correctness.*/
#if !defined(STM32_SPI_USE_SPI3)
#error "STM32_SPI_USE_SPI3 not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_SPI3_PRIORITY)
#error "STM32_IRQ_SPI3_PRIORITY not defined in mcuconf.h"
#endif

/* Priority settings checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_SPI3_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_SPI3_PRIORITY"
#endif

#endif /* STM32_HAS_SPI3 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void spi3_irq_init(void) {
#if defined(STM32_SPI3_IS_USED)
  nvicEnableVector(STM32_SPI3_NUMBER, STM32_IRQ_SPI3_PRIORITY);
#endif
}

static inline void spi3_irq_deinit(void) {
#if defined(STM32_SPI3_IS_USED)
  nvicDisableVector(STM32_SPI3_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_SPI3_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   SPI3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID3);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
