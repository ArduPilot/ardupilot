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
 * @file    USB/stm32_usb1_ucpd1_2.inc
 * @brief   Shared USB1, UCPD1, UCPD2 handler.
 *
 * @addtogroup STM32_USB1_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_USB1)
#error "STM32_HAS_USB1 not defined in registry"
#endif

#if !defined(STM32_HAS_UCPD1)
#error "STM32_HAS_UCPD1 not defined in registry"
#endif

#if !defined(STM32_HAS_UCPD2)
#error "STM32_HAS_UCPD2 not defined in registry"
#endif

#if STM32_HAS_USB1 || STM32_HAS_UCPD1 || STM32_HAS_UCPD2

/* Checks on vector names and numbers definitions.*/
#if !defined(STM32_USB1_UCPD1_2_NUMBER)
#error "STM32_USB1_UCPD1_2_NUMBER not defined in stm32_isr.h"
#endif

#if !defined(STM32_USB1_UCPD1_2_HANDLER)
#error "STM32_USB1_UCPD1_2_HANDLER not defined in stm32_isr.h"
#endif

/* Checks for mcuconf.h correctness.*/
#if !defined(STM32_IRQ_USB1_UCPD1_2_PRIORITY)
#error "STM32_IRQ_USB1_UCPD1_2_PRIORITY not defined in mcuconf.h"
#endif

/* Priority settings checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_USB1_UCPD1_2_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_USB1_UCPD1_2_PRIORITY"
#endif

#endif /* STM32_HAS_USB1 || STM32_HAS_UCPD1 || STM32_HAS_UCPD2 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void usb1_ucpd1_2_irq_init(void) {

#if defined(STM32_USB1_IS_USED) || defined(STM32_UCPD1_IS_USED) || defined(STM32_UCPD2_IS_USED)
  nvicEnableVector(STM32_USB1_UCPD1_2_NUMBER, STM32_IRQ_USB1_UCPD1_2_PRIORITY);
#endif
}

static inline void usb1_ucpd1_2_irq_deinit(void) {

#if defined(STM32_USB1_IS_USED) || defined(STM32_UCPD1_IS_USED) || defined(STM32_UCPD2_IS_USED)
  nvicDisableVector(STM32_USB1_UCPD1_2_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_USB1_IS_USED) || defined(STM32_UCPD1_IS_USED) ||          \
    defined(STM32_UCPD2_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   USB1, UCPD1, UCPD2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USB1_UCPD1_2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if defined(STM32_USB1_IS_USED)
  usb_lld_serve_interrupt(&USBD1);
#endif

#if defined(STM32_UCPD1_IS_USED)
  ucpd_lld_serve_interrupt(&UCPDD1);
#endif

#if defined(STM32_UCPD2_IS_USED)
  ucpd_lld_serve_interrupt(&UCPDD2);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
