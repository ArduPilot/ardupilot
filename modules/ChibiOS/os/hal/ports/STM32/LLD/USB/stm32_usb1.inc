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
 * @file    USB/stm32_usb1.inc
 * @brief   Shared USB1 handler.
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

#if STM32_HAS_USB1

/* Checks on vector names and numbers definitions.*/
#if !defined(STM32_USB1_NUMBER)
#error "STM32_USB1_NUMBER not defined in stm32_isr.h"
#endif
#if !defined(STM32_USB1_HANDLER)
#error "STM32_USB1_HANDLER not defined in stm32_isr.h"
#endif

/* Checks for mcuconf.h correctness.*/
#if !defined(STM32_USB_USE_USB1)
#error "STM32_USB_USE_USB1 not defined in mcuconf.h"
#endif

#if !defined(STM32_IRQ_USB1_PRIORITY)
#error "STM32_IRQ_USB1_PRIORITY not defined in mcuconf.h"
#endif

/* Priority settings checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_USB1_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_USB1_PRIORITY"
#endif

#endif /* STM32_HAS_USB1 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void usb1_irq_init(void) {
#if defined(STM32_USB1_IS_USED)
  nvicEnableVector(STM32_USB1_NUMBER, STM32_IRQ_USB1_PRIORITY);
#endif
}

static inline void usb1_irq_deinit(void) {
#if defined(STM32_USB1_IS_USED)
  nvicDisableVector(STM32_USB1_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_USB1_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   USB1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USB1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  usb_lld_serve_interrupt(&USBD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
