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
 * @file    DAC/stm32_dac1.inc
 * @brief   Shared DAC2 handler.
 *
 * @addtogroup STM32_DAC2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_DAC2_CH1)
#error "STM32_HAS_DAC2_CH1 not defined in registry"
#endif

#if !defined(STM32_HAS_DAC2_CH2)
#error "STM32_HAS_DAC2_CH2 not defined in registry"
#endif

/**
 * @brief   DAC2 presence flag.
 */
#if STM32_HAS_DAC2_CH1 || STM32_HAS_DAC2_CH2
#define STM32_HAS_DAC2      TRUE
#endif

#if STM32_HAS_DAC2

/* Checks on vector names and numbers definitions.*/
#if !defined(STM32_DAC2_NUMBER)
#error "STM32_DAC2_NUMBER not defined in stm32_isr.h"
#endif

#if !defined(STM32_DAC2_HANDLER)
#error "STM32_DAC2_HANDLER not defined in stm32_isr.h"
#endif

/* Checks for mcuconf.h correctness.*/
#if STM32_HAS_DAC2_CH1
#if !defined(STM32_DAC_USE_DAC2_CH1)
#error "STM32_DAC_USE_DAC2_CH1 not defined in mcuconf.h"
#endif
#endif

#if STM32_HAS_DAC2_CH2
#if STM32_HAS_DAC2_CH2 && !defined(STM32_DAC_USE_DAC2_CH2)
#error "STM32_DAC_USE_DAC2_CH2 not defined in mcuconf.h"
#endif
#endif

#if !defined(STM32_IRQ_DAC2_PRIORITY)
#error "STM32_IRQ_DAC2_PRIORITY not defined in mcuconf.h"
#endif

/* Priority settings checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_DAC2_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_DAC2_PRIORITY"
#endif

#endif /* STM32_HAS_DAC2 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void dac1_irq_init(void) {
#if defined(STM32_DAC2_IS_USED)
  nvicEnableVector(STM32_DAC2_NUMBER, STM32_IRQ_DAC2_PRIORITY);
#endif
}

static inline void dac1_irq_deinit(void) {
#if defined(STM32_DAC2_IS_USED)
  nvicDisableVector(STM32_DAC2_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DAC2_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   DAC2 interrupt handler.
 * @note    Both channels are handled here.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DAC2_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = DAC2->SR;
  DAC2->SR = isr;

#if defined(STM32_DAC_DAC2_IRQ_HOOK)
  STM32_DAC_DAC2_IRQ_HOOK(isr);
#endif

#if STM32_DAC_USE_DAC2_CH1
  if ((isr & DAC_SR_DMAUDR1) != 0U) {
    dac_lld_serve_interrupt(&DACD3);
  }
#endif

#if !STM32_DAC_DUAL_MODE && STM32_DAC_USE_DAC2_CH2
  if ((isr & DAC_SR_DMAUDR2) != 0U) {
    dac_lld_serve_interrupt(&DACD4);
  }
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
