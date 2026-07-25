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
 * @file    I2Cv4/stm32_i2c2.inc
 * @brief   Shared I2C2 handler.
 *
 * @addtogroup STM32_I2C2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_HAS_I2C2)
#error "STM32_HAS_I2C2 not defined in registry"
#endif

#if !defined(STM32_HAS_I2C3)
#error "STM32_HAS_I2C3 not defined in registry"
#endif

#if STM32_HAS_I2C2 || STM32_HAS_I2C3

/* Checks on vector names and numbers definitions.*/
#if STM32_I2C_SINGLE_IRQ == TRUE
#if !defined(STM32_I2C2_3_GLOBAL_NUMBER)
#error "STM32_I2C2_3_GLOBAL_NUMBER not defined in stm32_isr.h"
#endif
#if !defined(STM32_I2C2_3_GLOBAL_HANDLER)
#error "STM32_I2C2_3_GLOBAL_HANDLER not defined in stm32_isr.h"
#endif

#else
#error "split I2C EV/ER vectors case not supported for shared IRQs"
#endif

/* Checks for mcuconf.h correctness.*/
#if !defined(STM32_IRQ_I2C2_3_PRIORITY)
#error "STM32_IRQ_I2C2_3_PRIORITY not defined in mcuconf.h"
#endif

#if !defined(STM32_I2C_USE_I2C2)
#error "STM32_I2C_USE_I2C2 not defined in mcuconf.h"
#endif

#if !defined(STM32_I2C_USE_I2C3)
#error "STM32_I2C_USE_I2C3 not defined in mcuconf.h"
#endif

/* Priority settings checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_I2C2_3_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_I2C2_3_PRIORITY"
#endif

#endif /* STM32_HAS_I2C2 || STM32_HAS_I2C3 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void i2c2_i2c3_irq_init(void) {
#if defined(STM32_I2C2_IS_USED) || defined(STM32_I2C3_IS_USED)
  nvicEnableVector(STM32_I2C2_3_GLOBAL_NUMBER, STM32_IRQ_I2C2_3_PRIORITY);
#endif
}

static inline void i2c2_i2c3_irq_deinit(void) {
#if defined(STM32_I2C2_IS_USED) || defined(STM32_I2C3_IS_USED)
  nvicDisableVector(STM32_I2C2_3_GLOBAL_NUMBER);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_I2C2_IS_USED) || defined(STM32_I2C3_IS_USED) || defined(__DOXYGEN__)
/**
 * @brief   I2C2, I2C3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_I2C2_3_GLOBAL_HANDLER) {

  OSAL_IRQ_PROLOGUE();

#if STM32_I2C_USE_I2C2
  i2c_lld_serve_global_interrupt(&I2CD2);
#endif

#if STM32_I2C_USE_I2C3
  i2c_lld_serve_global_interrupt(&I2CD3);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_I2C2_IS_USED) || defined(STM32_I2C3_IS_USED) */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
