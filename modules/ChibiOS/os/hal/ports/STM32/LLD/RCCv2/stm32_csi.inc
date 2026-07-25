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
 * @file    RCCv2/stm32_csi.inc
 * @brief   Shared CSI clock handler.
 *
 * @addtogroup STM32_CSI_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   CSI clock frequency.
 */
#define STM32_CSICLK            4000000U

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_CSI)
#error "STM32_RCC_HAS_CSI not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_CSI_ENABLED)
#error "STM32_CSI_ENABLED not defined in mcuconf.h"
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

__STATIC_FORCEINLINE void csi_enable(void) {

  RCC->OCENSETR = RCC_OCENSETR_CSION;
  while ((RCC->OCRDYR & RCC_OCRDYR_CSIRDY) == 0U) {
    /* Waiting for CSI activation.*/
  }
}

__STATIC_FORCEINLINE void csi_disable(void) {

  RCC->OCENCLRR = RCC_OCENCLRR_CSION;
}

__STATIC_FORCEINLINE void csi_init(void) {

#if STM32_CSI_ENABLED
  /* HSI activation.*/
  csi_enable();
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
