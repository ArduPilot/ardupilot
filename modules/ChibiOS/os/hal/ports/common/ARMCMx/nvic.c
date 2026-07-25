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
 * @file    common/ARMCMx/nvic.c
 * @brief   Cortex-Mx NVIC support code.
 *
 * @addtogroup COMMON_ARMCMx_NVIC
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Fun part, lets change register names for no clear reason and, after that,
   let's avoid changing them on all platforms, consistency is not fun.*/
#if defined(__CORE_CM23_H_GENERIC) ||                                       \
    defined(__CORE_CM33_H_GENERIC) ||                                       \
    defined(__CORE_CM55_H_GENERIC)
#define __IPR           IPR
#define __ITNS          ITNS
#else
#define __IPR           IP
#endif

#if defined(__CORE_CM7_H_GENERIC) ||                                        \
    defined(__CORE_CM23_H_GENERIC) ||                                       \
    defined(__CORE_CM33_H_GENERIC) ||                                       \
    defined(__CORE_CM55_H_GENERIC)
#define __SHPR          SHPR
#else
#define __SHPR          SHP
#endif

#define __ICER          ICER
#define __ICPR          ICPR
#define __ISPR          ISPR
#define __ISER          ISER

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   NVIC clearing and initialization.
 */
void nvicInit(void) {
#if defined(__CORE_CM0_H_GENERIC) || defined(__CORE_CM0PLUS_H_GENERIC) ||   \
    defined(__CORE_CM23_H_GENERIC)
  uint32_t n = 0U;
#else
  uint32_t n = SCnSCB->ICTR;
#endif

  for (uint32_t i = 0U; i <= n; i++) {
    NVIC->__ICER[i] = 0xFFFFFFFFU;
    NVIC->__ICPR[i] = 0xFFFFFFFFU;
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
    NVIC->__ITNS[i] = 0xFFFFFFFFU;
#endif
  }
}

/**
 * @brief   Sets the priority of an interrupt handler and enables it.
 *
 * @param[in] n         the interrupt number
 * @param[in] prio      the interrupt priority
 */
void nvicEnableVector(uint32_t n, uint32_t prio) {

#if defined(__CORE_CM0_H_GENERIC) || defined(__CORE_CM0PLUS_H_GENERIC) ||   \
    defined(__CORE_CM23_H_GENERIC)
  NVIC->__IPR[_IP_IDX(n)] = (NVIC->__IPR[_IP_IDX(n)] & ~(0xFFU << _BIT_SHIFT(n))) |
                            (NVIC_PRIORITY_MASK(prio) << _BIT_SHIFT(n));
#else
  NVIC->__IPR[n] = NVIC_PRIORITY_MASK(prio);
#endif
  NVIC->__ICPR[n >> 5U] = 1U << (n & 0x1FU);
  NVIC->__ISER[n >> 5U] = 1U << (n & 0x1FU);
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
  /* If the IRQ is enabled from secure mode then it is marked as secure
     interrupt in ITNS.*/
  NVIC->__ITNS[n >> 5U] &= ~(1U << (n & 0x1FU));
#endif
}

/**
 * @brief   Disables an interrupt handler.
 *
 * @param[in] n         the interrupt number
 */
void nvicDisableVector(uint32_t n) {

  NVIC->__ICER[n >> 5U] = 1U << (n & 0x1FU);
  NVIC->__ICPR[n >> 5U] = 1U << (n & 0x1FU);
#if defined(__CORE_CM0_H_GENERIC) || defined(__CORE_CM0PLUS_H_GENERIC) ||   \
    defined(__CORE_CM23_H_GENERIC)
  NVIC->__IPR[_IP_IDX(n)] = NVIC->__IPR[_IP_IDX(n)] & ~(0xFFU << _BIT_SHIFT(n));
#else
  NVIC->__IPR[n] = 0U;
#endif
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
  /* Marked as not secure again.*/
  NVIC->__ITNS[n >> 5U] |= 1U << (n & 0x1FU);
#endif
}

/**
 * @brief   Changes the priority of a system handler.
 *
 * @param[in] handler   the system handler number
 * @param[in] prio      the system handler priority
 */
void nvicSetSystemHandlerPriority(uint32_t handler, uint32_t prio) {

  osalDbgCheck(handler < 12U);

#if defined(__CORE_CM0_H_GENERIC) || defined(__CORE_CM0PLUS_H_GENERIC) ||   \
    defined(__CORE_CM23_H_GENERIC)
  SCB->__SHPR[_SHP_IDX(handler)] = (SCB->__SHPR[_SHP_IDX(handler)] & ~(0xFFU << _BIT_SHIFT(handler))) |
                                   (NVIC_PRIORITY_MASK(prio) << _BIT_SHIFT(handler));
#else
  SCB->__SHPR[handler] = NVIC_PRIORITY_MASK(prio);
#endif
}

/**
 * @brief   Clears a pending interrupt source.
 *
 * @param[in] n         the interrupt number
 */
void nvicClearPending(uint32_t n) {

  NVIC->__ICPR[n >> 5] = 1 << (n & 0x1F);
}

/**
 * @brief   Sets as pending an interrupt source.
 *
 * @param[in] n         the interrupt number
 */
void nvicSetPending(uint32_t n) {

  NVIC->__ISPR[n >> 5] = 1 << (n & 0x1F);
}

/** @} */
