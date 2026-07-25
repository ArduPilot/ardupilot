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
 * @file    common/ARMCMx/sau.c
 * @brief   Cortex-Mx SAU support code.
 *
 * @addtogroup COMMON_ARMCMx_SAU
 * @{
 */

#include "hal.h"

#if (defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)) ||           \
    defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

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
 * @brief   Enables SAU.
 */
void sauEnable(void) {

  SAU->CTRL = 1U;
}

/**
 * @brief   Disables SAU.
 */
void sauDisable(void) {

  SAU->CTRL = 0U;
}

/**
 * @brief   Enables a SAU region.
 * @note    When SAU is enabled then the whole memory is marked as secure,
 *          using this function you can scale it down to "non-Secure Callable"
 *          or "non-Secure" but you have to consider also the IDAU security
 *          level, you cannot set a level less secure than the one specified
 *          in IDAU.
 *
 * @param[in] region    the region number
 * @param[in] start     the region start address (inclusive), must be a
 *                      multiple of 32
 * @param[in] end       the region end address (not inclusive), must be a
 *                      multiple of 32
 * @param[in] flags     regions mode, note, this is tricky, read carefully
 *                      the ARM documentation and the note above
 */
void sauEnableRegion(uint32_t region, uint32_t start,
                     uint32_t end, uint32_t flags) {

  osalDbgCheck(region < SAU->TYPE);
  osalDbgCheck((start & 0x1FU) == 0U);
  osalDbgCheck((end & 0x1FU) == 0U);

  SAU->RNR  = region;
  SAU->RBAR = start;
  SAU->RLAR = ((end - 1U) & 0xFFFFFFE0U) | (flags & SAU_REGION_NSC) | 1U;
}

/**
 * @brief   Disables a SAU region.
 *
 * @param[in] region    the region number
 */
void sauDisableRegion(uint32_t region) {

  osalDbgCheck(region < SAU->TYPE);

  SAU->RNR  = region;
  SAU->RLAR = 0U;
  SAU->RBAR = 0U;
}

#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

/** @} */
