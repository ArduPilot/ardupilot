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
 * @file    common/ARMCMx/sau.h
 * @brief   Cortex-Mx SAU support macros and structures.
 *
 * @addtogroup COMMON_ARMCMx_SAU
 * @{
 */

#ifndef SAU_H
#define SAU_H

#if (defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)) ||           \
    defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name System vectors numbers
 * @{
 */
#define SAU_REGION_NOT_NSC          (0U << 1U)
#define SAU_REGION_NSC              (1U << 1U)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sauEnable(void);
  void sauDisable(void);
  void sauEnableRegion(uint32_t region, uint32_t start,
                       uint32_t end, uint32_t flags);
  void sauDisableRegion(uint32_t region);
#ifdef __cplusplus
}
#endif

#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

#endif /* SAU_H */

/** @} */
