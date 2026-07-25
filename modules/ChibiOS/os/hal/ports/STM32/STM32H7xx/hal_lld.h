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
 * @file    STM32H7xx/hal_lld.h
 * @brief   STM32H7xx HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Sub-family identifier
 */
#if !defined(STM32H7XX) || defined(__DOXYGEN__)
#define STM32H7XX
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options (common)
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 * @note    All the clock tree constants are calculated but the initialization
 *          is not performed.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT                       FALSE
#endif

/**
 * @brief   Target code for this HAL configuration.
 * @note    Core 1 is the Cortex-M7, core 2 is the Cortex-M4.
 */
#if !defined(STM32_TARGET_CORE) || defined(__DOXYGEN__)
#define STM32_TARGET_CORE                   1
#endif

/**
 * @brief   Enables a no-cache RAM area using the MPU.
 */
#if !defined(STM32_NOCACHE_ENABLE) || defined(__DOXYGEN__)
#define STM32_NOCACHE_ENABLE                FALSE
#endif

/**
 * @brief   MPU region to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_MPU_REGION) || defined(__DOXYGEN__)
#define STM32_NOCACHE_MPU_REGION            MPU_REGION_6
#endif

/**
 * @brief   Base address to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_RBAR) || defined(__DOXYGEN__)
#define STM32_NOCACHE_RBAR                  0x24000000U
#endif

/**
 * @brief   Size to be used for no-cache RAM area.
 */
#if !defined(STM32_NOCACHE_RASR) || defined(__DOXYGEN__)
#define STM32_NOCACHE_RASR                  MPU_RASR_SIZE_16K
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"

#if (STM32_NOCACHE_RASR != MPU_RASR_SIZE_32)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_64)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_128)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_256)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_512)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_1K)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_2K)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_4K)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_8K)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_16K)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_32K)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_64K)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_128K) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_256K) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_512K) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_1M)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_2M)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_4M)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_8M)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_16M)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_32M)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_64M)  &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_128M) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_256M) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_512M) &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_1G)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_2G)   &&                           \
    (STM32_NOCACHE_RASR != MPU_RASR_SIZE_4G)
#error "invalid MPU RASR size value"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the frequency of a clock point in Hz.
 * @note    Static implementation.
 *
 * @param[in] clkpt     clock point to be returned
 * @return              The clock point frequency in Hz or zero if the
 *                      frequency is unknown.
 *
 * @notapi
 */
#define hal_lld_get_clock_point(clkpt) 0U

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if defined(STM32H723xx) || defined(STM32H733xx) ||                         \
    defined(STM32H725xx) || defined(STM32H735xx) ||                         \
    defined(STM32H730xx) || defined(__DOXYGEN__)
#include "hal_lld_type2.h"
#elif defined(STM32H7A3xx)  || defined(STM32H7B3xx)  ||                     \
      defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) ||                     \
      defined(STM32H7B0xx)
#include "hal_lld_type3.h"
#else
#include "hal_lld_type1.h"
#endif

/* Various helpers.*/
#include "stm32_isr.h"
#include "stm32_mdma.h"
#include "stm32_dma.h"
#include "stm32_bdma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
