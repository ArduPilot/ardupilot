/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC560Dxx/hal_lld.h
 * @brief   SPC560Dxx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - SPC5_XOSC_CLK.
 *          - SPC5_OSC_BYPASS (optionally).
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "registers.h"
#include "spc5_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS FALSE

/**
 * @name    Platform identification
 * @{
 */
#define PLATFORM_NAME               "SPC560Dxx Car Body and Convenience"
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum XOSC clock frequency.
 */
#define SPC5_XOSC_CLK_MAX           16000000

/**
 * @brief   Minimum XOSC clock frequency.
 */
#define SPC5_XOSC_CLK_MIN           4000000

/**
 * @brief   Maximum FMPLLs input clock frequency.
 */
#define SPC5_FMPLLIN_MIN            4000000

/**
 * @brief   Maximum FMPLLs input clock frequency.
 */
#define SPC5_FMPLLIN_MAX            48000000

/**
 * @brief   Maximum FMPLLs VCO clock frequency.
 */
#define SPC5_FMPLLVCO_MAX           512000000

/**
 * @brief   Maximum FMPLLs VCO clock frequency.
 */
#define SPC5_FMPLLVCO_MIN           256000000

/**
 * @brief   Maximum FMPLL0 output clock frequency.
 */
#define SPC5_FMPLL0_CLK_MAX         48000000
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define SPC5_IRC_CLK                16000000    /**< Internal fast RC
                                                     oscillator.            */
#define SPC5_SIRC_CLK               128000      /**< Internal RC slow
                                                     oscillator.            */
/** @} */

/**
 * @name    FMPLL_CR register bits definitions
 * @{
 */
#define SPC5_FMPLL_ODF_DIV2         (0U << 24)
#define SPC5_FMPLL_ODF_DIV4         (1U << 24)
#define SPC5_FMPLL_ODF_DIV8         (2U << 24)
#define SPC5_FMPLL_ODF_DIV16        (3U << 24)
/** @} */

/**
 * @name    ME_GS register bits definitions
 * @{
 */
#define SPC5_ME_GS_SYSCLK_MASK      (15U << 0)
#define SPC5_ME_GS_SYSCLK_IRC       (0U << 0)
#define SPC5_ME_GS_SYSCLK_DIVIRC    (1U << 0)
#define SPC5_ME_GS_SYSCLK_XOSC      (2U << 0)
#define SPC5_ME_GS_SYSCLK_DIVXOSC   (3U << 0)
#define SPC5_ME_GS_SYSCLK_FMPLL0    (4U << 0)
/** @} */

/**
 * @name    ME_ME register bits definitions
 * @{
 */
#define SPC5_ME_ME_RESET            (1U << 0)
#define SPC5_ME_ME_TEST             (1U << 1)
#define SPC5_ME_ME_SAFE             (1U << 2)
#define SPC5_ME_ME_DRUN             (1U << 3)
#define SPC5_ME_ME_RUN0             (1U << 4)
#define SPC5_ME_ME_RUN1             (1U << 5)
#define SPC5_ME_ME_RUN2             (1U << 6)
#define SPC5_ME_ME_RUN3             (1U << 7)
#define SPC5_ME_ME_HALT0            (1U << 8)
#define SPC5_ME_ME_STOP0            (1U << 10)
#define SPC5_ME_ME_STANDBY0         (1U << 13)
/** @} */

/**
 * @name    ME_xxx_MC registers bits definitions
 * @{
 */
#define SPC5_ME_MC_SYSCLK_MASK      (15U << 0)
#define SPC5_ME_MC_SYSCLK(n)        ((n) << 0)
#define SPC5_ME_MC_SYSCLK_IRC       SPC5_ME_MC_SYSCLK(0)
#define SPC5_ME_MC_SYSCLK_DIVIRC    SPC5_ME_MC_SYSCLK(1)
#define SPC5_ME_MC_SYSCLK_XOSC      SPC5_ME_MC_SYSCLK(2)
#define SPC5_ME_MC_SYSCLK_DIVXOSC   SPC5_ME_MC_SYSCLK(3)
#define SPC5_ME_MC_SYSCLK_FMPLL0    SPC5_ME_MC_SYSCLK(4)
#define SPC5_ME_MC_SYSCLK_DISABLED  SPC5_ME_MC_SYSCLK(15)
#define SPC5_ME_MC_IRCON            (1U << 4)
#define SPC5_ME_MC_XOSC0ON          (1U << 5)
#define SPC5_ME_MC_PLL0ON           (1U << 6)
#define SPC5_ME_MC_CFLAON_MASK      (3U << 16)
#define SPC5_ME_MC_CFLAON(n)        ((n) << 16)
#define SPC5_ME_MC_CFLAON_PD        (1U << 16)
#define SPC5_ME_MC_CFLAON_LP        (2U << 16)
#define SPC5_ME_MC_CFLAON_NORMAL    (3U << 16)
#define SPC5_ME_MC_DFLAON_MASK      (3U << 18)
#define SPC5_ME_MC_DFLAON(n)        ((n) << 18)
#define SPC5_ME_MC_DFLAON_PD        (1U << 18)
#define SPC5_ME_MC_DFLAON_LP        (2U << 18)
#define SPC5_ME_MC_DFLAON_NORMAL    (3U << 18)
#define SPC5_ME_MC_MVRON            (1U << 20)
#define SPC5_ME_MC_PDO              (1U << 23)
/** @} */

/**
 * @name    ME_MCTL register bits definitions
 * @{
 */
#define SPC5_ME_MCTL_KEY            0x5AF0U
#define SPC5_ME_MCTL_KEY_INV        0xA50FU
#define SPC5_ME_MCTL_MODE_MASK      (15U << 28)
#define SPC5_ME_MCTL_MODE(n)        ((n) << 28)
/** @} */

/**
 * @name    ME_RUN_PCx registers bits definitions
 * @{
 */
#define SPC5_ME_RUN_PC_TEST         (1U << 1)
#define SPC5_ME_RUN_PC_SAFE         (1U << 2)
#define SPC5_ME_RUN_PC_DRUN         (1U << 3)
#define SPC5_ME_RUN_PC_RUN0         (1U << 4)
#define SPC5_ME_RUN_PC_RUN1         (1U << 5)
#define SPC5_ME_RUN_PC_RUN2         (1U << 6)
#define SPC5_ME_RUN_PC_RUN3         (1U << 7)
/** @} */

/**
 * @name    ME_LP_PCx registers bits definitions
 * @{
 */
#define SPC5_ME_LP_PC_HALT0         (1U << 8)
#define SPC5_ME_LP_PC_STOP0         (1U << 10)
#define SPC5_ME_LP_PC_STANDBY0      (1U << 13)
/** @} */

/**
 * @name    ME_PCTL registers bits definitions
 * @{
 */
#define SPC5_ME_PCTL_RUN_MASK       (7U << 0)
#define SPC5_ME_PCTL_RUN(n)         ((n) << 0)
#define SPC5_ME_PCTL_LP_MASK        (7U << 3)
#define SPC5_ME_PCTL_LP(n)          ((n) << 3)
#define SPC5_ME_PCTL_DBG            (1U << 6)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Disables the clocks initialization in the HAL.
 */
#if !defined(SPC5_NO_INIT) || defined(__DOXYGEN__)
#define SPC5_NO_INIT                        FALSE
#endif

/**
 * @brief   Disables the overclock checks.
 */
#if !defined(SPC5_ALLOW_OVERCLOCK) || defined(__DOXYGEN__)
#define SPC5_ALLOW_OVERCLOCK                FALSE
#endif

/**
 * @brief   Disables the watchdog on start.
 */
#if !defined(SPC5_DISABLE_WATCHDOG) || defined(__DOXYGEN__)
#define SPC5_DISABLE_WATCHDOG               TRUE
#endif

/**
 * @brief   FMPLL0 IDF divider value.
 * @note    The default value is calculated for XOSC=8MHz and PHI=48MHz.
 */
#if !defined(SPC5_FMPLL0_IDF_VALUE) || defined(__DOXYGEN__)
#define SPC5_FMPLL0_IDF_VALUE               1
#endif

/**
 * @brief   FMPLL0 NDIV divider value.
 * @note    The default value is calculated for XOSC=8MHz and PHI=48MHz.
 */
#if !defined(SPC5_FMPLL0_NDIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_FMPLL0_NDIV_VALUE              48
#endif

/**
 * @brief   FMPLL0 ODF divider value.
 * @note    The default value is calculated for XOSC=8MHz and PHI=48MHz.
 */
#if !defined(SPC5_FMPLL0_ODF) || defined(__DOXYGEN__)
#define SPC5_FMPLL0_ODF                     SPC5_FMPLL_ODF_DIV8
#endif

/**
 * @brief   XOSC divider value.
 * @note    The allowed range is 1...32.
 */
#if !defined(SPC5_XOSCDIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_XOSCDIV_VALUE                  1
#endif

/**
 * @brief   Fast IRC divider value.
 * @note    The allowed range is 1...32.
 */
#if !defined(SPC5_IRCDIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_IRCDIV_VALUE                   1
#endif

/**
 * @brief   Peripherals Set 1 clock divider value.
 * @note    Zero means disabled clock.
 */
#if !defined(SPC5_PERIPHERAL1_CLK_DIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_PERIPHERAL1_CLK_DIV_VALUE      2
#endif

/**
 * @brief   Peripherals Set 2 clock divider value.
 * @note    Zero means disabled clock.
 */
#if !defined(SPC5_PERIPHERAL2_CLK_DIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_PERIPHERAL2_CLK_DIV_VALUE      2
#endif

/**
 * @brief   Peripherals Set 3 clock divider value.
 * @note    Zero means disabled clock.
 */
#if !defined(SPC5_PERIPHERAL3_CLK_DIV_VALUE) || defined(__DOXYGEN__)
#define SPC5_PERIPHERAL3_CLK_DIV_VALUE      2
#endif

/**
 * @brief   Active run modes in ME_ME register.
 * @note    Modes RESET, SAFE, DRUN, and RUN0 modes are always enabled, there
 *          is no need to specify them.
 */
#if !defined(SPC5_ME_ME_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_ME_BITS                     (SPC5_ME_ME_RUN1 |              \
                                             SPC5_ME_ME_RUN2 |              \
                                             SPC5_ME_ME_RUN3 |              \
                                             SPC5_ME_ME_HALT0 |             \
                                             SPC5_ME_ME_STOP0 |             \
                                             SPC5_ME_ME_STANDBY0)
#endif

/**
 * @brief   TEST mode settings.
 */
#if !defined(SPC5_ME_TEST_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_TEST_MC_BITS                (SPC5_ME_MC_SYSCLK_IRC |        \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   SAFE mode settings.
 */
#if !defined(SPC5_ME_SAFE_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_SAFE_MC_BITS                (SPC5_ME_MC_PDO)
#endif

/**
 * @brief   DRUN mode settings.
 */
#if !defined(SPC5_ME_DRUN_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_DRUN_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   RUN0 mode settings.
 */
#if !defined(SPC5_ME_RUN0_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN0_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   RUN1 mode settings.
 */
#if !defined(SPC5_ME_RUN1_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN1_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   RUN2 mode settings.
 */
#if !defined(SPC5_ME_RUN2_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN2_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   RUN3 mode settings.
 */
#if !defined(SPC5_ME_RUN3_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN3_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   HALT0 mode settings.
 */
#if !defined(SPC5_ME_HALT0_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_HALT0_MC_BITS               (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   STOP0 mode settings.
 */
#if !defined(SPC5_ME_STOP0_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_STOP0_MC_BITS               (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   STANDBY0 mode settings.
 */
#if !defined(SPC5_ME_STANDBY0_MC_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_STANDBY0_MC_BITS            (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_CFLAON_NORMAL |     \
                                             SPC5_ME_MC_DFLAON_NORMAL |     \
                                             SPC5_ME_MC_MVRON)
#endif

/**
 * @brief   Peripheral mode 0 (run mode).
 * @note    Do not change this setting, it is expected to be the "never run"
 *          mode.
 */
#if !defined(SPC5_ME_RUN_PC0_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC0_BITS                0
#endif

/**
 * @brief   Peripheral mode 1 (run mode).
 * @note    Do not change this setting, it is expected to be the "always run"
 *          mode.
 */
#if !defined(SPC5_ME_RUN_PC1_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC1_BITS                (SPC5_ME_RUN_PC_TEST |          \
                                             SPC5_ME_RUN_PC_SAFE |          \
                                             SPC5_ME_RUN_PC_DRUN |          \
                                             SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 2 (run mode).
 * @note    Do not change this setting, it is expected to be the "only during
 *          normal run" mode.
 */
#if !defined(SPC5_ME_RUN_PC2_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC2_BITS                (SPC5_ME_RUN_PC_DRUN |          \
                                             SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 3 (run mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_RUN_PC3_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC3_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 4 (run mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_RUN_PC4_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC4_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 5 (run mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_RUN_PC5_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC5_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 6 (run mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_RUN_PC6_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC6_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 7 (run mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_RUN_PC7_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_RUN_PC7_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#endif

/**
 * @brief   Peripheral mode 0 (low power mode).
 * @note    Do not change this setting, it is expected to be the "never run"
 *          mode.
 */
#if !defined(SPC5_ME_LP_PC0_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC0_BITS                 0
#endif

/**
 * @brief   Peripheral mode 1 (low power mode).
 * @note    Do not change this setting, it is expected to be the "always run"
 *          mode.
 */
#if !defined(SPC5_ME_LP_PC1_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC1_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0 |          \
                                             SPC5_ME_LP_PC_STANDBY0)
#endif

/**
 * @brief   Peripheral mode 2 (low power mode).
 * @note    Do not change this setting, it is expected to be the "halt only"
 *          mode.
 */
#if !defined(SPC5_ME_LP_PC2_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC2_BITS                 (SPC5_ME_LP_PC_HALT0)
#endif

/**
 * @brief   Peripheral mode 3 (low power mode).
 * @note    Do not change this setting, it is expected to be the "stop only"
 *          mode.
 */
#if !defined(SPC5_ME_LP_PC3_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC3_BITS                 (SPC5_ME_LP_PC_STOP0)
#endif

/**
 * @brief   Peripheral mode 4 (low power mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_LP_PC4_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC4_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#endif

/**
 * @brief   Peripheral mode 5 (low power mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_LP_PC5_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC5_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#endif

/**
 * @brief   Peripheral mode 6 (low power mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_LP_PC6_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC6_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#endif

/**
 * @brief   Peripheral mode 7 (low power mode).
 * @note    Not defined, available to application-specific modes.
 */
#if !defined(SPC5_ME_LP_PC7_BITS) || defined(__DOXYGEN__)
#define SPC5_ME_LP_PC7_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#endif

/**
 * @brief   PIT channel 0 IRQ priority.
 * @note    This PIT channel is allocated permanently for system tick
 *          generation.
 */
#if !defined(SPC5_PIT0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_PIT0_IRQ_PRIORITY              4
#endif

/**
 * @brief   Clock initialization failure hook.
 * @note    The default is to stop the system and let the RTC restart it.
 * @note    The hook code must not return.
 */
#if !defined(SPC5_CLOCK_FAILURE_HOOK) || defined(__DOXYGEN__)
#define SPC5_CLOCK_FAILURE_HOOK()           osalSysHalt("clock failure")
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(SPC560Dxx_MCUCONF)
#error "Using a wrong mcuconf.h file, SPC560Dxx_MCUCONF not defined"
#endif

/* Check on the XOSC frequency.*/
#if (SPC5_XOSC_CLK < SPC5_XOSC_CLK_MIN) ||                                  \
    (SPC5_XOSC_CLK > SPC5_XOSC_CLK_MAX)
#error "invalid SPC5_XOSC_CLK value specified"
#endif

/* Check on the XOSC divider.*/
#if (SPC5_XOSCDIV_VALUE < 1) || (SPC5_XOSCDIV_VALUE > 32)
#error "invalid SPC5_XOSCDIV_VALUE value specified"
#endif

/* Check on the IRC divider.*/
#if (SPC5_IRCDIV_VALUE < 1) || (SPC5_IRCDIV_VALUE > 32)
#error "invalid SPC5_IRCDIV_VALUE value specified"
#endif

/* Check on SPC5_FMPLL0_IDF_VALUE.*/
#if (SPC5_FMPLL0_IDF_VALUE < 1) || (SPC5_FMPLL0_IDF_VALUE > 15)
#error "invalid SPC5_FMPLL0_IDF_VALUE value specified"
#endif

/* Check on SPC5_FMPLL0_NDIV_VALUE.*/
#if (SPC5_FMPLL0_NDIV_VALUE < 32) || (SPC5_FMPLL0_NDIV_VALUE > 96)
#error "invalid SPC5_FMPLL0_NDIV_VALUE value specified"
#endif

/* Check on SPC5_FMPLL0_ODF.*/
#if (SPC5_FMPLL0_ODF == SPC5_FMPLL_ODF_DIV2)
#define SPC5_FMPLL0_ODF_VALUE    2
#elif (SPC5_FMPLL0_ODF == SPC5_FMPLL_ODF_DIV4)
#define SPC5_FMPLL0_ODF_VALUE    4
#elif (SPC5_FMPLL0_ODF == SPC5_FMPLL_ODF_DIV8)
#define SPC5_FMPLL0_ODF_VALUE    8
#elif (SPC5_FMPLL0_ODF == SPC5_FMPLL_ODF_DIV16)
#define SPC5_FMPLL0_ODF_VALUE    16
#else
#error "invalid SPC5_FMPLL0_ODF value specified"
#endif

/**
 * @brief   SPC5_FMPLL0_VCO_CLK clock point.
 */
#define SPC5_FMPLL0_VCO_CLK                                                 \
  ((SPC5_XOSC_CLK / SPC5_FMPLL0_IDF_VALUE) * SPC5_FMPLL0_NDIV_VALUE)

/* Check on FMPLL0 VCO output.*/
#if (SPC5_FMPLL0_VCO_CLK < SPC5_FMPLLVCO_MIN) ||                            \
    (SPC5_FMPLL0_VCO_CLK > SPC5_FMPLLVCO_MAX)
#error "SPC5_FMPLL0_VCO_CLK outside acceptable range (SPC5_FMPLLVCO_MIN...SPC5_FMPLLVCO_MAX)"
#endif

/**
 * @brief   SPC5_FMPLL0_CLK clock point.
 */
#define SPC5_FMPLL0_CLK                                                     \
  (SPC5_FMPLL0_VCO_CLK / SPC5_FMPLL0_ODF_VALUE)

/* Check on SPC5_FMPLL0_CLK.*/
#if (SPC5_FMPLL0_CLK > SPC5_FMPLL0_CLK_MAX) && !SPC5_ALLOW_OVERCLOCK
#error "SPC5_FMPLL0_CLK outside acceptable range (0...SPC5_FMPLL0_CLK_MAX)"
#endif

/* Check on the peripherals set 1 clock divider settings.*/
#if SPC5_PERIPHERAL1_CLK_DIV_VALUE == 0
#define SPC5_CGM_SC_DC0         0
#elif (SPC5_PERIPHERAL1_CLK_DIV_VALUE >= 1) &&                              \
      (SPC5_PERIPHERAL1_CLK_DIV_VALUE <= 16)
#define SPC5_CGM_SC_DC0         (0x80 | (SPC5_PERIPHERAL1_CLK_DIV_VALUE - 1))
#else
#error "invalid SPC5_PERIPHERAL1_CLK_DIV_VALUE value specified"
#endif

/* Check on the peripherals set 2 clock divider settings.*/
#if SPC5_PERIPHERAL2_CLK_DIV_VALUE == 0
#define SPC5_CGM_SC_DC1         0
#elif (SPC5_PERIPHERAL2_CLK_DIV_VALUE >= 1) &&                              \
      (SPC5_PERIPHERAL2_CLK_DIV_VALUE <= 16)
#define SPC5_CGM_SC_DC1         (0x80 | (SPC5_PERIPHERAL2_CLK_DIV_VALUE - 1))
#else
#error "invalid SPC5_PERIPHERAL2_CLK_DIV_VALUE value specified"
#endif

/* Check on the peripherals set 3 clock divider settings.*/
#if SPC5_PERIPHERAL3_CLK_DIV_VALUE == 0
#define SPC5_CGM_SC_DC2         0
#elif (SPC5_PERIPHERAL3_CLK_DIV_VALUE >= 1) &&                              \
      (SPC5_PERIPHERAL3_CLK_DIV_VALUE <= 16)
#define SPC5_CGM_SC_DC2         (0x80 | (SPC5_PERIPHERAL3_CLK_DIV_VALUE - 1))
#else
#error "invalid SPC5_PERIPHERAL3_CLK_DIV_VALUE value specified"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum {
  SPC5_RUNMODE_TEST  = 1,
  SPC5_RUNMODE_SAFE  = 2,
  SPC5_RUNMODE_DRUN  = 3,
  SPC5_RUNMODE_RUN0  = 4,
  SPC5_RUNMODE_RUN1  = 5,
  SPC5_RUNMODE_RUN2  = 6,
  SPC5_RUNMODE_RUN3  = 7,
  SPC5_RUNMODE_HALT0 = 8,
  SPC5_RUNMODE_STOP0 = 10
} spc5_runmode_t;

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

#include "spc5_edma.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void spc_clock_init(void);
  bool halSPCSetRunMode(spc5_runmode_t mode);
  void halSPCSetPeripheralClockMode(uint32_t n, uint32_t pctl);
#if !SPC5_NO_INIT
  uint32_t halSPCGetSystemClock(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
