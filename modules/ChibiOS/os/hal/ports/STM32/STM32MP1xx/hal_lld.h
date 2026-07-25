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
 * @file    STM32MP1xx/hal_lld.h
 * @brief   STM32MP1xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_HSECLK.
 *          .
 *          One of the following macros must also be defined:
 *          - STM32MP157Axx, STM32MP157Cxx, STM32MP157Dxx, STM32MP157Fxx,
 *          - STM32MP153Axx, STM32MP153Cxx, STM32MP153Dxx, STM32MP153Fxx,
 *          - STM32MP151Axx, STM32MP151Cxx, STM32MP151Dxx, STM32MP151Fxx,
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

#define STM32_HCLK 0

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32MP157Axx) || defined(STM32MP157Cxx) ||                     \
    defined(STM32MP157Dxx) || defined(STM32MP157Fxx) ||                     \
    defined(STM32MP153Axx) || defined(STM32MP153Cxx) ||                     \
    defined(STM32MP153Dxx) || defined(STM32MP153Fxx) ||                     \
    defined(STM32MP151Axx) || defined(STM32MP151Cxx) ||                     \
    defined(STM32MP151Dxx) || defined(STM32MP151Fxx) ||                     \
    defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32MP1 Microprocessor"

#else
#error "STM32MP1 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32MP1XX) || defined(__DOXYGEN__)
#define STM32MP1XX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#define CLK_MCU_CK              0U
#define CLK_HCLK1               CLK_MCU_CK
#define CLK_HCLK2               CLK_MCU_CK
#define CLK_HCLK3               CLK_MCU_CK
#define CLK_HCLK4               CLK_MCU_CK
#define CLK_PLL3P_CK            1U
#define CLK_PLL3Q_CK            2U
#define CLK_PLL3R_CK            3U
#define CLK_PLL4P_CK            4U
#define CLK_PLL4Q_CK            5U
#define CLK_PLL4R_CK            6U
#define CLK_PCLK1               7U
#define CLK_PCLK1TIM            8U
#define CLK_PCLK2               9U
#define CLK_PCLK2TIM            10U
#define CLK_PCLK3               11U
#define CLK_ARRAY_SIZE          12U
/** @} */

/**
 * @name    RCC_HSICFGR register bits definitions
 * @{
 */
#define STM32_HSIDIV_DIV1        (0 << 0)   /**< HSI is divided by 1.       */
#define STM32_HSIDIV_DIV2        (1 << 0)   /**< HSI is divided by 2.       */
#define STM32_HSIDIV_DIV4        (2 << 0)   /**< HSI is divided by 4.       */
#define STM32_HSIDIV_DIV8        (3 << 0)   /**< HSI is divided by 8.       */
/** @} */

/**
 * @name    RCC_RCK3SELR register bits definitions
 * @{
 */
#define STM32_PLL3SRC_HSI        (0 << 0)   /**< PLL3 clock source is HSI.  */
#define STM32_PLL3SRC_HSE        (1 << 0)   /**< PLL3 clock source is HSE.  */
#define STM32_PLL3SRC_CSI        (2 << 0)   /**< PLL3 clock source is CSI.  */
#define STM32_PLL3SRC_NOCLOCK    (3 << 0)   /**< PLL3 clock disabled.       */
/** @} */

/**
 * @name    RCC_RCK4SELR register bits definitions
 * @{
 */
#define STM32_PLL4SRC_HSI        (0 << 0)   /**< PLL4 clock source is HSI.  */
#define STM32_PLL4SRC_HSE        (1 << 0)   /**< PLL4 clock source is HSE.  */
#define STM32_PLL4SRC_CSI        (2 << 0)   /**< PLL4 clock source is CSI.  */
#define STM32_PLL4SRC_I2S_CKIN   (3 << 0)   /**< PLL4 clock source is I2SCK.*/
/** @} */

/**
 * @name    RCC_MSSCKSELR register bits definitions
 * @{
 */
#define STM32_MCUSSRC_HSI        (0 << 0)   /**< MCU clock source is HSI.  */
#define STM32_MCUSSRC_HSE        (1 << 0)   /**< MCU clock source is HSE.  */
#define STM32_MCUSSRC_CSI        (2 << 0)   /**< MCU clock source is CSI.  */
#define STM32_MCUSSRC_PLL3P      (3 << 0)   /**< MCU clock source is PLL3P.*/
/** @} */

/**
 * @name    RCC_CPERCKSELR register bits definitions
 * @{
 */
#define STM32_CKPERSRC_HSI       (0 << 0)   /**< PERSRC clock source is HSI.*/
#define STM32_CKPERSRC_CSI       (1 << 0)   /**< PERSRC clock source is HSE.*/
#define STM32_CKPERSRC_HSE       (2 << 0)   /**< PERSRC clock source is CSI.*/
#define STM32_CKPERSRC_NOCLOCK   (3 << 0)   /**< PERSRC clock disabled.     */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables initializations in HAL entirely.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT                       FALSE
#endif

/**
 * @brief   If enabled assumes TZEN active.
 */
#if !defined(STM32_TZEN_ENABLED) || defined(__DOXYGEN__)
#define STM32_TZEN_ENABLED                  FALSE
#endif

/**
 * @brief   If enabled assumes MCKPROT active.
 */
#if !defined(STM32_MCKPROT_ENABLED) || defined(__DOXYGEN__)
#define STM32_MCKPROT_ENABLED               FALSE
#endif

/**
 * @brief   Enables the dynamic clock handling.
 */
#if !defined(STM32_CLOCK_DYNAMIC) || defined(__DOXYGEN__)
#define STM32_CLOCK_DYNAMIC                 FALSE
#endif

/**
 * @brief   PWR MCUCR register initialization value.
 */
#if !defined(STM32_PWR_MCUCR) || defined(__DOXYGEN__)
#define STM32_PWR_MCUCR                     0
#endif

/**
 * @brief   PWR MCUWKUPENR register initialization value.
 */
#if !defined(STM32_PWR_MCUWKUPENR) || defined(__DOXYGEN__)
#define STM32_PWR_MCUWKUPENR                0
#endif

/**
 * @brief   Frequency of the external I2S clock or zero if unused.
 */
#if !defined(STM32_I2S_CKIN_VALUE) || defined(__DOXYGEN__)
#define STM32_I2S_CKIN_VALUE                0
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 * @note    This initialization is performed only if TZEN=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the CSI clock source.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_CSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_CSI_ENABLED                   TRUE
#endif

/**
 * @brief   Enables or disables the HSI clock source.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_HSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI_ENABLED                   TRUE
#endif

/**
 * @brief   HSI divider setting.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_HSIDIV) || defined(__DOXYGEN__)
#define STM32_HSIDIV                        STM32_HSIDIV_DIV1
#endif

/**
 * @brief   Clock source for the PLL3.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3SRC) || defined(__DOXYGEN__)
#define STM32_PLL3SRC                       STM32_PLL3SRC_HSE
#endif

/**
 * @brief   PLL3 M divider value.
 * @note    The allowed values are 1..64.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3DIVM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3DIVM_VALUE                3
#endif

/**
 * @brief   PLL3 N multiplier value.
 * @note    The allowed values are 25..200.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3DIVN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3DIVN_VALUE                50
#endif

/**
 * @brief   PLL3 N multiplier fractional value.
 * @note    The allowed values are 0..8191.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3FRACV_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3FRACV_VALUE               0
#endif

/**
 * @brief   PLL3 P divider value or zero if disabled.
 * @note    The allowed values are 1..128.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3DIVP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3DIVP_VALUE                2
#endif

/**
 * @brief   PLL3 Q divider value.
 * @note    The allowed values are 1..128.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3DIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3DIVQ_VALUE                4
#endif

/**
 * @brief   PLL3 R divider value.
 * @note    The allowed values are 1..128.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_PLL3DIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL3DIVR_VALUE                4
#endif

/**
 * @brief   Clock source for the PLL4.
 */
#if !defined(STM32_PLL4SRC) || defined(__DOXYGEN__)
#define STM32_PLL4SRC                       STM32_PLL4SRC_HSE
#endif

/**
 * @brief   PLL4 M divider value.
 * @note    The allowed values are 1..64.
 */
#if !defined(STM32_PLL4DIVM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL4DIVM_VALUE                2222222222
#endif

/**
 * @brief   PLL4 N multiplier value.
 * @note    The allowed values are 25..200.
 */
#if !defined(STM32_PLL4DIVN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL4DIVN_VALUE                2222222222
#endif

/**
 * @brief   PLL4 P divider value or zero if disabled.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL4DIVP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL4DIVP_VALUE                2222222222
#endif

/**
 * @brief   PLL4 Q divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL4DIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL4DIVQ_VALUE                2222222222
#endif

/**
 * @brief   PLL4 R divider value.
 * @note    The allowed values are 1..128.
 */
#if !defined(STM32_PLL4DIVR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLL4DIVR_VALUE                2222222222
#endif

/**
 * @brief   MCU divider setting.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_MCUDIV) || defined(__DOXYGEN__)
#define STM32_MCUDIV                        2222222222
#endif

/**
 * @brief   MCU main clock source selection.
 * @note    This initialization is performed only if TZEN=0 or MCKPROT=0
 *          otherwise the setting must match the initialization performed
 *          on the Cortex-A side.
 */
#if !defined(STM32_MCUSSRC) || defined(__DOXYGEN__)
#define STM32_MCUSSRC                       STM32_MCUSSRC_PLL3P
#endif

/**
 * @brief   APB1DIV prescaler setting.
 */
#if !defined(STM32_APB1DIV) || defined(__DOXYGEN__)
#define STM32_APB1DIV                       2222222222
#endif

/**
 * @brief   APB2DIV prescaler setting.
 */
#if !defined(STM32_APB2DIV) || defined(__DOXYGEN__)
#define STM32_APB2DIV                       2222222222
#endif

/**
 * @brief   APB3DIV prescaler setting.
 */
#if !defined(STM32_APB3DIV) || defined(__DOXYGEN__)
#define STM32_APB3DIV                       2222222222
#endif

/**
 * @brief   Clock source for peripherals.
 */
#if !defined(STM32_CKPERSRC) || defined(__DOXYGEN__)
#define STM32_CKPERSRC                      STM32_CKPERSRC_HSI
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Clock handling mode selection.*/
#if STM32_CLOCK_DYNAMIC == TRUE
#define HAL_LLD_USE_CLOCK_MANAGEMENT
#endif

/*
 * Configuration-related checks.
 */
#if !defined(STM32MP1xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP1xx_MCUCONF not defined"
#endif

#if defined(STM32MP157Axx) && !defined(STM32MP157A_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP157A_MCUCONF not defined"

#elif defined(STM32MP157Cxx) && !defined(STM32MP157C_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP157C_MCUCONF not defined"

#elif defined(STM32MP157Dxx) && !defined(STM32MP157D_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP157D_MCUCONF not defined"

#elif defined(STM32MP157Fxx) && !defined(STM32MP157F_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP157F_MCUCONF not defined"

#elif defined(STM32MP153Axx) && !defined(STM32MP153A_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP153A_MCUCONF not defined"

#elif defined(STM32MP153Cxx) && !defined(STM32MP153C_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP153C_MCUCONF not defined"

#elif defined(STM32MP153Dxx) && !defined(STM32MP153D_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP153D_MCUCONF not defined"

#elif defined(STM32MP153Fxx) && !defined(STM32MP153F_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP153F_MCUCONF not defined"

#elif defined(STM32MP151Axx) && !defined(STM32MP151A_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP151A_MCUCONF not defined"

#elif defined(STM32MP151Cxx) && !defined(STM32MP151C_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP151C_MCUCONF not defined"

#elif defined(STM32MP151Dxx) && !defined(STM32MP151D_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP151D_MCUCONF not defined"

#elif defined(STM32MP151Fxx) && !defined(STM32MP151F_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32MP151F_MCUCONF not defined"

#endif

/**
 * @name    System Limits
 * @{
 */
#define STM32_MCUSS_CK_MAX              209000000

#define STM32_HSECLK_MAX                48000000
#define STM32_HSECLK_MIN                8000000
#define STM32_HSECLK_BYP_MAX            48000000
#define STM32_HSECLK_BYP_MIN            8000000

#define STM32_PLL3REFCLK_MAX            16000000
#define STM32_PLL3REFCLK_MIN            4000000
#define STM32_PLL3REFCLK_SD_MIN         8000000
#define STM32_PLL3VCOCLK_MAX            800000000
#define STM32_PLL3VCOCLK_MIN            400000000
#define STM32_PLL3PCLK_MAX              800000000
#define STM32_PLL3PCLK_MIN              3125000
#define STM32_PLL3QCLK_MAX              800000000
#define STM32_PLL3QCLK_MIN              3125000
#define STM32_PLL3RCLK_MAX              800000000
#define STM32_PLL3RCLK_MIN              3125000
#define STM32_PLL3DIVM_MAX              64
#define STM32_PLL3DIVM_MIN              1
#define STM32_PLL3DIVN_MAX              200
#define STM32_PLL3DIVN_MIN              25
#define STM32_PLL3DIVP_MAX              128
#define STM32_PLL3DIVP_MIN              1
#define STM32_PLL3DIVQ_MAX              128
#define STM32_PLL3DIVQ_MIN              1
#define STM32_PLL3DIVR_MAX              128
#define STM32_PLL3DIVR_MIN              1
#define STM32_PCLK1_MAX                 104500000
#define STM32_PCLK2_MAX                 104500000
#define STM32_PCLK3_MAX                 104500000
#define STM32_ADCCLK_BOOST_MAX          36000000
#define STM32_ADCCLK_NOBOOST_MAX        20000000
/** @} */

/* External oscillator settings check.*/
#if !defined(STM32_LSECLK)
#error "STM32_LSECLK not defined in board.h"
#endif

#if !defined(STM32_HSECLK)
#error "STM32_HSECLK not defined in board.h"
#endif

/* Clock handlers.*/
#include "stm32_csi.inc"
#include "stm32_hsi.inc"
#include "stm32_lsi.inc"
#include "stm32_hse.inc"

/*
 * CSI related checks.
 */
#if STM32_CSI_ENABLED
#else /* !STM32_CSI_ENABLED */

  #if STM32_PLL3SRC == STM32_PLL3SRC_CSI
    #error "CSI not enabled, required by STM32_PLL3SRC"
  #endif

  #if STM32_PLL4SRC == STM32_PLL4SRC_CSI
    #error "CSI not enabled, required by STM32_PLL4SRC"
  #endif

  #if STM32_CKPERSRC == STM32_CKPERSRC_CSI
    #error "CSI not enabled, required by STM32_CKPERSRC"
  #endif
#endif /* !STM32_CSI_ENABLED */

/**
 * @brief   PLL3 input clock frequency.
 */
#if (STM32_PLL3SRC == STM32_PLL3SRC_HSI) || defined(__DOXYGEN__)
  #define STM32_PLL3MCLK            STM32_HSICLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_HSE
  #define STM32_PLL3MCLK            STM32_HSECLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_CSI
  #define STM32_PLL3MCLK            STM32_CSICLK

#elif STM32_PLL3SRC == STM32_PLL3SRC_NOCLOCK
  #define STM32_PLL3MCLK            0

#else
  #error "invalid STM32_PLL3SRC value specified"
#endif

/*
 * PLL3 enable check.
 * TODO: Check all conditions.
 */
#if TRUE ||                                                                 \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL activation flag.
   */
  #define STM32_ACTIVATE_PLL3       TRUE
#else

  #define STM32_ACTIVATE_PLL3       FALSE
#endif

/**
 * @brief   STM32_PLL3DIVPEN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL3DIVPEN          (1 << 4)

#else
  #define STM32_PLL3DIVPEN          (0 << 4)
#endif

/**
 * @brief   STM32_PLL3DIVQEN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL3DIVQEN          (1 << 5)

#else
  #define STM32_PLL3DIVQEN          (0 << 5)
#endif

/**
 * @brief   STM32_PLL3DIVREN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL3DIVREN          (1 << 6)

#else
  #define STM32_PLL3DIVREN          (0 << 6)
#endif

/**
 * @brief   PLL4 input clock frequency.
 */
#if (STM32_PLL4SRC == STM32_PLL4SRC_HSI) || defined(__DOXYGEN__)
  #define STM32_PLL4MCLK            STM32_HSICLK

#elif STM32_PLL4SRC == STM32_PLL4SRC_HSE
  #define STM32_PLL4MCLK            STM32_HSECLK

#elif STM32_PLL4SRC == STM32_PLL4SRC_CSI
  #define STM32_PLL4MCLK            STM32_CSICLK

#elif STM32_PLL4SRC == STM32_PLL4SRC_I2S_CKIN
  #if STM32_I2S_CKIN_VALUE <= 0
    #error "STM32_I2S_CKIN_VALUE is zero but it is selected as PLL4 input"
  #endif
  #define STM32_PLL4MCLK            STM32_I2S_CKIN_VALUE

#else
  #error "invalid STM32_PLL4SRC value specified"
#endif

/*
 * PLL4 enable check.
 * TODO: Check all conditions.
 */
#if TRUE ||                                                                 \
    defined(__DOXYGEN__)
  /**
   * @brief   PLL activation flag.
   */
  #define STM32_ACTIVATE_PLL4       TRUE
#else

  #define STM32_ACTIVATE_PLL4       FALSE
#endif

/**
 * @brief   STM32_PLL4DIVPEN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL4DIVPEN          (1 << 4)

#else
  #define STM32_PLL4DIVPEN          (0 << 4)
#endif

/**
 * @brief   STM32_PLL4DIVQEN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL4DIVQEN          (1 << 5)

#else
  #define STM32_PLL4DIVQEN          (0 << 5)
#endif

/**
 * @brief   STM32_PLL4DIVREN field.
 * TODO: Check all conditions.
 */
#if TRUE ||                         \
    defined(__DOXYGEN__)
  #define STM32_PLL4DIVREN          (1 << 6)

#else
  #define STM32_PLL4DIVREN          (0 << 6)
#endif

/* Inclusion of PLL-related checks and calculations.*/
#include <stm32_pll3.inc>

/**
 * @brief   MCU system clock source.
 */
#if STM32_NO_INIT || defined(__DOXYGEN__)
  #define STM32_MCUSS_CK             STM32_CSICLK

#elif (STM32_MCUSSRC == STM32_MCUSSRC_HSI)
  #define STM32_MCUSS_CK             STM32_HSICLK

#elif (STM32_MCUSSRC == STM32_MCUSSRC_HSE)
  #define STM32_MCUSS_CK             STM32_HSECLK

#elif (STM32_MCUSSRC == STM32_MCUSSRC_CSI)
  #define STM32_MCUSS_CK             STM32_CSICLK

#elif (STM32_MCUSSRC == STM32_MCUSSRC_PLL3P)
  #define STM32_MCUSS_CK             STM32_PLL3_P_CLKOUT

#else
  #error "invalid STM32_MCUSSRC value specified"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Type of a clock configuration structure.
 */
typedef struct {
  /* TODO */
} halclkcfg_t;
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#if !defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
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
#define hal_lld_get_clock_point(clkpt)                                      \
  ((clkpt) == CLK_SYSCLK   ? STM32_SYSCLK        :                          \
   (clkpt) == CLK_PLLPCLK  ? STM32_PLL_P_CLKOUT  :                          \
   (clkpt) == CLK_PLLQCLK  ? STM32_PLL_Q_CLKOUT  :                          \
   (clkpt) == CLK_PLLRCLK  ? STM32_PLL_R_CLKOUT  :                          \
   (clkpt) == CLK_HCLK     ? STM32_HCLK          :                          \
   (clkpt) == CLK_PCLK1    ? STM32_PCLK1         :                          \
   (clkpt) == CLK_PCLK1TIM ? STM32_TIMP1CLK      :                          \
   (clkpt) == CLK_PCLK2    ? STM32_PCLK2         :                          \
   (clkpt) == CLK_PCLK2TIM ? STM32_TIMP2CLK      :                          \
   (clkpt) == CLK_MCO      ? STM32_MCOCLK        :                          \
   0U)
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"
#include "stm32_isr.h"
//#include "stm32_dma.h"
//#include "stm32_exti.h"
#include "stm32_rcc.h"
//#include "stm32_tim.h"

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) && !defined(__DOXYGEN__)
extern const halclkcfg_t hal_clkcfg_reset;
extern const halclkcfg_t hal_clkcfg_default;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
  bool hal_lld_clock_switch_mode(const halclkcfg_t *ccp);
  halfreq_t hal_lld_get_clock_point(halclkpt_t clkpt);
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
