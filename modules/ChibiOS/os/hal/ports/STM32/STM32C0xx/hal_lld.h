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
    limitations under the License.f
*/

/**
 * @file    STM32C0xx/hal_lld.h
 * @brief   STM32C0xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32C011xx, STM32C031xx.
 *          - STM32C051xx, STM32C071xx.
 *          - STM32C091xx, STM32C092xx.
 *          .
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
 * @brief   Requires use of SPIv2 driver model.
 */
#define HAL_LLD_SELECT_SPI_V2               TRUE

/**
 * @brief   Signals that TIM17 is used internally by the HAL for timeouts.
 */
#define STM32_TIM17_IS_USED

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32C011xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU"

#elif defined(STM32C031xx)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU"

#elif defined(STM32C051xx)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU"

#elif defined(STM32C071xx)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU with USB"

#elif defined(STM32C091xx)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU with FDCAN"

#elif defined(STM32C092xx)
#define PLATFORM_NAME                       "STM32C0 Entry-level Mainstream MCU with FDCAN"

#else
#error "STM32C0 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32C0XX) || defined(__DOXYGEN__)
#define STM32C0XX
#endif
/** @} */

/**
 * @name   Clock points names
 * @{
 */
#if defined(RCC_CR_HSIUSB48ON) || defined(__DOXYGEN__)
#define CLK_SYSCLK                          0U
#define CLK_HSE                             1U
#define CLK_HSISYS                          2U
#define CLK_HSIKER                          3U
#define CLK_HSIUSB48                        4U
#define CLK_HCLK                            5U
#define CLK_PCLK                            6U
#define CLK_PCLKTIM                         7U
#define CLK_MCO1                            8U
#define CLK_MCO2                            9U
#define CLK_ARRAY_SIZE                      10U
#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "SYSCLK", "HSE", "HSISYS", "HSIKER", "HSIUSB48", "HCLK", "PCLK",        \
    "PCLKTIM", "MCO1", "MCO2"                                               \
  }
#else
#define CLK_SYSCLK                          0U
#define CLK_HSE                             1U
#define CLK_HSISYS                          2U
#define CLK_HSIKER                          3U
#define CLK_HCLK                            4U
#define CLK_PCLK                            5U
#define CLK_PCLKTIM                         6U
#define CLK_MCO1                            7U
#define CLK_MCO2                            8U
#define CLK_ARRAY_SIZE                      9U
#define CLK_POINT_NAMES                                                     \
  {                                                                         \
    "SYSCLK", "HSE", "HSISYS", "HSIKER", "HCLK", "PCLK",                    \
    "PCLKTIM", "MCO1", "MCO2"                                               \
  }
#endif
/** @} */

/**
 * @name    PWR_CR2 register helpers
 * @{
 */
#define PWR_CR2_PVM_VDDIO2_FIELD(n)         ((n) << PWR_CR2_PVM_VDDIO2_Pos)
#define PWR_CR2_PVM_VDDIO2_DISABLED         PWR_CR2_PVM_VDDIO2_FIELD(0U)
#define PWR_CR2_PVM_VDDIO2_ENABLED          PWR_CR2_PVM_VDDIO2_FIELD(1U)
#define PWR_CR2_PVM_VDDIO2_BYPASSED         PWR_CR2_PVM_VDDIO2_FIELD(2U)
/** @} */

/**
 * @name    RCC_CR register helpers
 * @{
 */
#define RCC_CR_HSIDIV_FIELD(n)              ((n) << RCC_CR_HSIDIV_Pos)
#define RCC_CR_HSIDIV_DIV1                  RCC_CR_HSIDIV_FIELD(0U)
#define RCC_CR_HSIDIV_DIV2                  RCC_CR_HSIDIV_FIELD(1U)
#define RCC_CR_HSIDIV_DIV4                  RCC_CR_HSIDIV_FIELD(2U)
#define RCC_CR_HSIDIV_DIV8                  RCC_CR_HSIDIV_FIELD(3U)
#define RCC_CR_HSIDIV_DIV16                 RCC_CR_HSIDIV_FIELD(4U)
#define RCC_CR_HSIDIV_DIV32                 RCC_CR_HSIDIV_FIELD(5U)
#define RCC_CR_HSIDIV_DIV64                 RCC_CR_HSIDIV_FIELD(6U)
#define RCC_CR_HSIDIV_DIV128                RCC_CR_HSIDIV_FIELD(7U)

#define RCC_CR_HSIKERDIV_FIELD(n)           ((n) << RCC_CR_HSIKERDIV_Pos)
#define RCC_CR_HSIKERDIV_DIV1               RCC_CR_HSIKERDIV_FIELD(0U)
#define RCC_CR_HSIKERDIV_DIV2               RCC_CR_HSIKERDIV_FIELD(1U)
#define RCC_CR_HSIKERDIV_DIV3               RCC_CR_HSIKERDIV_FIELD(2U)
#define RCC_CR_HSIKERDIV_DIV4               RCC_CR_HSIKERDIV_FIELD(3U)
#define RCC_CR_HSIKERDIV_DIV5               RCC_CR_HSIKERDIV_FIELD(4U)
#define RCC_CR_HSIKERDIV_DIV6               RCC_CR_HSIKERDIV_FIELD(5U)
#define RCC_CR_HSIKERDIV_DIV7               RCC_CR_HSIKERDIV_FIELD(6U)
#define RCC_CR_HSIKERDIV_DIV8               RCC_CR_HSIKERDIV_FIELD(7U)

#if STM32_RCC_HAS_SYSDIV || defined(__DOXYGEN__)
#define RCC_CR_SYSDIV_FIELD(n)              ((n) << RCC_CR_SYSDIV_Pos)
#define RCC_CR_SYSDIV_DIV1                  RCC_CR_SYSDIV_FIELD(0U)
#define RCC_CR_SYSDIV_DIV2                  RCC_CR_SYSDIV_FIELD(1U)
#define RCC_CR_SYSDIV_DIV3                  RCC_CR_SYSDIV_FIELD(2U)
#define RCC_CR_SYSDIV_DIV4                  RCC_CR_SYSDIV_FIELD(3U)
#define RCC_CR_SYSDIV_DIV5                  RCC_CR_SYSDIV_FIELD(4U)
#define RCC_CR_SYSDIV_DIV6                  RCC_CR_SYSDIV_FIELD(5U)
#define RCC_CR_SYSDIV_DIV7                  RCC_CR_SYSDIV_FIELD(6U)
#define RCC_CR_SYSDIV_DIV8                  RCC_CR_SYSDIV_FIELD(7U)
#endif
/** @} */

/**
 * @name    RCC_CFGR register helpers
 * @{
 */
#define RCC_CFGR_SW_FIELD(n)                ((n) << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSISYS                  RCC_CFGR_SW_FIELD(0U)
#define RCC_CFGR_SW_HSE                     RCC_CFGR_SW_FIELD(1U)
#if defined(RCC_CR_HSIUSB48ON) || defined(__DOXYGEN__)
#define RCC_CFGR_SW_HSIUSB48                RCC_CFGR_SW_FIELD(2U)
#endif
#define RCC_CFGR_SW_LSI                     RCC_CFGR_SW_FIELD(3U)
#define RCC_CFGR_SW_LSE                     RCC_CFGR_SW_FIELD(4U)

#define RCC_CFGR_MCO2SEL_FIELD(n)           ((n) << RCC_CFGR_MCO2SEL_Pos)
#define RCC_CFGR_MCO2SEL_NOCLOCK            RCC_CFGR_MCO2SEL_FIELD(0U)
#define RCC_CFGR_MCO2SEL_SYSCLK             RCC_CFGR_MCO2SEL_FIELD(1U)
#define RCC_CFGR_MCO2SEL_HSI48              RCC_CFGR_MCO2SEL_FIELD(3U)
#define RCC_CFGR_MCO2SEL_HSE                RCC_CFGR_MCO2SEL_FIELD(4U)
#define RCC_CFGR_MCO2SEL_LSI                RCC_CFGR_MCO2SEL_FIELD(6U)
#define RCC_CFGR_MCO2SEL_LSE                RCC_CFGR_MCO2SEL_FIELD(7U)
#if defined(RCC_CR_HSIUSB48ON) || defined(__DOXYGEN__)
#define RCC_CFGR_MCO2SEL_HSIUSB48           RCC_CFGR_MCO2SEL_FIELD(8U)
#endif

#define RCC_CFGR_MCO2PRE_FIELD(n)           ((n) << RCC_CFGR_MCO2PRE_Pos)
#define RCC_CFGR_MCO2PRE_DIV1               RCC_CFGR_MCO2PRE_FIELD(0U)
#define RCC_CFGR_MCO2PRE_DIV2               RCC_CFGR_MCO2PRE_FIELD(1U)
#define RCC_CFGR_MCO2PRE_DIV4               RCC_CFGR_MCO2PRE_FIELD(2U)
#define RCC_CFGR_MCO2PRE_DIV8               RCC_CFGR_MCO2PRE_FIELD(3U)
#define RCC_CFGR_MCO2PRE_DIV16              RCC_CFGR_MCO2PRE_FIELD(4U)
#define RCC_CFGR_MCO2PRE_DIV32              RCC_CFGR_MCO2PRE_FIELD(5U)
#define RCC_CFGR_MCO2PRE_DIV64              RCC_CFGR_MCO2PRE_FIELD(6U)
#define RCC_CFGR_MCO2PRE_DIV128             RCC_CFGR_MCO2PRE_FIELD(7U)
#if STM32_RCC_HAS_MCODIVEXT || defined(__DOXYGEN__)
#define RCC_CFGR_MCO2PRE_DIV256             RCC_CFGR_MCO2PRE_FIELD(8U)
#define RCC_CFGR_MCO2PRE_DIV512             RCC_CFGR_MCO2PRE_FIELD(9U)
#define RCC_CFGR_MCO2PRE_DIV1024            RCC_CFGR_MCO2PRE_FIELD(10U)
#endif

#define RCC_CFGR_MCOSEL_FIELD(n)            ((n) << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_NOCLOCK             RCC_CFGR_MCOSEL_FIELD(0U)
#define RCC_CFGR_MCOSEL_SYSCLK              RCC_CFGR_MCOSEL_FIELD(1U)
#define RCC_CFGR_MCOSEL_HSI48               RCC_CFGR_MCOSEL_FIELD(3U)
#define RCC_CFGR_MCOSEL_HSE                 RCC_CFGR_MCOSEL_FIELD(4U)
#define RCC_CFGR_MCOSEL_LSI                 RCC_CFGR_MCOSEL_FIELD(6U)
#define RCC_CFGR_MCOSEL_LSE                 RCC_CFGR_MCOSEL_FIELD(7U)
#if defined(RCC_CR_HSIUSB48ON) || defined(__DOXYGEN__)
#define RCC_CFGR_MCOSEL_HSIUSB48            RCC_CFGR_MCOSEL_FIELD(8U)
#endif

#define RCC_CFGR_MCOPRE_FIELD(n)            ((n) << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV1                RCC_CFGR_MCOPRE_FIELD(0U)
#define RCC_CFGR_MCOPRE_DIV2                RCC_CFGR_MCOPRE_FIELD(1U)
#define RCC_CFGR_MCOPRE_DIV4                RCC_CFGR_MCOPRE_FIELD(2U)
#define RCC_CFGR_MCOPRE_DIV8                RCC_CFGR_MCOPRE_FIELD(3U)
#define RCC_CFGR_MCOPRE_DIV16               RCC_CFGR_MCOPRE_FIELD(4U)
#define RCC_CFGR_MCOPRE_DIV32               RCC_CFGR_MCOPRE_FIELD(5U)
#define RCC_CFGR_MCOPRE_DIV64               RCC_CFGR_MCOPRE_FIELD(6U)
#define RCC_CFGR_MCOPRE_DIV128              RCC_CFGR_MCOPRE_FIELD(7U)
#if STM32_RCC_HAS_MCODIVEXT || defined(__DOXYGEN__)
#define RCC_CFGR_MCOPRE_DIV256              RCC_CFGR_MCOPRE_FIELD(8U)
#define RCC_CFGR_MCOPRE_DIV512              RCC_CFGR_MCOPRE_FIELD(9U)
#define RCC_CFGR_MCOPRE_DIV1024             RCC_CFGR_MCOPRE_FIELD(10U)
#endif
/** @} */

/**
 * @name    RCC_CCIPR register helpers
 * @{
 */
#define RCC_CCIPR_USART1SEL_FIELD(n)        ((n) << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_PCLK            RCC_CCIPR_USART1SEL_FIELD(0U)
#define RCC_CCIPR_USART1SEL_SYSCLK          RCC_CCIPR_USART1SEL_FIELD(1U)
#define RCC_CCIPR_USART1SEL_HSIKER          RCC_CCIPR_USART1SEL_FIELD(2U)
#define RCC_CCIPR_USART1SEL_LSE             RCC_CCIPR_USART1SEL_FIELD(3U)

#if defined(RCC_CCIPR_FDCAN1SEL_Pos) || defined(__DOXYGEN__)
#define RCC_CCIPR_FDCAN1SEL_FIELD(n)        ((n) << RCC_CCIPR_FDCAN1SEL_Pos)
#else
#define RCC_CCIPR_FDCAN1SEL_FIELD(n)        0U
#endif
#define RCC_CCIPR_FDCAN1SEL_PCLK            RCC_CCIPR_FDCAN1SEL_FIELD(0U)
#define RCC_CCIPR_FDCAN1SEL_HSIKER          RCC_CCIPR_FDCAN1SEL_FIELD(1U)
#define RCC_CCIPR_FDCAN1SEL_HSE             RCC_CCIPR_FDCAN1SEL_FIELD(2U)

#define RCC_CCIPR_I2C1SEL_FIELD(n)          ((n) << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL_PCLK              RCC_CCIPR_I2C1SEL_FIELD(0U)
#define RCC_CCIPR_I2C1SEL_SYSCLK            RCC_CCIPR_I2C1SEL_FIELD(1U)
#define RCC_CCIPR_I2C1SEL_HSIKER            RCC_CCIPR_I2C1SEL_FIELD(2U)

#define RCC_CCIPR_I2S1SEL_FIELD(n)          ((n) << RCC_CCIPR_I2S1SEL_Pos)
#define RCC_CCIPR_I2S1SEL_SYSCLK            RCC_CCIPR_I2S1SEL_FIELD(0U)
#define RCC_CCIPR_I2S1SEL_HSIKER            RCC_CCIPR_I2S1SEL_FIELD(2U)
#define RCC_CCIPR_I2S1SEL_CKIN              RCC_CCIPR_I2S1SEL_FIELD(3U)

#define RCC_CCIPR_ADCSEL_FIELD(n)          ((n) << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL_SYSCLK             RCC_CCIPR_ADCSEL_FIELD(0U)
#define RCC_CCIPR_ADCSEL_HSIKER             RCC_CCIPR_ADCSEL_FIELD(2U)
/** @} */

/**
 * @name    RCC_CCIPR2 register helpers
 * @{
 */
#if defined(RCC_CCIPR2_USBSEL_Pos) || defined(__DOXYGEN__)
#define RCC_CCIPR2_USBSEL_FIELD(n)          ((n) << RCC_CCIPR2_USBSEL_Pos)
#else
#define RCC_CCIPR2_USBSEL_FIELD(n)          0U
#endif
#define RCC_CCIPR2_USBSEL_HSIUSB48          RCC_CCIPR2_USBSEL_FIELD(0U)
#define RCC_CCIPR2_USBSEL_HSE               RCC_CCIPR2_USBSEL_FIELD(1U)
/** @} */

/**
 * @name    RCC_CSR1 register helpers
 * @{
 */
#define RCC_CSR1_RTCSEL_FIELD(n)            ((n) << RCC_CSR1_RTCSEL_Pos)
#define RCC_CSR1_RTCSEL_NOCLOCK             RCC_CSR1_RTCSEL_FIELD(0U)
#define RCC_CSR1_RTCSEL_LSE                 RCC_CSR1_RTCSEL_FIELD(1U)
#define RCC_CSR1_RTCSEL_LSI                 RCC_CSR1_RTCSEL_FIELD(2U)
#define RCC_CSR1_RTCSEL_HSEDIV              RCC_CSR1_RTCSEL_FIELD(3U)

#define RCC_CSR1_LSCOSEL_FIELD(n)           ((n) << RCC_CSR1_LSCOSEL_Pos)
#define RCC_CSR1_LSCOSEL_NOCLOCK            RCC_CSR1_LSCOSEL_FIELD(0U)
#define RCC_CSR1_LSCOSEL_LSI                RCC_CSR1_LSCOSEL_FIELD(1U)
#define RCC_CSR1_LSCOSEL_LSE                RCC_CSR1_LSCOSEL_FIELD(3U)
/** @} */

/* ST headers inconsistencies...*/
#if !defined(FLASH_ACR_LATENCY_0WS)
#define FLASH_ACR_LATENCY_0WS   (0U << FLASH_ACR_LATENCY_Pos)
#endif
#if !defined(FLASH_ACR_LATENCY_1WS)
#define FLASH_ACR_LATENCY_1WS   (1U << FLASH_ACR_LATENCY_Pos)
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT                       FALSE
#endif

/**
 * @brief   Enables the dynamic clock handling.
 */
#if !defined(STM32_CLOCK_DYNAMIC) || defined(__DOXYGEN__)
#define STM32_CLOCK_DYNAMIC                 FALSE
#endif

/**
 * @brief   SYSCFG CFGR1 register initialization value.
 */
#if !defined(STM32_SYSCFG_CFGR1) || defined(__DOXYGEN__)
#define STM32_SYSCFG_CFGR1                  (0U)
#endif

/**
 * @brief   SYSCFG CFGR2 register initialization value.
 */
#if !defined(STM32_SYSCFG_CFGR2) || defined(__DOXYGEN__)
#define STM32_SYSCFG_CFGR2                  (0U)
#endif

/**
 * @brief   SYSCFG CFGR3 register initialization value.
 */
#if !defined(STM32_SYSCFG_CFGR3) || defined(__DOXYGEN__)
#define STM32_SYSCFG_CFGR3                  (0U)
#endif

/**
 * @brief   PWR CR2 register initialization value.
 */
#if !defined(STM32_PWR_CR2) || defined(__DOXYGEN__)
#define STM32_PWR_CR2                       (PWR_CR2_PVM_VDDIO2_ENABLED)
#endif

/**
 * @brief   PWR CR3 register initialization value.
 */
#if !defined(STM32_PWR_CR3) || defined(__DOXYGEN__)
#define STM32_PWR_CR3                       (PWR_CR3_EIWUL)
#endif

/**
 * @brief   PWR CR4 register initialization value.
 */
#if !defined(STM32_PWR_CR4) || defined(__DOXYGEN__)
#define STM32_PWR_CR4                       (0U)
#endif

/**
 * @brief   PWR PUCRA register initialization value.
 */
#if !defined(STM32_PWR_PUCRA) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRA                     (0U)
#endif

/**
 * @brief   PWR PDCRA register initialization value.
 */
#if !defined(STM32_PWR_PDCRA) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRA                     (0U)
#endif

/**
 * @brief   PWR PUCRB register initialization value.
 */
#if !defined(STM32_PWR_PUCRB) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRB                     (0U)
#endif

/**
 * @brief   PWR PDCRB register initialization value.
 */
#if !defined(STM32_PWR_PDCRB) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRB                     (0U)
#endif

/**
 * @brief   PWR PUCRC register initialization value.
 */
#if !defined(STM32_PWR_PUCRC) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRC                     (0U)
#endif

/**
 * @brief   PWR PDCRC register initialization value.
 */
#if !defined(STM32_PWR_PDCRC) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRC                     (0U)
#endif

/**
 * @brief   PWR PUCRD register initialization value.
 */
#if !defined(STM32_PWR_PUCRD) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRD                     (0U)
#endif

/**
 * @brief   PWR PDCRD register initialization value.
 */
#if !defined(STM32_PWR_PDCRD) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRD                     (0U)
#endif

/**
 * @brief   PWR PUCRF register initialization value.
 */
#if !defined(STM32_PWR_PUCRF) || defined(__DOXYGEN__)
#define STM32_PWR_PUCRF                     (0U)
#endif

/**
 * @brief   PWR PDCRF register initialization value.
 */
#if !defined(STM32_PWR_PDCRF) || defined(__DOXYGEN__)
#define STM32_PWR_PDCRF                     (0U)
#endif

/**
 * @brief   FLASH ACR register initialization value.
 * @note    Do not specify the LATENCY bits because those are calculated
 *          depending on other settings and ORed to this value.
 */
#if !defined(STM32_FLASH_ACR) || defined(__DOXYGEN__)
#define STM32_FLASH_ACR                     (FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN)
#endif

/**
 * @brief   HSIDIV divider value.
 * @note    The allowed values are 1, 2, 4, 8, 16, 32, 64, 128.
 */
#if !defined(STM32_HSIDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_HSIDIV_VALUE                  1
#endif

/**
 * @brief   HSIKER divider value.
 * @note    The allowed values are 1..8.
 */
#if !defined(STM32_HSIKERDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_HSIKERDIV_VALUE               1
#endif

/**
 * @brief   Enables or disables the HSI48 clock source.
 */
#if !defined(STM32_HSI48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI48_ENABLED                 TRUE
#endif

/**
 * @brief   Enables or disables the HSIUSB48 clock source.
 */
#if !defined(STM32_HSIUSB48_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSIUSB48_ENABLED              FALSE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED                   FALSE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED                   FALSE
#endif

/**
 * @brief   Main clock source selection.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                            RCC_CFGR_SW_HSISYS
#endif

/**
 * @brief   SYSDIV divider value.
 * @note    The allowed values are 1..8.
 */
#if !defined(STM32_SYSDIV_VALUE) || defined(__DOXYGEN__)
#define STM32_SYSDIV_VALUE                  1
#endif

/**
 * @brief   AHB prescaler value.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                          RCC_CFGR_HPRE_DIV1
#endif

/**
 * @brief   APB prescaler value.
 */
#if !defined(STM32_PPRE) || defined(__DOXYGEN__)
#define STM32_PPRE                          RCC_CFGR_PPRE_DIV1
#endif

/**
 * @brief   MCO clock source.
 */
#if !defined(STM32_MCOSEL) || defined(__DOXYGEN__)
#define STM32_MCOSEL                        RCC_CFGR_MCOSEL_NOCLOCK
#endif

/**
 * @brief   MCO divider setting.
 */
#if !defined(STM32_MCOPRE) || defined(__DOXYGEN__)
#define STM32_MCOPRE                        RCC_CFGR_MCOPRE_DIV1
#endif

/**
 * @brief   MCO2 clock source.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL                       RCC_CFGR_MCO2SEL_NOCLOCK
#endif

/**
 * @brief   MCO2 divider setting.
 */
#if !defined(STM32_MCO2PRE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE                       RCC_CFGR_MCO2PRE_DIV1
#endif

/**
 * @brief   LSCO clock source.
 */
#if !defined(STM32_LSCOSEL) || defined(__DOXYGEN__)
#define STM32_LSCOSEL                       RCC_CSR1_LSCOSEL_NOCLOCK
#endif

/**
 * @brief   USART1 clock source.
 */
#if !defined(STM32_USART1SEL) || defined(__DOXYGEN__)
#define STM32_USART1SEL                     RCC_CCIPR_USART1SEL_SYSCLK
#endif

/**
 * @brief   FDCAN1 clock source.
 */
#if !defined(STM32_FDCAN1SEL) || defined(__DOXYGEN__)
#define STM32_FDCAN1SEL                     RCC_CCIPR_FDCAN1SEL_PCLK
#endif

/**
 * @brief   I2C1 clock source.
 */
#if !defined(STM32_I2C1SEL) || defined(__DOXYGEN__)
#define STM32_I2C1SEL                       RCC_CCIPR_I2C1SEL_PCLK
#endif

/**
 * @brief   I2S1 clock source.
 */
#if !defined(STM32_I2S1SEL) || defined(__DOXYGEN__)
#define STM32_I2S1SEL                       RCC_CCIPR_I2S1SEL_SYSCLK
#endif

/**
 * @brief   ADC clock source.
 */
#if !defined(STM32_ADCSEL) || defined(__DOXYGEN__)
#define STM32_ADCSEL                        RCC_CCIPR_ADCSEL_SYSCLK
#endif

/**
 * @brief   USB clock source.
 */
#if !defined(STM32_USBSEL) || defined(__DOXYGEN__)
#define STM32_USBSEL                        RCC_CCIPR2_USBSEL_HSIUSB48
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                        RCC_CSR1_RTCSEL_NOCLOCK
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
#if !defined(STM32C0xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C0xx_MCUCONF not defined"
#endif

#if defined(STM32C011xx) && !defined(STM32C011_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C011_MCUCONF not defined"

#elif defined(STM32C031xx) && !defined(STM32C031_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C031_MCUCONF not defined"

#elif defined(STM32C051xx) && !defined(STM32C051_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C051_MCUCONF not defined"

#elif defined(STM32C071xx) && !defined(STM32C071_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C071_MCUCONF not defined"

#elif defined(STM32C091xx) && !defined(STM32C091_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C091_MCUCONF not defined"

#elif defined(STM32C092xx) && !defined(STM32C092_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32C092_MCUCONF not defined"

#endif

/*
 * Board files sanity checks.
 */
#if !defined(STM32_LSECLK)
#error "STM32_LSECLK not defined in board.h"
#endif

#if !defined(STM32_LSEDRV)
#error "STM32_LSEDRV not defined in board.h"
#endif

#if !defined(STM32_HSECLK)
#error "STM32_HSECLK not defined in board.h"
#endif

/**
 * @name    System Limits
 * @{
 */
#define STM32_SYSCLK_MAX                    48000000
#define STM32_HSECLK_MAX                    48000000
#define STM32_HSECLK_BYP_MAX                48000000
#define STM32_HSECLK_MIN                    4000000
#define STM32_HSECLK_BYP_MIN                8000000
#define STM32_LSECLK_MAX                    32768
#define STM32_LSECLK_BYP_MAX                1000000
#define STM32_LSECLK_MIN                    32768
#define STM32_LSECLK_BYP_MIN                32768
#define STM32_PCLK_MAX                      48000000
#define STM32_ADCCLK_MAX                    35000000

#define STM32_0WS_THRESHOLD                 24000000
#define STM32_1WS_THRESHOLD                 48000000
/** @} */

/* Clock handlers.*/
#include "stm32_lse_v3.inc"
#include "stm32_lsi_v3.inc"
#include "stm32_hsi48.inc"
#include "stm32_hsiusb48.inc"
#include "stm32_hse.inc"

/*
 * HSI48 related checks.
 */
#if STM32_RCC_HAS_HSI48
#if STM32_HSI48_ENABLED
#else /* !STM32_HSI48_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_HSISYS
    #error "HSI48 not enabled, required by STM32_SW"
  #endif

  #if STM32_MCOSEL == RCC_CFGR_MCOSEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI48
    #error "HSI48 not enabled, required by STM32_MCO2SEL"
  #endif

  #if (STM32_ADCSEL == RCC_CCIPR_ADCSEL_HSIKER)
    #error "HSI48 not enabled, required by STM32_ADCSEL"
  #endif

  #if (STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_HSIKER)
    #error "HSI48 not enabled, required by STM32_I2C1SEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_HSIKER)
    #error "HSI48 not enabled, required by STM32_USART1SEL"
  #endif

#endif /* !STM32_HSI48_ENABLED */
#endif /* STM32_RCC_HAS_HSI48 */

/*
 * HSIUSB48 related checks.
 */
#if STM32_RCC_HAS_HSIUSB48
#if STM32_HSIUSB48_ENABLED
#else /* !STM32_HSIUSB48_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_HSIUSB48
    #error "HSIUSB48 not enabled, required by STM32_SW"
  #endif

  #if STM32_MCOSEL == RCC_CFGR_MCOSEL_HSIUSB48
    #error "HSIUSB48 not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSIUSB48
    #error "HSIUSB48 not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_USBSEL == RCC_CCIPR2_USBSEL_HSIUSB48
    #error "HSIUSB48 not enabled, required by STM32_USBSEL"
  #endif

#endif /* !STM32_HSIUSB48_ENABLED */
#endif /* STM32_RCC_HAS_HSI48 */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED

  #if STM32_HSECLK == 0
   #error "HSE oscillator not available on the board"
  #endif

#else /* !STM32_HSE_ENABLED */

  #if STM32_SW == RCC_CFGR_SW_HSE
    #error "HSE not enabled, required by STM32_SW"
  #endif

  #if STM32_MCOSEL == RCC_CFGR_MCOSEL_HSE
    #error "HSE not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSE
    #error "HSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_RTCSEL == RCC_CSR1_RTCSEL_HSEDIV
    #error "HSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_HAS_USB1
    #if STM32_USBSEL == RCC_CCIPR2_USBSEL_HSE
      #error "HSE not enabled, required by STM32_USBSEL"
    #endif
  #endif

#endif /* !STM32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

  #if HAL_USE_RTC && (STM32_RTCSEL == RCC_CSR1_RTCSEL_LSI)
    #error "LSI not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCOSEL == RCC_CFGR_MCOSEL_LSI
    #error "LSI not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSI
    #error "LSI not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_LSCOSEL == RCC_CSR1_LSCOSEL_LSI
    #error "LSI not enabled, required by STM32_LSCOSEL"
  #endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

  #if STM32_LSECLK == 0
   #error "LSE oscillator not available on the board"
  #endif

#else /* !STM32_LSE_ENABLED */

  #if STM32_RTCSEL == RCC_CSR1_RTCSEL_LSE
    #error "LSE not enabled, required by STM32_RTCSEL"
  #endif

  #if STM32_MCOSEL == RCC_CFGR_MCOSEL_LSE
    #error "LSE not enabled, required by STM32_MCOSEL"
  #endif

  #if STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSE
    #error "LSE not enabled, required by STM32_MCO2SEL"
  #endif

  #if STM32_LSCOSEL == RCC_CSR1_LSCOSEL_LSE
    #error "LSE not enabled, required by STM32_LSCOSEL"
  #endif

  #if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_LSE)
    #error "LSE not enabled, required by STM32_USART1SEL"
  #endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   STM32_HSIDIV field.
 */
#if (STM32_HSIDIV_VALUE == 1) || defined(__DOXYGEN__)
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV1

#elif STM32_HSIDIV_VALUE == 2
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV2

#elif STM32_HSIDIV_VALUE == 4
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV4

#elif STM32_HSIDIV_VALUE == 8
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV8

#elif STM32_HSIDIV_VALUE == 16
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV16

#elif STM32_HSIDIV_VALUE == 32
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV32

#elif STM32_HSIDIV_VALUE == 64
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV64

#elif STM32_HSIDIV_VALUE == 128
  #define STM32_HSIDIV                      RCC_CR_HSIDIV_DIV128

#else
  #error "invalid STM32_HSIDIV_VALUE value specified"
#endif

/**
 * @brief   STM32_HSIKERDIV field.
 */
#if (STM32_HSIKERDIV_VALUE == 1) || defined(__DOXYGEN__)
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV1

#elif STM32_HSIKERDIV_VALUE == 2
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV2

#elif STM32_HSIKERDIV_VALUE == 3
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV3

#elif STM32_HSIKERDIV_VALUE == 4
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV4

#elif STM32_HSIKERDIV_VALUE == 5
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV5

#elif STM32_HSIKERDIV_VALUE == 6
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV6

#elif STM32_HSIKERDIV_VALUE == 7
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV7

#elif STM32_HSIKERDIV_VALUE == 8
  #define STM32_HSIKERDIV                   RCC_CR_HSIKERDIV_DIV8

#else
  #error "invalid STM32_HSIKERDIV_VALUE value specified"
#endif

/**
 * @brief   STM32_SYSDIV field.
 */
#if STM32_RCC_HAS_SYSDIV || defined(__DOXYGEN__)
  #if (STM32_SYSDIV_VALUE == 1) || defined(__DOXYGEN__)
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV1

  #elif STM32_SYSDIV_VALUE == 2
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV2

  #elif STM32_SYSDIV_VALUE == 3
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV3

  #elif STM32_SYSDIV_VALUE == 4
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV4

  #elif STM32_SYSDIV_VALUE == 5
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV5

  #elif STM32_SYSDIV_VALUE == 6
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV6

  #elif STM32_SYSDIV_VALUE == 7
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV7

  #elif STM32_SYSDIV_VALUE == 8
    #define STM32_SYSDIV                    RCC_CR_SYSDIV_DIV8

  #else
    #error "invalid STM32_SYSDIV_VALUE value specified"
  #endif
#else
  #undef STM32_SYSDIV_VALUE                 /* Ignoring the setting.*/
  #define STM32_SYSDIV_VALUE                1U
  #define STM32_SYSDIV                      0U
#endif

/**
 * @brief   HSISYS clock frequency.
 */
#define STM32_HSISYSCLK                     (STM32_HSI48CLK / STM32_HSIDIV_VALUE)

/**
 * @brief   HSIKER clock frequency.
 */
#define STM32_HSIKERCLK                     (STM32_HSI48CLK / STM32_HSIKERDIV_VALUE)

/**
 * @brief   System clock source.
 */
#if (STM32_SW == RCC_CFGR_SW_HSISYS)
  #define STM32_SYSCLK                      (STM32_HSISYSCLK / STM32_SYSDIV_VALUE)

#elif (STM32_SW == RCC_CFGR_SW_HSE)
  #define STM32_SYSCLK                      (STM32_HSECLK / STM32_SYSDIV_VALUE)

#elif defined(RCC_CR_HSIUSB48ON) && (STM32_SW == RCC_CFGR_SW_HSIUSB48)
  #define STM32_SYSCLK                      (STM32_HSIUSB48CLK / STM32_SYSDIV_VALUE)

#elif (STM32_SW == RCC_CFGR_SW_LSI)
  #define STM32_SYSCLK                      (STM32_LSICLK / STM32_SYSDIV_VALUE)

#elif (STM32_SW == RCC_CFGR_SW_LSE)
  #define STM32_SYSCLK                      (STM32_LSECLK / STM32_SYSDIV_VALUE)

#else
#error "invalid STM32_SW value specified"
#endif

/* Bus handlers.*/
#include "stm32_ahb.inc"
#include "stm32_apb.inc"

/*
 * Compatibility definitions.
 */
#define STM32_PCLK1                         STM32_PCLK
#define STM32_PCLK2                         STM32_PCLK

/**
 * @brief   MCO divider clock frequency.
 */
#if (STM32_MCOSEL == RCC_CFGR_MCOSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCODIVCLK                   0

#elif STM32_MCOSEL == RCC_CFGR_MCOSEL_SYSCLK
  #define STM32_MCODIVCLK                   STM32_SYSCLK

#elif STM32_MCOSEL == RCC_CFGR_MCOSEL_HSI48
  #define STM32_MCODIVCLK                   STM32_HSI48CLK

#elif STM32_MCOSEL == RCC_CFGR_MCOSEL_HSE
  #define STM32_MCODIVCLK                   STM32_HSECLK

#elif STM32_MCOSEL == RCC_CFGR_MCOSEL_LSI
  #define STM32_MCODIVCLK                   STM32_LSICLK

#elif STM32_MCOSEL == RCC_CFGR_MCOSEL_LSE
  #define STM32_MCODIVCLK                   STM32_LSECLK

#else
  #error "invalid STM32_MCOSEL value specified"
#endif

/**
 * @brief   MCO output pin clock frequency.
 */
#if (STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_MCOCLK                      STM32_MCODIVCLK

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV2
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 2)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV4
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 4)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV8
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 8)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV16
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 16)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV32
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 32)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV64
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 64)

#elif STM32_MCOPRE == RCC_CFGR_MCOPRE_DIV128
  #define STM32_MCOCLK                      (STM32_MCODIVCLK / 128)

#else
  #error "invalid STM32_MCOPRE value specified"
#endif

/**
 * @brief   MCO2 divider clock frequency.
 */
#if (STM32_MCO2SEL == RCC_CFGR_MCO2SEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_MCO2DIVCLK                  0

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_SYSCLK
  #define STM32_MCO2DIVCLK                  STM32_SYSCLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSI48
  #define STM32_MCO2DIVCLK                  STM32_HSI48CLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_HSE
  #define STM32_MCO2DIVCLK                  STM32_HSECLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSI
  #define STM32_MCO2DIVCLK                  STM32_LSICLK

#elif STM32_MCO2SEL == RCC_CFGR_MCO2SEL_LSE
  #define STM32_MCO2DIVCLK                  STM32_LSECLK

#else
  #error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock frequency.
 */
#if (STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_MCO2CLK                     STM32_MCO2DIVCLK

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV2
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 2)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV4
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 4)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV8
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 8)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV16
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 16)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV32
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 32)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV64
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 64)

#elif STM32_MCO2PRE == RCC_CFGR_MCO2PRE_DIV128
  #define STM32_MCO2CLK                     (STM32_MCO2DIVCLK / 128)

#else
  #error "invalid STM32_MCO2PRE value specified"
#endif

/**
 * @brief   RTC clock frequency.
 */
#if (STM32_RTCSEL == RCC_CSR1_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
  #define STM32_RTCCLK                      0

#elif STM32_RTCSEL == RCC_CSR1_RTCSEL_LSE
  #define STM32_RTCCLK                      STM32_LSECLK

#elif STM32_RTCSEL == RCC_CSR1_RTCSEL_LSI
  #define STM32_RTCCLK                      STM32_LSICLK

#elif STM32_RTCSEL == RCC_CSR1_RTCSEL_HSEDIV
  #define STM32_RTCCLK                      (hal_lld_get_clock_point(CLK_HSE) / 32)

#else
  #error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   USART1 clock frequency.
 */
#if (STM32_USART1SEL == RCC_CCIPR_USART1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_SYSCLK
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_HSIKER
  #define STM32_USART1CLK                   hal_lld_get_clock_point(CLK_HSIKER)

#elif STM32_USART1SEL == RCC_CCIPR_USART1SEL_LSE
  #define STM32_USART1CLK                   STM32_LSECLK

#else
  #error "invalid source selected for USART1 clock"
#endif

/**
 * @brief   USART2 clock frequency.
 */
#define STM32_USART2CLK                     hal_lld_get_clock_point(CLK_PCLK)

/**
 * @brief   FDCAN1 clock frequency.
 */
#if (STM32_FDCAN1SEL == RCC_CCIPR_FDCAN1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_FDCAN1CLK                   hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_FDCAN1SEL == RCC_CCIPR_FDCAN1SEL_HSIKER
  #define STM32_FDCAN1CLK                   hal_lld_get_clock_point(CLK_HSIKER)

#elif STM32_FDCAN1SEL == RCC_CCIPR_FDCAN1SEL_HSE
  #define STM32_FDCAN1CLK                   hal_lld_get_clock_point(CLK_HSE)

#else
  #error "invalid source selected for FDCAN1 clock"
#endif

/**
 * @brief   I2C1 clock frequency.
 */
#if (STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_PCLK) || defined(__DOXYGEN__)
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_PCLK)

#elif STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_SYSCLK
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2C1SEL == RCC_CCIPR_I2C1SEL_HSIKER
  #define STM32_I2C1CLK                     hal_lld_get_clock_point(CLK_HSIKER)

#else
  #error "invalid source selected for I2C1 clock"
#endif

/**
 * @brief   I2C2 clock frequency.
 */
#define STM32_I2C2CLK                       hal_lld_get_clock_point(CLK_PCLK)

/**
 * @brief   I2S1 clock frequency.
 */
#if (STM32_I2S1SEL == RCC_CCIPR_I2S1SEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_I2S1CLK                     hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_I2S1SEL == RCC_CCIPR_I2S1SEL_HSIKER
  #define STM32_I2S1CLK                     hal_lld_get_clock_point(CLK_HSIKER)

#elif STM32_I2S1SEL == RCC_CCIPR_I2S1SEL_CKIN
  #define STM32_I2S1CLK                     0 /* Unknown, would require a board value */

#else
#error "invalid source selected for I2S1 clock"
#endif

/**
 * @brief   ADC clock frequency.
 */
#if (STM32_ADCSEL == RCC_CCIPR_ADCSEL_SYSCLK) || defined(__DOXYGEN__)
  #define STM32_ADCCLK                      hal_lld_get_clock_point(CLK_SYSCLK)

#elif STM32_ADCSEL == RCC_CCIPR_ADCSEL_HSIKER
  #define STM32_ADCCLK                      hal_lld_get_clock_point(CLK_HSIKER)

#else
  #error "invalid source selected for ADC clock"
#endif

/**
 * @brief   USB clock frequency.
 */
#if (STM32_USBSEL == RCC_CCIPR2_USBSEL_HSIUSB48) || defined(__DOXYGEN__)
  #define STM32_USBCLK                      hal_lld_get_clock_point(CLK_HSIUSB48)

#elif STM32_USBSEL == RCC_CCIPR2_USBSEL_HSE
  #define STM32_USBCLK                      STM32_HSECLK

#else
  #error "invalid source selected for USB clock"
#endif

/**
 * @brief   TIMPCLK clock frequency.
 */
#if (STM32_PPRE == STM32_PPRE_DIV1) || defined(__DOXYGEN__)
  #define STM32_TIMPCLK                     (STM32_PCLK * 1)

#else
  #define STM32_TIMPCLK                     (STM32_PCLK * 2)
#endif

/**
 * @brief   Clock of timers connected to APB1.
 */
#define STM32_TIMCLK1                       hal_lld_get_clock_point(CLK_PCLKTIM)

/**
 * @brief   Clock of timers connected to APB2.
 */
#define STM32_TIMCLK2                       hal_lld_get_clock_point(CLK_PCLKTIM)

/**
 * @brief   Flash settings.
 */
#if (STM32_HCLK <= STM32_0WS_THRESHOLD) || defined(__DOXYGEN__)
  #define STM32_FLASHBITS                   FLASH_ACR_LATENCY_0WS

#else
  #define STM32_FLASHBITS                   FLASH_ACR_LATENCY_1WS
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a clock configuration and switch structure.
 */
typedef struct {
  uint32_t          rcc_cr;
  uint32_t          rcc_cfgr;
  uint32_t          flash_acr;
} halclkcfg_t;

/**
 * @brief   Type of a timeout counter.
 * @note    16 bits because it must match TIM17 counter size.
 */
typedef uint16_t halcnt_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Real time counter frequency exported to the safety module.
 * @note    The counter is clocked at 1MHz, the prescaled is recalculated
 *          if/when the input frequency changes.
 */
#define HAL_LLD_GET_CNT_FREQUENCY()         1000000U

/**
 * @brief   Real time counter value exported to the safety module.
 */
#define HAL_LLD_GET_CNT_VALUE()             ((halcnt_t)TIM17->CNT)

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
#if defined(CLK_HSIUSB48) || defined(__DoXYGEN__)
#define hal_lld_get_clock_point(clkpt)                                      \
  ((clkpt) == CLK_SYSCLK        ? STM32_SYSCLK      :                       \
   (clkpt) == CLK_HSE           ? STM32_HSECLK      :                       \
   (clkpt) == CLK_HSISYS        ? STM32_HSISYSCLK   :                       \
   (clkpt) == CLK_HSIKER        ? STM32_HSIKERCLK   :                       \
   (clkpt) == CLK_HSIUSB48      ? STM32_HSIUSB48CLK :                       \
   (clkpt) == CLK_HCLK          ? STM32_HCLK        :                       \
   (clkpt) == CLK_PCLK          ? STM32_PCLK        :                       \
   (clkpt) == CLK_PCLKTIM       ? STM32_TIMPCLK     :                       \
   (clkpt) == CLK_MCO1          ? STM32_MCOCLK      :                       \
   (clkpt) == CLK_MCO2          ? STM32_MCO2CLK     :                       \
   0U)
#else
#define hal_lld_get_clock_point(clkpt)                                      \
  ((clkpt) == CLK_SYSCLK        ? STM32_SYSCLK      :                       \
   (clkpt) == CLK_HSE           ? STM32_HSECLK      :                       \
   (clkpt) == CLK_HSISYS        ? STM32_HSISYSCLK   :                       \
   (clkpt) == CLK_HSIKER        ? STM32_HSIKERCLK   :                       \
   (clkpt) == CLK_HCLK          ? STM32_HCLK        :                       \
   (clkpt) == CLK_PCLK          ? STM32_PCLK        :                       \
   (clkpt) == CLK_PCLKTIM       ? STM32_TIMPCLK     :                       \
   (clkpt) == CLK_MCO1          ? STM32_MCOCLK      :                       \
   (clkpt) == CLK_MCO2          ? STM32_MCO2CLK     :                       \
   0U)
#endif
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "stm32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

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
