/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_lld.h
 * @brief   ADUCM36x HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - ADUCM_LFXTAL.
 *          .
 *          One of the following macros must also be defined:
 *          - ADUCM360, ADUCM361.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "aducm_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(ADUCM360) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "ADUCM360 Precision Analog MCU with Dual 24-bit ADC"

#elif defined(ADUCM361)
#define PLATFORM_NAME           "ADUCM361 Precision Analog MCU with Single 24-bit ADC"

#else
#error "ADUCM36x device unsupported or not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum External Low Frequency crystal clock frequency.
 */
#define ADUCM_LFXTAL_MAX        32768
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define ADUCM_HFOSC             16000000    /**< High speed internal clock. */
#define ADUCM_LFOSC             32768       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    CLKCON0 register bits definitions
 * @{
 */
#define ADUCM_CD_DIV1           (0 << 0)    /**< Clock divided by 1.        */
#define ADUCM_CD_DIV2           (1 << 0)    /**< Clock divided by 2.        */
#define ADUCM_CD_DIV4           (2 << 0)    /**< Clock divided by 4.        */
#define ADUCM_CD_DIV8           (3 << 0)    /**< Clock divided by 8.        */
#define ADUCM_CD_DIV16          (4 << 0)    /**< Clock divided by 16.       */
#define ADUCM_CD_DIV32          (5 << 0)    /**< Clock divided by 32.       */
#define ADUCM_CD_DIV64          (6 << 0)    /**< Clock divided by 64.       */
#define ADUCM_CD_DIV128         (7 << 0)    /**< Clock divided by 128.      */

#define ADUCM_CLKMUX_HFOSC      (0 << 3)    /**< Clock source is HFOSC.     */
#define ADUCM_CLKMUX_LFXTAL     (1 << 3)    /**< Clock source is LFXTAL.    */
#define ADUCM_CLKMUX_LFOSC      (2 << 3)    /**< Clock source is LFOSC.     */
#define ADUCM_CLKMUX_EXTCLK     (3 << 3)    /**< Clock source is EXTCLK.    */
/** @} */

/**
 * @name    CLKCON1 register bits definitions
 * @{
 */
#define ADUCM_SPI0CD_DIV1       (0 << 0)    /**< SPI0 Clock divided by 1.   */
#define ADUCM_SPI0CD_DIV2       (1 << 0)    /**< SPI0 Clock divided by 2.   */
#define ADUCM_SPI0CD_DIV4       (2 << 0)    /**< SPI0 Clock divided by 4.   */
#define ADUCM_SPI0CD_DIV8       (3 << 0)    /**< SPI0 Clock divided by 8.   */
#define ADUCM_SPI0CD_DIV16      (4 << 0)    /**< SPI0 Clock divided by 16.  */
#define ADUCM_SPI0CD_DIV32      (5 << 0)    /**< SPI0 Clock divided by 32.  */
#define ADUCM_SPI0CD_DIV64      (6 << 0)    /**< SPI0 Clock divided by 64.  */
#define ADUCM_SPI0CD_DIV128     (7 << 0)    /**< SPI0 Clock divided by 128. */

#define ADUCM_SPI1CD_DIV1       (0 << 3)    /**< SPI1 Clock divided by 1.   */
#define ADUCM_SPI1CD_DIV2       (1 << 3)    /**< SPI1 Clock divided by 2.   */
#define ADUCM_SPI1CD_DIV4       (2 << 3)    /**< SPI1 Clock divided by 4.   */
#define ADUCM_SPI1CD_DIV8       (3 << 3)    /**< SPI1 Clock divided by 8.   */
#define ADUCM_SPI1CD_DIV16      (4 << 3)    /**< SPI1 Clock divided by 16.  */
#define ADUCM_SPI1CD_DIV32      (5 << 3)    /**< SPI1 Clock divided by 32.  */
#define ADUCM_SPI1CD_DIV64      (6 << 3)    /**< SPI1 Clock divided by 64.  */
#define ADUCM_SPI1CD_DIV128     (7 << 3)    /**< SPI1 Clock divided by 128. */

#define ADUCM_I2CCD_DIV1        (0 << 6)    /**< I2C Clock divided by 1.    */
#define ADUCM_I2CCD_DIV2        (1 << 6)    /**< I2C Clock divided by 2.    */
#define ADUCM_I2CCD_DIV4        (2 << 6)    /**< I2C Clock divided by 4.    */
#define ADUCM_I2CCD_DIV8        (3 << 6)    /**< I2C Clock divided by 8.    */
#define ADUCM_I2CCD_DIV16       (4 << 6)    /**< I2C Clock divided by 16.   */
#define ADUCM_I2CCD_DIV32       (5 << 6)    /**< I2C Clock divided by 32.   */
#define ADUCM_I2CCD_DIV64       (6 << 6)    /**< I2C Clock divided by 64.   */
#define ADUCM_I2CCD_DIV128      (7 << 6)    /**< I2C Clock divided by 128.  */

#define ADUCM_UARTCD_DIV1       (0 << 9)    /**< UART Clock divided by 1.   */
#define ADUCM_UARTCD_DIV2       (1 << 9)    /**< UART Clock divided by 2.   */
#define ADUCM_UARTCD_DIV4       (2 << 9)    /**< UART Clock divided by 4.   */
#define ADUCM_UARTCD_DIV8       (3 << 9)    /**< UART Clock divided by 8.   */
#define ADUCM_UARTCD_DIV16      (4 << 9)    /**< UART Clock divided by 16.  */
#define ADUCM_UARTCD_DIV32      (5 << 9)    /**< UART Clock divided by 32.  */
#define ADUCM_UARTCD_DIV64      (6 << 9)    /**< UART Clock divided by 64.  */
#define ADUCM_UARTCD_DIV128     (7 << 9)    /**< UART Clock divided by 128. */

#define ADUCM_PWMCD_DIV1        (0 << 12)   /**< PWM Clock divided by 1.    */
#define ADUCM_PWMCD_DIV2        (1 << 12)   /**< PWM Clock divided by 2.    */
#define ADUCM_PWMCD_DIV4        (2 << 12)   /**< PWM Clock divided by 4.    */
#define ADUCM_PWMCD_DIV8        (3 << 12)   /**< PWM Clock divided by 8.    */
#define ADUCM_PWMCD_DIV16       (4 << 12)   /**< PWM Clock divided by 16.   */
#define ADUCM_PWMCD_DIV32       (5 << 12)   /**< PWM Clock divided by 32.   */
#define ADUCM_PWMCD_DIV64       (6 << 12)   /**< PWM Clock divided by 64.   */
#define ADUCM_PWMCD_DIV128      (7 << 12)   /**< PWM Clock divided by 128.  */
/** @} */

/**
 * @name    XOSCOCN register bits definitions
 * @{
 */
#define ADUCM_XOSC_DISABLE      (0 << 0)    /**< EXTOSC disabled.           */
#define ADUCM_XOSC_ENABLE       (1 << 0)    /**< EXTOSC enabled.            */

#define ADUCM_XOSC_DIV1         (0 << 2)    /**< EXTOSC Clock divided by 1. */ 
#define ADUCM_XOSC_DIV2         (1 << 2)    /**< EXTOSC Clock divided by 2. */ 
/** @} */

/**
 * @name    CLKSYSDIV register bits definitions
 * @{
 */
#define ADUCM_HFOSC_DIV1        (0 << 0)    /**< HFOSC Clock divided by 1.  */ 
#define ADUCM_HFOSC_DIV2        (1 << 0)    /**< HFOSC Clock divided by 2.  */ 
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the CA initialization in the HAL.
 */
#if !defined(ADUCM_NO_INIT) || defined(__DOXYGEN__)
#define ADUCM_NO_INIT                       FALSE
#endif

/**
 * @brief   Enables or disables the XOSC clock source.
 */
#if !defined(ADUCM_XOSC_ENABLED) || defined(__DOXYGEN__)
#define ADUCM_XOSC_ENABLED                  FALSE
#endif

/**
 * @brief   Main clock source selection.
 */
#if !defined(ADUCM_CLKMUX) || defined(__DOXYGEN__)
#define ADUCM_CLKMUX                        ADUCM_CLKMUX_HFOSC
#endif

/**
 * @brief   Internal High Speed oscillator clock source pre-divider.
 * @note    This setting has only effect if the HFOSC is selected as the
 *          system clock source.
 */
#if !defined(ADUCM_HFOSC_PREDIV) || defined(__DOXYGEN__)
#define ADUCM_HFOSC_PREDIV                  ADUCM_HFOSC_DIV1
#endif

/**
 * @brief   External oscillator clock source pre-divider.
 * @note    This setting has only effect if the LFXTAL is selected as the
 *          system clock source.
 */
#if !defined(ADUCM_XOSC_PREDIV) || defined(__DOXYGEN__)
#define ADUCM_XOSC_PREDIV                   ADUCM_XOSC_DIV1
#endif

/**
 * @brief   Main clock divider.
 */
#if !defined(ADUCM_CD_DIV) || defined(__DOXYGEN__)
#define ADUCM_CD_DIV                        ADUCM_CD_DIV1
#endif
/** @} */

/**
 * @brief   SPI0 clock divider.
 */
#if !defined(ADUCM_SPI0CD_DIV) || defined(__DOXYGEN__)
#define ADUCM_SPI0CD_DIV                    ADUCM_SPI0CD_DIV1
#endif
/** @} */

/**
 * @brief   SPI1 clock divider.
 */
#if !defined(ADUCM_SPI1CD_DIV) || defined(__DOXYGEN__)
#define ADUCM_SPI1CD_DIV                    ADUCM_SPI1CD_DIV1
#endif
/** @} */

/**
 * @brief   I2C clock divider.
 */
#if !defined(ADUCM_I2CCD_DIV) || defined(__DOXYGEN__)
#define ADUCM_I2CCD_DIV                     ADUCM_I2CCD_DIV1
#endif
/** @} */

/**
 * @brief   UART clock divider.
 */
#if !defined(ADUCM_UARTCD_DIV) || defined(__DOXYGEN__)
#define ADUCM_UARTCD_DIV                    ADUCM_UARTCD_DIV1
#endif
/** @} */

/**
 * @brief   PWM clock divider.
 */
#if !defined(ADUCM_PWMCD_DIV) || defined(__DOXYGEN__)
#define ADUCM_PWMCD_DIV                     ADUCM_PWMCD_DIV1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(ADUCM36x_MCUCONF)
#error "Using a wrong mcuconf.h file, ADUCM36x_MCUCONF not defined"
#endif

/*
 * LFXTAL related checks.
 */
#if ADUCM_CLKMUX == ADUCM_CLKMUX_LFXTAL
#define ADUCM_XOSC_REQUIRED         TRUE
#if (ADUCM_XOSC_ENABLED == FALSE)
#error "LFXTAL not enabled, required by ADUCM_CLKMUX"
#endif
#if (ADUCM_LFXTAL != ADUCM_LFXTAL_MAX)
#error "LFXTAL wrong frequency or not defined"
#endif
#else
#define ADUCM_XOSC_REQUIRED         FALSE
#endif

/**
 * @brief   UCLK source.
 */
#if (ADUCM_CLKMUX == ADUCM_CLKMUX_HFOSC) || defined(__DOXYGEN__)
#if (ADUCM_HFOSC_PREDIV == ADUCM_HFOSC_DIV1) || defined(__DOXYGEN__)
#define ADUCM_UCLK                  (ADUCM_HFOSC / 1)
#elif (ADUCM_HFOSC_PREDIV == ADUCM_HFOSC_DIV1)
#define ADUCM_UCLK                  (ADUCM_HFOSC / 2)
#else
#error "wrong HFOSC divider"
#endif
#elif ADUCM_CLKMUX == ADUCM_CLKMUX_LFXTAL
#if (ADUCM_XOSC_PREDIV == ADUCM_XOSC_DIV1)
#define ADUCM_UCLK                  (ADUCM_LFXTAL / 1)
#elif (ADUCM_XOSC_PREDIV == ADUCM_XOSC_DIV1)
#define ADUCM_UCLK                  (ADUCM_LFXTAL / 2)
#else
#error "wrong XOSC divider"
#endif
#elif ADUCM_CLKMUX == ADUCM_CLKMUX_LFOSC
#define ADUCM_UCLK                  (ADUCM_LFOSC / 1)
#elif ADUCM_CLKMUX == ADUCM_CLKMUX_EXTCLK
#error "external clock currently unsupported"
#else
#error "invalid ADUCM_CLKMUX value specified"
#endif

/**
 * @brief   FCLK, HCLK, PCLK frequency.
 */
#if (ADUCM_CD_DIV == ADUCM_CD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_FCLK                  (ADUCM_UCLK / 1)
#define ADUCM_HCLK                  (ADUCM_UCLK / 1)
#define ADUCM_PCLK                  (ADUCM_UCLK / 1)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV2
#define ADUCM_FCLK                  (ADUCM_UCLK / 2)
#define ADUCM_HCLK                  (ADUCM_UCLK / 2)
#define ADUCM_PCLK                  (ADUCM_UCLK / 2)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV4
#define ADUCM_FCLK                  (ADUCM_UCLK / 4)
#define ADUCM_HCLK                  (ADUCM_UCLK / 4)
#define ADUCM_PCLK                  (ADUCM_UCLK / 4)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV8
#define ADUCM_FCLK                  (ADUCM_UCLK / 8)
#define ADUCM_HCLK                  (ADUCM_UCLK / 8)
#define ADUCM_PCLK                  (ADUCM_UCLK / 8)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV16
#define ADUCM_FCLK                  (ADUCM_UCLK / 16)
#define ADUCM_HCLK                  (ADUCM_UCLK / 16)
#define ADUCM_PCLK                  (ADUCM_UCLK / 16)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV32
#define ADUCM_FCLK                  (ADUCM_UCLK / 32)
#define ADUCM_HCLK                  (ADUCM_UCLK / 32)
#define ADUCM_PCLK                  (ADUCM_UCLK / 32)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV64
#define ADUCM_FCLK                  (ADUCM_UCLK / 64)
#define ADUCM_HCLK                  (ADUCM_UCLK / 64)
#define ADUCM_PCLK                  (ADUCM_UCLK / 64)
#elif ADUCM_CD_DIV == ADUCM_CD_DIV128
#define ADUCM_FCLK                  (ADUCM_UCLK / 128)
#define ADUCM_HCLK                  (ADUCM_UCLK / 128)
#define ADUCM_PCLK                  (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_CD_DIV value specified"
#endif

/**
 * @brief   SPI0 frequency.
 */
#if (ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 1)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV2
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 2)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV4
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 4)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV8
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 8)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV16
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 16)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV32
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 32)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV64
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 64)
#elif ADUCM_SPI0CD_DIV == ADUCM_SPI0CD_DIV128
#define ADUCM_SPI0CLK               (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_SPI0CD_DIV value specified"
#endif

/**
 * @brief   SPI1 frequency.
 */
#if (ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 1)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV2
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 2)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV4
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 4)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV8
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 8)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV16
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 16)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV32
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 32)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV64
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 64)
#elif ADUCM_SPI1CD_DIV == ADUCM_SPI1CD_DIV128
#define ADUCM_SPI1CLK               (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_SPI1CD_DIV value specified"
#endif

/**
 * @brief   I2C frequency.
 */
#if (ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 1)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV2
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 2)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV4
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 4)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV8
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 8)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV16
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 16)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV32
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 32)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV64
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 64)
#elif ADUCM_I2CCD_DIV == ADUCM_I2CCD_DIV128
#define ADUCM_I2C0CLK               (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_I2CCD_DIV value specified"
#endif

/**
 * @brief   UART frequency.
 */
#if (ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_UART0CLK              (ADUCM_UCLK / 1)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV2
#define ADUCM_UART0CLK              (ADUCM_UCLK / 2)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV4
#define ADUCM_UART0CLK              (ADUCM_UCLK / 4)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV8
#define ADUCM_UART0CLK              (ADUCM_UCLK / 8)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV16
#define ADUCM_UART0CLK              (ADUCM_UCLK / 16)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV32
#define ADUCM_UART0CLK              (ADUCM_UCLK / 32)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV64
#define ADUCM_UART0CLK              (ADUCM_UCLK / 64)
#elif ADUCM_UARTCD_DIV == ADUCM_UARTCD_DIV128
#define ADUCM_UART0CLK              (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_UARTCD_DIV value specified"
#endif

/**
 * @brief   PWM frequency.
 */
#if (ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV1) || defined(__DOXYGEN__)
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 1)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV2
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 2)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV4
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 4)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV8
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 8)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV16
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 16)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV32
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 32)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV64
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 64)
#elif ADUCM_PWMCD_DIV == ADUCM_PWMCD_DIV128
#define ADUCM_PWM0CLK               (ADUCM_UCLK / 128)
#else
#error "invalid ADUCM_PWMCD_DIV value specified"
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

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "aducm_isr.h"
#include "aducm_cc.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void aducm_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
