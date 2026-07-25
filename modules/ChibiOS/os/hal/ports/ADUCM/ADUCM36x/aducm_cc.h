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
 * @file    ADUCM36x/aducm_cc.h
 * @brief   CC helper driver header.
 * @note    This file requires definitions from the ADI header file
 *          @p ADuCM36x.h.
 *
 * @addtogroup ADUCM36x_CC
 * @{
 */

#ifndef ADUCM_CC_H
#define ADUCM_CC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

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

/**
 * @name    Generic CC operations
 * @{
 */
/**
 * @brief   Enables the clock of one or more peripheral.
 *
 * @param[in] mask      CLKDIS peripherals mask
 *
 * @api
 */
#define ccEnableClk(mask) {                                                 \
  pADI_CLKCTL->CLKDIS &= ~(mask);                                           \
  (void)pADI_CLKCTL->CLKDIS;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB bus.
 *
 * @param[in] mask      CLKDIS peripherals mask
 *
 * @api
 */
#define ccDisableClk(mask) {                                                \
  pADI_CLKCTL->CLKDIS |= (mask);                                            \
  (void)pADI_CLKCTL->CLKDIS;                                                \
}
/** @} */

/**
 * @name    SPI peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the SPI0 peripheral clock.
 *
 * @api
 */
#define ccEnableSPI0() ccEnableClk(CLKDIS_DISSPI0CLK)

/**
 * @brief   Disables the SPI0 peripheral clock.
 *
 * @api
 */
#define ccDisableSPI0() ccDisableClk(CLKDIS_DISSPI0CLK)

/**
 * @brief   Enables the SPI1 peripheral clock.
 *
 * @api
 */
#define ccEnableSPI1() ccEnableClk(CLKDIS_DISSPI1CLK)

/**
 * @brief   Disables the SPI1 peripheral clock.
 *
 */
#define ccDisableSPI1() ccDisableClk(CLKDIS_DISSPI1CLK)
/** @} */

/**
 * @name    UART peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the UART0 peripheral clock.
 *
 * @api
 */
#define ccEnableUART0() ccEnableClk(CLKDIS_DISUARTCLK)

/**
 * @brief   Disables the UART0 peripheral clock.
 *
 * @api
 */
#define ccDisableUART0() ccDisableClk(CLKDIS_DISUARTCLK)
/** @} */

/**
 * @name    PWM peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the PWM0 peripheral clock.
 *
 * @api
 */
#define ccEnablePWM0() ccEnableClk(CLKDIS_DISPWMCLK)

/**
 * @brief   Disables the PWM0 peripheral clock.
 *
 * @api
 */
#define ccDisablePWM0() ccDisableClk(CLKDIS_DISPWMCLK)
/** @} */

/**
 * @name    T0 peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the T0 peripheral clock.
 *
 * @api
 */
#define ccEnableT0() ccEnableClk(CLKDIS_DIST0CLK)

/**
 * @brief   Disables the T0 peripheral clock.
 *
 * @api
 */
#define ccDisableT0() ccDisableClk(CLKDIS_DIST0CLK)

/**
 * @brief   Enables the T1 peripheral clock.
 *
 * @api
 */
#define ccEnableT1() ccEnableClk(CLKDIS_DIST1CLK)

/**
 * @brief   Disables the T1 peripheral clock.
 *
 * @api
 */
#define ccDisableT1() ccDisableClk(CLKDIS_DIST1CLK)
/** @} */

/**
 * @name    DAC peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the DAC0 peripheral clock.
 *
 * @api
 */
#define ccEnableDAC0() ccEnableClk(CLKDIS_DISDACCLK)

/**
 * @brief   Disables the DAC0 peripheral clock.
 *
 * @api
 */
#define ccDisableDAC0() ccDisableClk(CLKDIS_DISDACCLK)
/** @} */

/**
 * @name    DMA peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the DMA0 peripheral clock.
 *
 * @api
 */
#define ccEnableDMA0() ccEnableClk(CLKDIS_DISDMACLK)

/**
 * @brief   Disables the DMA0 peripheral clock.
 *
 * @api
 */
#define ccDisableDMA0() ccDisableClk(CLKDIS_DISDMACLK)
/** @} */

/**
 * @name    ADC peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the ADCs peripheral clock.
 *
 * @api
 */
#define ccEnableADCs() ccEnableClk(CLKDIS_DISADCCLK)

/**
 * @brief   Disables the ADCs peripheral clock.
 *
 * @api
 */
#define ccDisableADCs() ccDisableClk(CLKDIS_DISADCCLK)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* ADUCM_CC_H */

/** @} */
