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
 * @file    STM32F7xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32f7xx.h.
 *
 * @addtogroup STM32F7xx_RCC
 * @{
 */
#ifndef STM32_RCC_H
#define STM32_RCC_H

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
 * @name    Generic RCC operations
 * @{
 */
/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1(mask, lp) {                                           \
  RCC->APB1ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APB1LPENR |= (mask);                                               \
  else                                                                      \
    RCC->APB1LPENR &= ~(mask);                                              \
  (void)RCC->APB1LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1(mask) {                                              \
  RCC->APB1ENR &= ~(mask);                                                  \
  RCC->APB1LPENR &= ~(mask);                                                \
  (void)RCC->APB1LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rccResetAPB1(mask) {                                                \
  RCC->APB1RSTR |= (mask);                                                  \
  RCC->APB1RSTR &= ~(mask);                                                 \
  (void)RCC->APB1RSTR;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB2(mask, lp) {                                           \
  RCC->APB2ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APB2LPENR |= (mask);                                               \
  else                                                                      \
    RCC->APB2LPENR &= ~(mask);                                              \
  (void)RCC->APB2LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB2(mask) {                                              \
  RCC->APB2ENR &= ~(mask);                                                  \
  RCC->APB2LPENR &= ~(mask);                                                \
  (void)RCC->APB2LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccResetAPB2(mask) {                                                \
  RCC->APB2RSTR |= (mask);                                                  \
  RCC->APB2RSTR &= ~(mask);                                                 \
  (void)RCC->APB2RSTR;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB1 bus.
 *
 * @param[in] mask      AHB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB1(mask, lp) {                                           \
  RCC->AHB1ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->AHB1LPENR |= (mask);                                               \
  else                                                                      \
    RCC->AHB1LPENR &= ~(mask);                                              \
  (void)RCC->AHB1LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB1 bus.
 *
 * @param[in] mask      AHB1 peripherals mask
 *
 * @api
 */
#define rccDisableAHB1(mask) {                                              \
  RCC->AHB1ENR &= ~(mask);                                                  \
  RCC->AHB1LPENR &= ~(mask);                                                \
  (void)RCC->AHB1LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the AHB1 bus.
 *
 * @param[in] mask      AHB1 peripherals mask
 *
 * @api
 */
#define rccResetAHB1(mask) {                                                \
  RCC->AHB1RSTR |= (mask);                                                  \
  RCC->AHB1RSTR &= ~(mask);                                                 \
  (void)RCC->AHB1RSTR;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB2 bus.
 *
 * @param[in] mask      AHB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB2(mask, lp) {                                           \
  RCC->AHB2ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->AHB2LPENR |= (mask);                                               \
  else                                                                      \
    RCC->AHB2LPENR &= ~(mask);                                              \
  (void)RCC->AHB2LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB2 bus.
 *
 * @param[in] mask      AHB2 peripherals mask
 *
 * @api
 */
#define rccDisableAHB2(mask) {                                              \
  RCC->AHB2ENR &= ~(mask);                                                  \
  RCC->AHB2LPENR &= ~(mask);                                                \
  (void)RCC->AHB2LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the AHB2 bus.
 *
 * @param[in] mask      AHB2 peripherals mask
 *
 * @api
 */
#define rccResetAHB2(mask) {                                                \
  RCC->AHB2RSTR |= (mask);                                                  \
  RCC->AHB2RSTR &= ~(mask);                                                 \
  (void)RCC->AHB2RSTR;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB3 (FSMC) bus.
 *
 * @param[in] mask      AHB3 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB3(mask, lp) {                                           \
  RCC->AHB3ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->AHB3LPENR |= (mask);                                               \
  else                                                                      \
    RCC->AHB3LPENR &= ~(mask);                                              \
  (void)RCC->AHB3LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB3 (FSMC) bus.
 *
 * @param[in] mask      AHB3 peripherals mask
 *
 * @api
 */
#define rccDisableAHB3(mask) {                                              \
  RCC->AHB3ENR &= ~(mask);                                                  \
  RCC->AHB3LPENR &= ~(mask);                                                \
  (void)RCC->AHB3LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the AHB3 (FSMC) bus.
 *
 * @param[in] mask      AHB3 peripherals mask
 *
 * @api
 */
#define rccResetAHB3(mask) {                                                \
  RCC->AHB3RSTR |= (mask);                                                  \
  RCC->AHB3RSTR &= ~(mask);                                                 \
  (void)RCC->AHB3RSTR;                                                      \
}
/** @} */

/**
 * @name    ADC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Resets ADC peripherals.
 *
 * @api
 */
#define rccResetADC() rccResetAPB2(RCC_APB2RSTR_ADCRST)

/**
 * @brief   Enables the ADC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC1(lp) rccEnableAPB2(RCC_APB2ENR_ADC1EN, lp)

/**
 * @brief   Disables the ADC1 peripheral clock.
 *
 * @api
 */
#define rccDisableADC1() rccDisableAPB2(RCC_APB2ENR_ADC1EN)

/**
 * @brief   Enables the ADC2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC2(lp) rccEnableAPB2(RCC_APB2ENR_ADC2EN, lp)

/**
 * @brief   Disables the ADC2 peripheral clock.
 *
 * @api
 */
#define rccDisableADC2() rccDisableAPB2(RCC_APB2ENR_ADC2EN)

/**
 * @brief   Enables the ADC3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC3(lp) rccEnableAPB2(RCC_APB2ENR_ADC3EN, lp)

/**
 * @brief   Disables the ADC3 peripheral clock.
 *
 * @api
 */
#define rccDisableADC3() rccDisableAPB2(RCC_APB2ENR_ADC3EN)
/** @} */

/**
 * @name    DAC peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DAC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDAC1(lp) rccEnableAPB1(RCC_APB1ENR_DACEN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC1() rccDisableAPB1(RCC_APB1ENR_DACEN)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAPB1(RCC_APB1RSTR_DACRST)
/** @} */

/**
 * @name    DMA peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DMA1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA1(lp) rccEnableAHB1(RCC_AHB1ENR_DMA1EN, lp)

/**
 * @brief   Disables the DMA1 peripheral clock.
 *
 * @api
 */
#define rccDisableDMA1() rccDisableAHB1(RCC_AHB1ENR_DMA1EN)

/**
 * @brief   Resets the DMA1 peripheral.
 *
 * @api
 */
#define rccResetDMA1() rccResetAHB1(RCC_AHB1RSTR_DMA1RST)

/**
 * @brief   Enables the DMA2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA2(lp) rccEnableAHB1(RCC_AHB1ENR_DMA2EN, lp)

/**
 * @brief   Disables the DMA2 peripheral clock.
 *
 * @api
 */
#define rccDisableDMA2() rccDisableAHB1(RCC_AHB1ENR_DMA2EN)

/**
 * @brief   Resets the DMA2 peripheral.
 *
 * @api
 */
#define rccResetDMA2() rccResetAHB1(RCC_AHB1RSTR_DMA2RST)
/** @} */

/**
 * @name    BKPSRAM specific RCC operations
 * @{
 */
/**
 * @brief   Enables the BKPSRAM peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableBKPSRAM(lp) rccEnableAHB1(RCC_AHB1ENR_BKPSRAMEN, lp)

/**
 * @brief   Disables the BKPSRAM peripheral clock.
 *
 * @api
 */
#define rccDisableBKPSRAM() rccDisableAHB1(RCC_AHB1ENR_BKPSRAMEN)
/** @} */

/**
 * @name    PWR interface specific RCC operations
 * @{
 */
/**
 * @brief   Enables the PWR interface clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnablePWRInterface(lp) rccEnableAPB1(RCC_APB1ENR_PWREN, lp)

/**
 * @brief   Disables PWR interface clock.
 *
 * @api
 */
#define rccDisablePWRInterface() rccDisableAPB1(RCC_APB1ENR_PWREN)

/**
 * @brief   Resets the PWR interface.
 *
 * @api
 */
#define rccResetPWRInterface() rccResetAPB1(RCC_APB1RSTR_PWRRST)
/** @} */

/**
 * @name    CAN peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the CAN1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableCAN1(lp) rccEnableAPB1(RCC_APB1ENR_CAN1EN, lp)

/**
 * @brief   Disables the CAN1 peripheral clock.
 *
 * @api
 */
#define rccDisableCAN1() rccDisableAPB1(RCC_APB1ENR_CAN1EN)

/**
 * @brief   Resets the CAN1 peripheral.
 *
 * @api
 */
#define rccResetCAN1() rccResetAPB1(RCC_APB1RSTR_CAN1RST)

/**
 * @brief   Enables the CAN2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableCAN2(lp) rccEnableAPB1(RCC_APB1ENR_CAN2EN, lp)

/**
 * @brief   Disables the CAN2 peripheral clock.
 *
 * @api
 */
#define rccDisableCAN2() rccDisableAPB1(RCC_APB1ENR_CAN2EN)

/**
 * @brief   Resets the CAN2 peripheral.
 *
 * @api
 */
#define rccResetCAN2() rccResetAPB1(RCC_APB1RSTR_CAN2RST)

/**
 * @brief   Resets the CAN3 peripheral.
 *
 * @api
 */
#define rccResetCAN3() rccResetAPB1(RCC_APB1RSTR_CAN3RST)

/**
 * @brief   Enables the CAN3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableCAN3(lp) rccEnableAPB1(RCC_APB1ENR_CAN3EN, lp)

/**
 * @brief   Disables the CAN3 peripheral clock.
 *
 * @api
 */
#define rccDisableCAN3() rccDisableAPB1(RCC_APB1ENR_CAN3EN)
/** @} */

/**
 * @name    ETH peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the ETH peripheral clock.
 *
 * @api
 */
#define rccEnableETH(lp) rccEnableAHB1(RCC_AHB1ENR_ETHMACEN |               \
                                       RCC_AHB1ENR_ETHMACTXEN |             \
                                       RCC_AHB1ENR_ETHMACRXEN, lp)

/**
 * @brief   Disables the ETH peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableETH() rccDisableAHB1(RCC_AHB1ENR_ETHMACEN |               \
                                       RCC_AHB1ENR_ETHMACTXEN |             \
                                       RCC_AHB1ENR_ETHMACRXEN)

/**
 * @brief   Resets the ETH peripheral.
 *
 * @api
 */
#define rccResetETH() rccResetAHB1(RCC_AHB1RSTR_ETHMACRST)
/** @} */

/**
 * @name    I2C peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the I2C1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C1(lp) rccEnableAPB1(RCC_APB1ENR_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C1() rccDisableAPB1(RCC_APB1ENR_I2C1EN)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rccResetI2C1() rccResetAPB1(RCC_APB1RSTR_I2C1RST)

/**
 * @brief   Enables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C2(lp) rccEnableAPB1(RCC_APB1ENR_I2C2EN, lp)

/**
 * @brief   Disables the I2C2 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C2() rccDisableAPB1(RCC_APB1ENR_I2C2EN)

/**
 * @brief   Resets the I2C2 peripheral.
 *
 * @api
 */
#define rccResetI2C2() rccResetAPB1(RCC_APB1RSTR_I2C2RST)

/**
 * @brief   Enables the I2C3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C3(lp) rccEnableAPB1(RCC_APB1ENR_I2C3EN, lp)

/**
 * @brief   Disables the I2C3 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C3() rccDisableAPB1(RCC_APB1ENR_I2C3EN)

/**
 * @brief   Resets the I2C3 peripheral.
 *
 * @api
 */
#define rccResetI2C3() rccResetAPB1(RCC_APB1RSTR_I2C3RST)

/**
 * @brief   Enables the I2C4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C4(lp) rccEnableAPB1(RCC_APB1ENR_I2C4EN, lp)

/**
 * @brief   Disables the I2C4 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C4() rccDisableAPB1(RCC_APB1ENR_I2C4EN)

/**
 * @brief   Resets the I2C4 peripheral.
 *
 * @api
 */
#define rccResetI2C4() rccResetAPB1(RCC_APB1RSTR_I2C4RST)
/** @} */

/**
 * @name    OTG peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the OTG_FS peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableOTG_FS(lp) rccEnableAHB2(RCC_AHB2ENR_OTGFSEN, lp)

/**
 * @brief   Disables the OTG_FS peripheral clock.
 *
 * @api
 */
#define rccDisableOTG_FS() rccDisableAHB2(RCC_AHB2ENR_OTGFSEN)

/**
 * @brief   Resets the OTG_FS peripheral.
 *
 * @api
 */
#define rccResetOTG_FS() rccResetAHB2(RCC_AHB2RSTR_OTGFSRST)

/**
 * @brief   Enables the OTG_HS peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableOTG_HS(lp) rccEnableAHB1(RCC_AHB1ENR_OTGHSEN, lp)

/**
 * @brief   Disables the OTG_HS peripheral clock.
 *
 * @api
 */
#define rccDisableOTG_HS() rccDisableAHB1(RCC_AHB1ENR_OTGHSEN)

/**
 * @brief   Resets the OTG_HS peripheral.
 *
 * @api
 */
#define rccResetOTG_HS() rccResetAHB1(RCC_AHB1RSTR_OTGHRST)

/**
 * @brief   Enables the OTG_HS peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableOTG_HSULPI(lp) rccEnableAHB1(RCC_AHB1ENR_OTGHSULPIEN, lp)

/**
 * @brief   Disables the OTG_HS peripheral clock.
 *
 * @api
 */
#define rccDisableOTG_HSULPI() rccDisableAHB1(RCC_AHB1ENR_OTGHSULPIEN)
/** @} */

/**
 * @name    QUADSPI peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the QUADSPI1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableQUADSPI1(lp) rccEnableAHB3(RCC_AHB3ENR_QSPIEN, lp)

/**
 * @brief   Disables the QUADSPI1 peripheral clock.
 *
 * @api
 */
#define rccDisableQUADSPI1() rccDisableAHB3(RCC_AHB3ENR_QSPIEN)

/**
 * @brief   Resets the QUADSPI1 peripheral.
 *
 * @api
 */
#define rccResetQUADSPI1() rccResetAHB3(RCC_AHB3RSTR_QSPIRST)
/** @} */

/**
 * @name    RNG peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the RNG peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableRNG(lp) rccEnableAHB2(RCC_AHB2ENR_RNGEN, lp)

/**
 * @brief   Disables the RNG peripheral clock.
 *
 * @api
 */
#define rccDisableRNG() rccDisableAHB2(RCC_AHB2ENR_RNGEN)

/**
 * @brief   Resets the RNG peripheral.
 *
 * @api
 */
#define rccResetRNG() rccResetAHB2(RCC_AHB2RSTR_RNGRST)
/** @} */

/**
 * @name    SDMMC peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SDMMC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSDMMC1(lp) rccEnableAPB2(RCC_APB2ENR_SDMMC1EN, lp)

/**
 * @brief   Disables the SDMMC1 peripheral clock.
 *
 * @api
 */
#define rccDisableSDMMC1() rccDisableAPB2(RCC_APB2ENR_SDMMC1EN)

/**
 * @brief   Resets the SDMMC1 peripheral.
 *
 * @api
 */
#define rccResetSDMMC1() rccResetAPB2(RCC_APB2RSTR_SDMMC1RST)

/**
 * @brief   Enables the SDMMC2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSDMMC2(lp) rccEnableAPB2(RCC_APB2ENR_SDMMC2EN, lp)

/**
 * @brief   Disables the SDMMC2 peripheral clock.
 *
 * @api
 */
#define rccDisableSDMMC2() rccDisableAPB2(RCC_APB2ENR_SDMMC2EN)

/**
 * @brief   Resets the SDMMC2 peripheral.
 *
 * @api
 */
#define rccResetSDMMC2() rccResetAPB2(RCC_APB2RSTR_SDMMC2RST)
/** @} */

/**
 * @name    SPI peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SPI1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI1(lp) rccEnableAPB2(RCC_APB2ENR_SPI1EN, lp)

/**
 * @brief   Disables the SPI1 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI1() rccDisableAPB2(RCC_APB2ENR_SPI1EN)

/**
 * @brief   Resets the SPI1 peripheral.
 *
 * @api
 */
#define rccResetSPI1() rccResetAPB2(RCC_APB2RSTR_SPI1RST)

/**
 * @brief   Enables the SPI2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI2(lp) rccEnableAPB1(RCC_APB1ENR_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI2() rccDisableAPB1(RCC_APB1ENR_SPI2EN)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rccResetSPI2() rccResetAPB1(RCC_APB1RSTR_SPI2RST)

/**
 * @brief   Enables the SPI3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI3(lp) rccEnableAPB1(RCC_APB1ENR_SPI3EN, lp)

/**
 * @brief   Disables the SPI3 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI3() rccDisableAPB1(RCC_APB1ENR_SPI3EN)

/**
 * @brief   Resets the SPI3 peripheral.
 *
 * @api
 */
#define rccResetSPI3() rccResetAPB1(RCC_APB1RSTR_SPI3RST)

/**
 * @brief   Enables the SPI4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI4(lp) rccEnableAPB2(RCC_APB2ENR_SPI4EN, lp)

/**
 * @brief   Disables the SPI4 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI4() rccDisableAPB2(RCC_APB2ENR_SPI4EN)

/**
 * @brief   Resets the SPI4 peripheral.
 *
 * @api
 */
#define rccResetSPI4() rccResetAPB2(RCC_APB2RSTR_SPI4RST)

/**
 * @brief   Enables the SPI5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI5(lp) rccEnableAPB2(RCC_APB2ENR_SPI5EN, lp)

/**
 * @brief   Disables the SPI5 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI5() rccDisableAPB2(RCC_APB2ENR_SPI5EN)

/**
 * @brief   Resets the SPI5 peripheral.
 *
 * @api
 */
#define rccResetSPI5() rccResetAPB2(RCC_APB2RSTR_SPI5RST)

/**
 * @brief   Enables the SPI6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI6(lp) rccEnableAPB2(RCC_APB2ENR_SPI6EN, lp)

/**
 * @brief   Disables the SPI6 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI6() rccDisableAPB2(RCC_APB2ENR_SPI6EN)

/**
 * @brief   Resets the SPI6 peripheral.
 *
 * @api
 */
#define rccResetSPI6() rccResetAPB2(RCC_APB2RSTR_SPI6RST)
/** @} */

/**
 * @name    TIM peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the TIM1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM1(lp) rccEnableAPB2(RCC_APB2ENR_TIM1EN, lp)

/**
 * @brief   Disables the TIM1 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM1() rccDisableAPB2(RCC_APB2ENR_TIM1EN)

/**
 * @brief   Resets the TIM1 peripheral.
 *
 * @api
 */
#define rccResetTIM1() rccResetAPB2(RCC_APB2RSTR_TIM1RST)

/**
 * @brief   Enables the TIM2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM2(lp) rccEnableAPB1(RCC_APB1ENR_TIM2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM2() rccDisableAPB1(RCC_APB1ENR_TIM2EN)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rccResetTIM2() rccResetAPB1(RCC_APB1RSTR_TIM2RST)

/**
 * @brief   Enables the TIM3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM3(lp) rccEnableAPB1(RCC_APB1ENR_TIM3EN, lp)

/**
 * @brief   Disables the TIM3 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM3() rccDisableAPB1(RCC_APB1ENR_TIM3EN)

/**
 * @brief   Resets the TIM3 peripheral.
 *
 * @api
 */
#define rccResetTIM3() rccResetAPB1(RCC_APB1RSTR_TIM3RST)

/**
 * @brief   Enables the TIM4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM4(lp) rccEnableAPB1(RCC_APB1ENR_TIM4EN, lp)

/**
 * @brief   Disables the TIM4 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM4() rccDisableAPB1(RCC_APB1ENR_TIM4EN)

/**
 * @brief   Resets the TIM4 peripheral.
 *
 * @api
 */
#define rccResetTIM4() rccResetAPB1(RCC_APB1RSTR_TIM4RST)

/**
 * @brief   Enables the TIM5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM5(lp) rccEnableAPB1(RCC_APB1ENR_TIM5EN, lp)

/**
 * @brief   Disables the TIM5 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM5() rccDisableAPB1(RCC_APB1ENR_TIM5EN)

/**
 * @brief   Resets the TIM5 peripheral.
 *
 * @api
 */
#define rccResetTIM5() rccResetAPB1(RCC_APB1RSTR_TIM5RST)

/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM6(lp) rccEnableAPB1(RCC_APB1ENR_TIM6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM6() rccDisableAPB1(RCC_APB1ENR_TIM6EN)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rccResetTIM6() rccResetAPB1(RCC_APB1RSTR_TIM6RST)

/**
 * @brief   Enables the TIM7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM7(lp) rccEnableAPB1(RCC_APB1ENR_TIM7EN, lp)

/**
 * @brief   Disables the TIM7 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM7() rccDisableAPB1(RCC_APB1ENR_TIM7EN)

/**
 * @brief   Resets the TIM7 peripheral.
 *
 * @api
 */
#define rccResetTIM7() rccResetAPB1(RCC_APB1RSTR_TIM7RST)

/**
 * @brief   Enables the TIM8 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM8(lp) rccEnableAPB2(RCC_APB2ENR_TIM8EN, lp)

/**
 * @brief   Disables the TIM8 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM8() rccDisableAPB2(RCC_APB2ENR_TIM8EN)

/**
 * @brief   Resets the TIM8 peripheral.
 *
 * @api
 */
#define rccResetTIM8() rccResetAPB2(RCC_APB2RSTR_TIM8RST)

/**
 * @brief   Enables the TIM9 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM9(lp) rccEnableAPB2(RCC_APB2ENR_TIM9EN, lp)

/**
 * @brief   Disables the TIM9 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM9() rccDisableAPB2(RCC_APB2ENR_TIM9EN)

/**
 * @brief   Resets the TIM9 peripheral.
 *
 * @api
 */
#define rccResetTIM9() rccResetAPB2(RCC_APB2RSTR_TIM9RST)

/**
 * @brief   Enables the TIM10 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM10(lp) rccEnableAPB2(RCC_APB2ENR_TIM10EN, lp)

/**
 * @brief   Disables the TIM10 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM10() rccDisableAPB2(RCC_APB2ENR_TIM10EN)

/**
 * @brief   Resets the TIM10 peripheral.
 *
 * @api
 */
#define rccResetTIM10() rccResetAPB2(RCC_APB2RSTR_TIM10RST)

/**
 * @brief   Enables the TIM11 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM11(lp) rccEnableAPB2(RCC_APB2ENR_TIM11EN, lp)

/**
 * @brief   Disables the TIM11 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM11() rccDisableAPB2(RCC_APB2ENR_TIM11EN)

/**
 * @brief   Resets the TIM11 peripheral.
 *
 * @api
 */
#define rccResetTIM11() rccResetAPB2(RCC_APB2RSTR_TIM11RST)

/**
 * @brief   Enables the TIM12 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM12(lp) rccEnableAPB1(RCC_APB1ENR_TIM12EN, lp)

/**
 * @brief   Disables the TIM12 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM12() rccDisableAPB1(RCC_APB1ENR_TIM12EN)

/**
 * @brief   Resets the TIM12 peripheral.
 *
 * @api
 */
#define rccResetTIM12() rccResetAPB1(RCC_APB1RSTR_TIM12RST)

/**
 * @brief   Enables the TIM13 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM13(lp) rccEnableAPB1(RCC_APB1ENR_TIM13EN, lp)

/**
 * @brief   Disables the TIM13 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM13() rccDisableAPB1(RCC_APB1ENR_TIM13EN)

/**
 * @brief   Resets the TIM13 peripheral.
 *
 * @api
 */
#define rccResetTIM13() rccResetAPB1(RCC_APB1RSTR_TIM13RST)

/**
 * @brief   Enables the TIM14 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM14(lp) rccEnableAPB1(RCC_APB1ENR_TIM14EN, lp)

/**
 * @brief   Disables the TIM14 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM14() rccDisableAPB1(RCC_APB1ENR_TIM14EN)

/**
 * @brief   Resets the TIM14 peripheral.
 *
 * @api
 */
#define rccResetTIM14() rccResetAPB1(RCC_APB1RSTR_TIM14RST)
/** @} */

/**
 * @name    USART/UART peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the USART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART1(lp) rccEnableAPB2(RCC_APB2ENR_USART1EN, lp)

/**
 * @brief   Disables the USART1 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART1() rccDisableAPB2(RCC_APB2ENR_USART1EN)

/**
 * @brief   Resets the USART1 peripheral.
 *
 * @api
 */
#define rccResetUSART1() rccResetAPB2(RCC_APB2RSTR_USART1RST)

/**
 * @brief   Enables the USART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART2(lp) rccEnableAPB1(RCC_APB1ENR_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART2() rccDisableAPB1(RCC_APB1ENR_USART2EN)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rccResetUSART2() rccResetAPB1(RCC_APB1RSTR_USART2RST)

/**
 * @brief   Enables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART3(lp) rccEnableAPB1(RCC_APB1ENR_USART3EN, lp)

/**
 * @brief   Disables the USART3 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART3() rccDisableAPB1(RCC_APB1ENR_USART3EN)

/**
 * @brief   Resets the USART3 peripheral.
 *
 * @api
 */
#define rccResetUSART3() rccResetAPB1(RCC_APB1RSTR_USART3RST)

/**
 * @brief   Enables the UART4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART4(lp) rccEnableAPB1(RCC_APB1ENR_UART4EN, lp)

/**
 * @brief   Disables the UART4 peripheral clock.
 *
 * @api
 */
#define rccDisableUART4() rccDisableAPB1(RCC_APB1ENR_UART4EN)

/**
 * @brief   Resets the UART4 peripheral.
 *
 * @api
 */
#define rccResetUART4() rccResetAPB1(RCC_APB1RSTR_UART4RST)

/**
 * @brief   Enables the UART5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART5(lp) rccEnableAPB1(RCC_APB1ENR_UART5EN, lp)

/**
 * @brief   Disables the UART5 peripheral clock.
 *
 * @api
 */
#define rccDisableUART5() rccDisableAPB1(RCC_APB1ENR_UART5EN)

/**
 * @brief   Resets the UART5 peripheral.
 *
 * @api
 */
#define rccResetUART5() rccResetAPB1(RCC_APB1RSTR_UART5RST)

/**
 * @brief   Enables the USART6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART6(lp) rccEnableAPB2(RCC_APB2ENR_USART6EN, lp)

/**
 * @brief   Disables the USART6 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART6() rccDisableAPB2(RCC_APB2ENR_USART6EN)

/**
 * @brief   Resets the USART6 peripheral.
 *
 * @api
 */
#define rccResetUSART6() rccResetAPB2(RCC_APB2RSTR_USART6RST)

/**
 * @brief   Enables the UART7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART7(lp) rccEnableAPB1(RCC_APB1ENR_UART7EN, lp)

/**
 * @brief   Disables the UART7 peripheral clock.
 *
 * @api
 */
#define rccDisableUART7() rccDisableAPB1(RCC_APB1ENR_UART7EN)

/**
 * @brief   Resets the UART7 peripheral.
 *
 * @api
 */
#define rccResetUART7() rccResetAPB1(RCC_APB1RSTR_UART7RST)

/**
 * @brief   Enables the UART8 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART8(lp) rccEnableAPB1(RCC_APB1ENR_UART8EN, lp)

/**
 * @brief   Disables the UART8 peripheral clock.
 *
 * @api
 */
#define rccDisableUART8() rccDisableAPB1(RCC_APB1ENR_UART8EN)

/**
 * @brief   Resets the UART8 peripheral.
 *
 * @api
 */
#define rccResetUART8() rccResetAPB1(RCC_APB1RSTR_UART8RST)
/** @} */

/**
 * @name    LTDC peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the LTDC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableLTDC(lp) rccEnableAPB2(RCC_APB2ENR_LTDCEN, lp)

/**
 * @brief   Disables the LTDC peripheral clock.
 *
 * @api
 */
#define rccDisableLTDC() rccDisableAPB2(RCC_APB2ENR_LTDCEN)

/**
 * @brief   Resets the LTDC peripheral.
 *
 * @api
 */
#define rccResetLTDC() rccResetAPB2(RCC_APB2RSTR_LTDCRST)
/** @} */

/**
 * @name    DMA2D peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DMA2D peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA2D(lp) rccEnableAHB1(RCC_AHB1ENR_DMA2DEN, lp)

/**
 * @brief   Disables the DMA2D peripheral clock.
 *
 * @api
 */
#define rccDisableDMA2D() rccDisableAHB1(RCC_AHB1ENR_DMA2DEN)

/**
 * @brief   Resets the DMA2D peripheral.
 *
 * @api
 */
#define rccResetDMA2D() rccResetAHB1(RCC_AHB1RSTR_DMA2DRST)
/** @} */

/**
 * @name    CRC peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the CRC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableCRC(lp) rccEnableAHB1(RCC_AHB1ENR_CRCEN, lp)

/**
 * @brief   Disables the CRC peripheral clock.
 *
 * @api
 */
#define rccDisableCRC() rccDisableAHB1(RCC_AHB1ENR_CRCEN)

/**
 * @brief   Resets the CRC peripheral.
 *
 * @api
 */
#define rccResetCRC() rccResetAHB1(RCC_AHB1RSTR_CRCRST)
/** @} */

/**
 * @name    HASH peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the CRYP peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableCRYP(lp) rccEnableAHB2(RCC_AHB2ENR_CRYPEN, lp)

/**
 * @brief   Disables the CRYP peripheral clock.
 *
 * @api
 */
#define rccDisableCRYP() rccDisableAHB2(RCC_AHB2ENR_CRYPEN)

/**
 * @brief   Resets the CRYP peripheral.
 *
 * @api
 */
#define rccResetCRYP() rccResetAHB2(RCC_AHB2RSTR_CRYPRST)
/** @} */

/**
 * @name    HASH peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the HASH peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableHASH(lp) rccEnableAHB2(RCC_AHB2ENR_HASHEN, lp)

/**
 * @brief   Disables the HASH peripheral clock.
 *
 * @api
 */
#define rccDisableHASH() rccDisableAHB2(RCC_AHB2ENR_HASHEN)

/**
 * @brief   Resets the HASH peripheral.
 *
 * @api
 */
#define rccResetHASH() rccResetAHB2(RCC_AHB2RSTR_HASHRST)
/** @} */

/**
 * @name    FSMC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the FSMC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#if defined(STM32_FSMC_IS_FMC)
  #define rccEnableFSMC(lp) rccEnableAHB3(RCC_AHB3ENR_FMCEN, lp)
#else
  #define rccEnableFSMC(lp) rccEnableAHB3(RCC_AHB3ENR_FSMCEN, lp)
#endif

/**
 * @brief   Disables the FSMC peripheral clock.
 *
 * @api
 */
#if defined(STM32_FSMC_IS_FMC)
  #define rccDisableFSMC() rccDisableAHB3(RCC_AHB3ENR_FMCEN)
#else
  #define rccDisableFSMC() rccDisableAHB3(RCC_AHB3ENR_FSMCEN)
#endif

/**
 * @brief   Resets the FSMC peripheral.
 *
 * @api
 */
#if defined(STM32_FSMC_IS_FMC)
  #define rccResetFSMC() rccResetAHB3(RCC_AHB3RSTR_FMCRST)
#else
  #define rccResetFSMC() rccResetAHB3(RCC_AHB3RSTR_FSMCRST)
#endif
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

#endif /* STM32_RCC_H */

/** @} */
