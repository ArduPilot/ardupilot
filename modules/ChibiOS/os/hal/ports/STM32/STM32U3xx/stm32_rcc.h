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
 * @file    STM32U3xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32u3xx.h.
 *
 * @addtogroup STM32U3xx_RCC
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
 * @brief   Enables the clock of one or more peripheral on the APB1 bus (R1).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1R1(mask, lp) {                                         \
  RCC->APB1ENR1 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->APB1SLPENR1 |= (mask);                                             \
  else                                                                      \
    RCC->APB1SLPENR1 &= ~(mask);                                            \
  (void)RCC->APB1SLPENR1;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (R1).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1R1(mask) {                                            \
  RCC->APB1ENR1 &= ~(mask);                                                 \
  RCC->APB1SLPENR1 &= ~(mask);                                              \
  (void)RCC->APB1SLPENR1;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus (R1).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 *
 * @api
 */
#define rccResetAPB1R1(mask) {                                              \
  RCC->APB1RSTR1 |= (mask);                                                 \
  RCC->APB1RSTR1 &= ~(mask);                                                \
  (void)RCC->APB1RSTR1;                                                     \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus (R2).
 *
 * @param[in] mask      APB1 R2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1R2(mask, lp) {                                         \
  RCC->APB1ENR2 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->APB1SLPENR2 |= (mask);                                             \
  else                                                                      \
    RCC->APB1SLPENR2 &= ~(mask);                                            \
  (void)RCC->APB1SLPENR2;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (R2).
 *
 * @param[in] mask      APB1 R2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1R2(mask) {                                            \
  RCC->APB1ENR2 &= ~(mask);                                                 \
  RCC->APB1SLPENR2 &= ~(mask);                                              \
  (void)RCC->APB1SLPENR2;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus (R2).
 *
 * @param[in] mask      APB1 R2 peripherals mask
 *
 * @api
 */
#define rccResetAPB1R2(mask) {                                              \
  RCC->APB1RSTR2 |= (mask);                                                 \
  RCC->APB1RSTR2 &= ~(mask);                                                \
  (void)RCC->APB1RSTR2;                                                     \
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
    RCC->APB2SLPENR |= (mask);                                              \
  else                                                                      \
    RCC->APB2SLPENR &= ~(mask);                                             \
  (void)RCC->APB2SLPENR;                                                    \
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
  RCC->APB2SLPENR &= ~(mask);                                               \
  (void)RCC->APB2SLPENR;                                                    \
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
 * @brief   Enables the clock of one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB3 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB3(mask, lp) {                                           \
  RCC->APB3ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APB3SLPENR |= (mask);                                              \
  else                                                                      \
    RCC->APB3SLPENR &= ~(mask);                                             \
  (void)RCC->APB3SLPENR;                                                    \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB3(mask) {                                              \
  RCC->APB3ENR &= ~(mask);                                                  \
  RCC->APB3SLPENR &= ~(mask);                                               \
  (void)RCC->APB3SLPENR;                                                    \
}

/**
 * @brief   Resets one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB3 peripherals mask
 *
 * @api
 */
#define rccResetAPB3(mask) {                                                \
  RCC->APB3RSTR |= (mask);                                                  \
  RCC->APB3RSTR &= ~(mask);                                                 \
  (void)RCC->APB3RSTR;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB1 bus (R1).
 *
 * @param[in] mask      AHB1 R1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB1R1(mask, lp) {                                         \
  RCC->AHB1ENR1 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->AHB1SLPENR1 |= (mask);                                             \
  else                                                                      \
    RCC->AHB1SLPENR1 &= ~(mask);                                            \
  (void)RCC->AHB1SLPENR1;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB1 bus (R1).
 *
 * @param[in] mask      AHB1 R1 peripherals mask
 *
 * @api
 */
#define rccDisableAHB1R1(mask) {                                            \
  RCC->AHB1ENR1 &= ~(mask);                                                 \
  RCC->AHB1SLPENR1 &= ~(mask);                                              \
  (void)RCC->AHB1SLPENR1;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the AHB1 bus (R1).
 *
 * @param[in] mask      AHB1 R1 peripherals mask
 *
 * @api
 */
#define rccResetAHB1R1(mask) {                                              \
  RCC->AHB1RSTR1 |= (mask);                                                 \
  RCC->AHB1RSTR1 &= ~(mask);                                                \
  (void)RCC->AHB1RSTR1;                                                     \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB1 bus (R2).
 *
 * @param[in] mask      AHB1 R2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB1R2(mask, lp) {                                         \
  RCC->AHB1ENR2 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->AHB1SLPENR2 |= (mask);                                             \
  else                                                                      \
    RCC->AHB1SLPENR2 &= ~(mask);                                            \
  (void)RCC->AHB1SLPENR2;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB1 bus (R2).
 *
 * @param[in] mask      AHB1 R2 peripherals mask
 *
 * @api
 */
#define rccDisableAHB1R2(mask) {                                            \
  RCC->AHB1ENR2 &= ~(mask);                                                 \
  RCC->AHB1SLPENR2 &= ~(mask);                                              \
  (void)RCC->AHB1SLPENR2;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the AHB1 bus (R2).
 *
 * @param[in] mask      AHB1 R2 peripherals mask
 *
 * @api
 */
#define rccResetAHB1R2(mask)

/**
 * @brief   Enables the clock of one or more peripheral on the AHB2 bus (R1).
 *
 * @param[in] mask      AHB2 R1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB2R1(mask, lp) {                                         \
  RCC->AHB2ENR1 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->AHB2SLPENR1 |= (mask);                                             \
  else                                                                      \
    RCC->AHB2SLPENR1 &= ~(mask);                                            \
  (void)RCC->AHB2SLPENR1;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB2 bus (R1).
 *
 * @param[in] mask      AHB2 R1 peripherals mask
 *
 * @api
 */
#define rccDisableAHB2R1(mask) {                                            \
  RCC->AHB2ENR1 &= ~(mask);                                                 \
  RCC->AHB2SLPENR1 &= ~(mask);                                              \
  (void)RCC->AHB2SLPENR1;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the AHB2 bus (R1).
 *
 * @param[in] mask      AHB2 R1 peripherals mask
 *
 * @api
 */
#define rccResetAHB2R1(mask) {                                              \
  RCC->AHB2RSTR1 |= (mask);                                                 \
  RCC->AHB2RSTR1 &= ~(mask);                                                \
  (void)RCC->AHB2RSTR1;                                                     \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB2 bus (R2).
 *
 * @param[in] mask      AHB2 R2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB2R2(mask, lp) {                                         \
  RCC->AHB2ENR2 |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->AHB2SLPENR2 |= (mask);                                             \
  else                                                                      \
    RCC->AHB2SLPENR2 &= ~(mask);                                            \
  (void)RCC->AHB2SLPENR2;                                                   \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB2 bus (R2).
 *
 * @param[in] mask      AHB2 R2 peripherals mask
 *
 * @api
 */
#define rccDisableAHB2R2(mask) {                                            \
  RCC->AHB2ENR2 &= ~(mask);                                                 \
  RCC->AHB2SLPENR21 &= ~(mask);                                              \
  (void)RCC->AHB2SLPENR2;                                                   \
}

/**
 * @brief   Resets one or more peripheral on the AHB2 bus (R2).
 *
 * @param[in] mask      AHB2 R2 peripherals mask
 *
 * @api
 */
#define rccResetAHB2R2(mask) {                                              \
  RCC->AHB2RSTR2 |= (mask);                                                 \
  RCC->AHB2RSTR2 &= ~(mask);                                                \
  (void)RCC->AHB2RSTR2;                                                     \
}
/** @} */

/**
 * @name    ADC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the ADC1/ADC2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC12(lp) rccEnableAHB2R1(RCC_AHB2ENR1_ADC12EN, lp)

/**
 * @brief   Disables the ADC1/ADC2 peripheral clock.
 *
 * @api
 */
#define rccDisableADC12() rccDisableAHB2R1(RCC_AHB2ENR1_ADC12EN)

/**
 * @brief   Resets the ADC1/ADC2 peripheral.
 *
 * @api
 */
#define rccResetADC12() rccResetAHB2R1(RCC_AHB2RSTR1_ADC12RST)
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
#define rccEnableDAC1(lp) rccEnableAHB2R1(RCC_AHB2ENR1_DAC1EN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC1() rccDisableAHB2R1(RCC_AHB2ENR1_DAC1EN)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAHB2R1(RCC_AHB2RSTR1_DAC1RST)
/** @} */

/**
 * @name    GPDMA peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the GPDMA1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA31(lp) rccEnableAHB1R1(RCC_AHB1ENR1_GPDMA1EN, lp)

/**
 * @brief   Disables the GPDMA1 peripheral clock.
 *
 * @api
 */
#define rccDisableDMA31() rccDisableAHB1R1(RCC_AHB1ENR1_GPDMA1EN)

/**
 * @brief   Resets the GPDMA1 peripheral.
 *
 * @api
 */
#define rccResetDMA31() rccResetAHB1R1(RCC_AHB1RSTR1_GPDMA1RST)
/** @} */

/**
 * @name    FDCAN peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the FDCAN peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableFDCAN(lp) rccEnableAPB1R2(RCC_APB1ENR2_FDCAN1EN, lp)

/**
 * @brief   Disables the FDCAN peripheral clock.
 *
 * @api
 */
#define rccDisableFDCAN() rccDisableAPB1R2(RCC_APB1ENR2_FDCAN1EN)

/**
 * @brief   Resets the FDCAN peripheral.
 *
 * @api
 */
#define rccResetFDCAN() rccResetAPB1R2(RCC_APB1RSTR2_FDCAN1RST)
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
#define rccEnableI2C1(lp) rccEnableAPB1R1(RCC_APB1ENR1_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C1() rccDisableAPB1R1(RCC_APB1ENR1_I2C1EN)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rccResetI2C1() rccResetAPB1R1(RCC_APB1RSTR1_I2C1RST)

/**
 * @brief   Enables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C2(lp) rccEnableAPB1R1(RCC_APB1ENR1_I2C2EN, lp)

/**
 * @brief   Disables the I2C2 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C2() rccDisableAPB1R1(RCC_APB1ENR1_I2C2EN)

/**
 * @brief   Resets the I2C2 peripheral.
 *
 * @api
 */
#define rccResetI2C2() rccResetAPB1R1(RCC_APB1RSTR1_I2C2RST)

/**
 * @brief   Enables the I2C3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C3(lp) rccEnableAPB3(RCC_APB3ENR_I2C3EN, lp)

/**
 * @brief   Disables the I2C3 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C3() rccDisableAPB3(RCC_APB3ENR_I2C3EN)

/**
 * @brief   Resets the I2C3 peripheral.
 *
 * @api
 */
#define rccResetI2C3() rccResetAPB3(RCC_APB3RSTR_I2C3RST)
/** @} */

/**
 * @name    OCTOSPI peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the OCTOSPI1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableOCTOSPI1(lp) rccEnableAHB2R2(RCC_AHB2ENR2_OCTOSPI1EN, lp)

/**
 * @brief   Disables the OCTOSPI1 peripheral clock.
 *
 * @api
 */
#define rccDisableOCTOSPI1() rccDisableAHB2R2(RCC_AHB2ENR2_OCTOSPI1EN)

/**
 * @brief   Resets the OCTOSPI1 peripheral.
 *
 * @api
 */
#define rccResetOCTOSPI1() rccResetAHB2R2(RCC_AHB2RSTR2_OCTOSPI1RST)
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
#define rccEnablePWRInterface(lp) rccEnableAHB1R2(RCC_AHB1ENR2_PWREN, lp)

/**
 * @brief   Disables PWR interface clock.
 *
 * @api
 */
#define rccDisablePWRInterface() rccDisableAPBR1(RCC_AHB1ENR2_PWREN)

/**
 * @brief   Resets the PWR interface.
 *
 * @api
 */
#define rccResetPWRInterface()
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
#define rccEnableRNG(lp) rccEnableAHB2R1(RCC_AHB2ENR1_RNGEN, lp)

/**
 * @brief   Disables the RNG peripheral clock.
 *
 * @api
 */
#define rccDisableRNG() rccDisableAHB2R1(RCC_AHB2ENR1_RNGEN)

/**
 * @brief   Resets the RNG peripheral.
 *
 * @api
 */
#define rccResetRNG() rccResetAHB2R1(RCC_AHB2RSTR1_RNGRST)
/** @} */

/**
 * @name    SDMMC1 peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SDMMC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSDMMC1(lp) rccEnableAHB2R1(RCC_AHB2ENR1_SDMMC1EN, lp)

/**
 * @brief   Disables the SDMMC1 peripheral clock.
 *
 * @api
 */
#define rccDisableSDMMC1() rccDisableAHB2R1(RCC_AHB2ENR1_SDMMC1EN)

/**
 * @brief   Resets the SDMMC1 peripheral.
 *
 * @api
 */
#define rccResetSDMMC1() rccResetAHB2R1(RCC_AHB2RSTR1_SDMMC1RST)
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
#define rccEnableSPI2(lp) rccEnableAPB1R1(RCC_APB1ENR1_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI2() rccDisableAPB1R1(RCC_APB1ENR1_SPI2EN)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rccResetSPI2() rccResetAPB1R1(RCC_APB1RSTR1_SPI2RST)

/**
 * @brief   Enables the SPI3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI3(lp) rccEnableAPB1R1(RCC_APB1ENR1_SPI3EN, lp)

/**
 * @brief   Disables the SPI3 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI3() rccDisableAPB1R1(RCC_APB1ENR1_SPI3EN)

/**
 * @brief   Resets the SPI3 peripheral.
 *
 * @api
 */
#define rccResetSPI3() rccResetAPB1R1(RCC_APB1RSTR1_SPI3RST)
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
#define rccEnableTIM2(lp) rccEnableAPB1R1(RCC_APB1ENR1_TIM2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM2() rccDisableAPB1R1(RCC_APB1ENR1_TIM2EN)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rccResetTIM2() rccResetAPB1R1(RCC_APB1RSTR1_TIM2RST)

/**
 * @brief   Enables the TIM3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM3(lp) rccEnableAPB1R1(RCC_APB1ENR1_TIM3EN, lp)

/**
 * @brief   Disables the TIM3 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM3() rccDisableAPB1R1(RCC_APB1ENR1_TIM3EN)

/**
 * @brief   Resets the TIM3 peripheral.
 *
 * @api
 */
#define rccResetTIM3() rccResetAPB1R1(RCC_APB1RSTR1_TIM3RST)

/**
 * @brief   Enables the TIM4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM4(lp) rccEnableAPB1R1(RCC_APB1ENR1_TIM4EN, lp)

/**
 * @brief   Disables the TIM4 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM4() rccDisableAPB1R1(RCC_APB1ENR1_TIM4EN)

/**
 * @brief   Resets the TIM4 peripheral.
 *
 * @api
 */
#define rccResetTIM4() rccResetAPB1R1(RCC_APB1RSTR1_TIM4RST)

/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM6(lp) rccEnableAPB1R1(RCC_APB1ENR1_TIM6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM6() rccDisableAPB1R1(RCC_APB1ENR1_TIM6EN)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rccResetTIM6() rccResetAPB1R1(RCC_APB1RSTR1_TIM6RST)

/**
 * @brief   Enables the TIM7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM7(lp) rccEnableAPB1R1(RCC_APB1ENR1_TIM7EN, lp)

/**
 * @brief   Disables the TIM7 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM7() rccDisableAPB1R1(RCC_APB1ENR1_TIM7EN)

/**
 * @brief   Resets the TIM7 peripheral.
 *
 * @api
 */
#define rccResetTIM7() rccResetAPB1R1(RCC_APB1RSTR1_TIM7RST)

/**
 * @brief   Enables the TIM15 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM15(lp) rccEnableAPB2(RCC_APB2ENR_TIM15EN, lp)

/**
 * @brief   Disables the TIM15 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM15() rccDisableAPB2(RCC_APB2ENR_TIM15EN)

/**
 * @brief   Resets the TIM15 peripheral.
 *
 * @api
 */
#define rccResetTIM15() rccResetAPB2(RCC_APB2RSTR_TIM15RST)

/**
 * @brief   Enables the TIM16 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM16(lp) rccEnableAPB2(RCC_APB2ENR_TIM16EN, lp)

/**
 * @brief   Disables the TIM16 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM16() rccDisableAPB2(RCC_APB2ENR_TIM16EN)

/**
 * @brief   Resets the TIM16 peripheral.
 *
 * @api
 */
#define rccResetTIM16() rccResetAPB2(RCC_APB2RSTR_TIM16RST)

/**
 * @brief   Enables the TIM17 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM17(lp) rccEnableAPB2(RCC_APB2ENR_TIM17EN, lp)

/**
 * @brief   Disables the TIM17 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM17() rccDisableAPB2(RCC_APB2ENR_TIM17EN)

/**
 * @brief   Resets the TIM17 peripheral.
 *
 * @api
 */
#define rccResetTIM17() rccResetAPB2(RCC_APB2RSTR_TIM17RST)
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
 * @brief   Enables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART3(lp) rccEnableAPB1R1(RCC_APB1ENR1_USART3EN, lp)

/**
 * @brief   Disables the USART3 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART3() rccDisableAPB1R1(RCC_APB1ENR1_USART3EN)

/**
 * @brief   Resets the USART3 peripheral.
 *
 * @api
 */
#define rccResetUSART3() rccResetAPB1R1(RCC_APB1RSTR1_USART3RST)

/**
 * @brief   Enables the UART4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART4(lp) rccEnableAPB1R1(RCC_APB1ENR1_UART4EN, lp)

/**
 * @brief   Disables the UART4 peripheral clock.
 *
 * @api
 */
#define rccDisableUART4() rccDisableAPB1R1(RCC_APB1ENR1_UART4EN)

/**
 * @brief   Resets the UART4 peripheral.
 *
 * @api
 */
#define rccResetUART4() rccResetAPB1R1(RCC_APB1RSTR1_UART4RST)

/**
 * @brief   Enables the UART5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART5(lp) rccEnableAPB1R1(RCC_APB1ENR1_UART5EN, lp)

/**
 * @brief   Disables the UART5 peripheral clock.
 *
 * @api
 */
#define rccDisableUART5() rccDisableAPB1R1(RCC_APB1ENR1_UART5EN)

/**
 * @brief   Resets the UART5 peripheral.
 *
 * @api
 */
#define rccResetUART5() rccResetAPB1R1(RCC_APB1RSTR1_UART5RST)

/**
 * @brief   Enables the LPUART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableLPUART1(lp) rccEnableAPB3(RCC_APB3ENR_LPUART1EN, lp)

/**
 * @brief   Disables the LPUART1 peripheral clock.
 *
 * @api
 */
#define rccDisableLPUART1() rccDisableAPB3(RCC_APB3ENR_LPUART1EN)

/**
 * @brief   Resets the USART1 peripheral.
 *
 * @api
 */
#define rccResetLPUART1() rccResetAPB3(RCC_APB3RSTR_LPUART1RST)
/** @} */

/**
 * @name    USB peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the USB peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSB(lp) rccEnableAPB2(RCC_APB2ENR_USB1EN, lp)

/**
 * @brief   Disables the USB peripheral clock.
 *
 * @api
 */
#define rccDisableUSB() rccDisableAPB2(RCC_APB2ENR_USB1EN)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rccResetUSB() rccResetAPB2(RCC_APB2RSTR_USB1RST)
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
#define rccEnableCRC(lp) rccEnableAHB1R1(RCC_AHB1ENR1_CRCEN, lp)

/**
 * @brief   Disables the CRC peripheral clock.
 *
 * @api
 */
#define rccDisableCRC() rccDisableAHB1R1(RCC_AHB1ENR1_CRCEN)

/**
 * @brief   Resets the CRC peripheral.
 *
 * @api
 */
#define rccResetCRC() rccResetAHB1R1(RCC_AHB1RSTR1_CRCRST)
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
