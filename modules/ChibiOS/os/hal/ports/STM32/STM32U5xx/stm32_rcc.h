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
 * @file    STM32G4xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32g4xx.h.
 *
 * @addtogroup STM32G4xx_RCC
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
 * @brief   Enables the clock of one or more peripheral on the APB1 bus (low).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1L(mask, lp) {                                          \
  RCC->APB1LENR |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->APB1LLPENR |= (mask);                                              \
  else                                                                      \
    RCC->APB1LLPENR &= ~(mask);                                             \
  (void)RCC->APB1LLPENR;                                                    \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (low).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1L(mask) {                                             \
  RCC->APB1LENR &= ~(mask);                                                 \
  RCC->APB1LLPENR &= ~(mask);                                               \
  (void)RCC->APB1LLPENR;                                                    \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus (low).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 *
 * @api
 */
#define rccResetAPB1L(mask) {                                               \
  RCC->APB1LRSTR |= (mask);                                                 \
  RCC->APB1LRSTR &= ~(mask);                                                \
  (void)RCC->APB1LRSTR;                                                     \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus (high).
 *
 * @param[in] mask      APB1 high peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1H(mask, lp) {                                          \
  RCC->APB1HENR |= (mask);                                                  \
  if (lp)                                                                   \
    RCC->APB1HLPENR |= (mask);                                              \
  else                                                                      \
    RCC->APB1HLPENR &= ~(mask);                                             \
  (void)RCC->APB1HLPENR;                                                    \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (high).
 *
 * @param[in] mask      APB1 high peripherals mask
 *
 * @api
 */
#define rccDisableAPB1H(mask) {                                             \
  RCC->APB1HENR &= ~(mask);                                                 \
  RCC->APB1HLPENR &= ~(mask);                                               \
  (void)RCC->APB1HLPENR;                                                    \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus (high).
 *
 * @param[in] mask      APB1 high peripherals mask
 *
 * @api
 */
#define rccResetAPB1H(mask) {                                               \
  RCC->APB1HRSTR |= (mask);                                                 \
  RCC->APB1HRSTR &= ~(mask);                                                \
  (void)RCC->APB1HRSTR;                                                     \
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
    RCC->APB3LPENR |= (mask);                                               \
  else                                                                      \
    RCC->APB3LPENR &= ~(mask);                                              \
  (void)RCC->APB3LPENR;                                                     \
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
  RCC->APB3LPENR &= ~(mask);                                                \
  (void)RCC->APB3LPENR;                                                     \
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
 * @brief   Enables the clock of one or more peripheral on the AHB4 bus.
 *
 * @param[in] mask      AHB4 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB4(mask, lp) {                                           \
  RCC->AHB4ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->AHB4LPENR |= (mask);                                               \
  else                                                                      \
    RCC->AHB4LPENR &= ~(mask);                                              \
  (void)RCC->AHB4LPENR;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB4 bus.
 *
 * @param[in] mask      AHB4 peripherals mask
 *
 * @api
 */
#define rccDisableAHB4(mask) {                                              \
  RCC->AHB4ENR &= ~(mask);                                                  \
  RCC->AHB4LPENR &= ~(mask);                                                \
  (void)RCC->AHB4LPENR;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the AHB4 bus.
 *
 * @param[in] mask      AHB3 peripherals mask
 *
 * @api
 */
#define rccResetAHB4(mask) {                                                \
  RCC->AHB4RSTR |= (mask);                                                  \
  RCC->AHB4RSTR &= ~(mask);                                                 \
  (void)RCC->AHB4RSTR;                                                      \
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
#define rccEnableADC12(lp) rccEnableAHB2(RCC_AHB2ENR_ADCEN, lp)

/**
 * @brief   Disables the ADC1/ADC2 peripheral clock.
 *
 * @api
 */
#define rccDisableADC12() rccDisableAHB2(RCC_AHB2ENR_ADCEN)

/**
 * @brief   Resets the ADC1/ADC2 peripheral.
 *
 * @api
 */
#define rccResetADC12() rccResetAHB2(RCC_AHB2RSTR_ADCRST)
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
#define rccEnableDAC1(lp) rccEnableAHB2(RCC_AHB2ENR_DAC1EN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC1() rccDisableAHB2(RCC_AHB2ENR_DAC1EN)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAHB2(RCC_AHB2RSTR_DAC1RST)

/**
 * @brief   Enables the DAC2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDAC2(lp) rccEnableAHB2(RCC_AHB2ENR_DAC2EN, lp)

/**
 * @brief   Disables the DAC2 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC2() rccDisableAHB2(RCC_AHB2ENR_DAC2EN)

/**
 * @brief   Resets the DAC2 peripheral.
 *
 * @api
 */
#define rccResetDAC2() rccResetAHB2(RCC_AHB2RSTR_DAC2RST)

/**
 * @brief   Enables the DAC3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDAC3(lp) rccEnableAHB2(RCC_AHB2ENR_DAC3EN, lp)

/**
 * @brief   Disables the DAC3 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC3() rccDisableAHB2(RCC_AHB2ENR_DAC3EN)

/**
 * @brief   Resets the DAC3 peripheral.
 *
 * @api
 */
#define rccResetDAC3() rccResetAHB2(RCC_AHB2RSTR_DAC3RST)

/**
 * @brief   Enables the DAC4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDAC4(lp) rccEnableAHB2(RCC_AHB2ENR_DAC4EN, lp)

/**
 * @brief   Disables the DAC4 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC4() rccDisableAHB2(RCC_AHB2ENR_DAC4EN)

/**
 * @brief   Resets the DAC4 peripheral.
 *
 * @api
 */
#define rccResetDAC4() rccResetAHB2(RCC_AHB2RSTR_DAC4RST)
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
#define rccEnableGPDMA1(lp) rccEnableAHB1(RCC_AHB1ENR_GPDMA1EN, lp)

/**
 * @brief   Disables the GPDMA1 peripheral clock.
 *
 * @api
 */
#define rccDisableGPDMA1() rccDisableAHB1(RCC_AHB1ENR_GPDMA1EN)

/**
 * @brief   Resets the GPDMA1 peripheral.
 *
 * @api
 */
#define rccResetGPDMA1() rccResetAHB1(RCC_AHB1RSTR_GPDMA1RST)

/**
 * @brief   Enables the GPDMA2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableGPDMA2(lp) rccEnableAHB1(RCC_AHB1ENR_GPDMA2EN, lp)

/**
 * @brief   Disables the GPDMA2 peripheral clock.
 *
 * @api
 */
#define rccDisableGPDMA2() rccDisableAHB1(RCC_AHB1ENR_GPDMA2EN)

/**
 * @brief   Resets the GPDMA2 peripheral.
 *
 * @api
 */
#define rccResetGPDMA2() rccResetAHB1(RCC_AHB1RSTR_GPDMA2RST)
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
#define rccEnableFDCAN(lp) rccEnableAPB1L(RCC_APB1LENR_FDCANEN, lp)

/**
 * @brief   Disables the FDCAN peripheral clock.
 *
 * @api
 */
#define rccDisableFDCAN() rccDisableAPB1L(RCC_APB1LENR_FDCANEN)

/**
 * @brief   Resets the FDCAN peripheral.
 *
 * @api
 */
#define rccResetFDCAN() rccResetAPB1L(RCC_APB1LRSTR_FDCANRST)
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
#define rccEnableI2C1(lp) rccEnableAPB1L(RCC_APB1LENR_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C1() rccDisableAPB1L(RCC_APB1LENR_I2C1EN)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rccResetI2C1() rccResetAPB1L(RCC_APB1LRSTR_I2C1RST)

/**
 * @brief   Enables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C2(lp) rccEnableAPB1L(RCC_APB1LENR_I2C2EN, lp)

/**
 * @brief   Disables the I2C2 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C2() rccDisableAPB1L(RCC_APB1LENR_I2C2EN)

/**
 * @brief   Resets the I2C2 peripheral.
 *
 * @api
 */
#define rccResetI2C2() rccResetAPB1L(RCC_APB1LRSTR_I2C2RST)

/**
 * @brief   Enables the I2C3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C3(lp) rccEnableAPB1L(RCC_APB1LENR_I2C3EN, lp)

/**
 * @brief   Disables the I2C3 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C3() rccDisableAPB1L(RCC_APB1LENR_I2C3EN)

/**
 * @brief   Resets the I2C3 peripheral.
 *
 * @api
 */
#define rccResetI2C3() rccResetAPB1L(RCC_APB1LRSTR_I2C3RST)

/**
 * @brief Enables the I2C4 peripheral clock.
 *
 * @param[in] lp low power enable flag
 *
 * @api
 */
#define rccEnableI2C4(lp) rccEnableAPB1H(RCC_APB1HENR_I2C4EN, lp)

/**
 * @brief Disables the I2C4 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C4() rccDisableAPB1L(RCC_APB1HENR_I2C4EN)

/**
 * @brief Resets the I2C4 peripheral.
 *
 * @api
 */
#define rccResetI2C4() rccResetAPB1L(RCC_APB1RSTR2_I2C4RST)
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
#define rccEnableSPI2(lp) rccEnableAPB1L(RCC_APB1LENR_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI2() rccDisableAPB1L(RCC_APB1LENR_SPI2EN)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rccResetSPI2() rccResetAPB1L(RCC_APB1LRSTR_SPI2RST)

/**
 * @brief   Enables the SPI3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI3(lp) rccEnableAPB1L(RCC_APB1LENR_SPI3EN, lp)

/**
 * @brief   Disables the SPI3 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI3() rccDisableAPB1L(RCC_APB1LENR_SPI3EN)

/**
 * @brief   Resets the SPI3 peripheral.
 *
 * @api
 */
#define rccResetSPI3() rccResetAPB1L(RCC_APB1LRSTR_SPI3RST)

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
#define rccEnableSPI5(lp) rccEnableAPB3(RCC_APB3ENR_SPI5EN, lp)

/**
 * @brief   Disables the SPI5 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI5() rccDisableAPB3(RCC_APB3ENR_SPI5EN)

/**
 * @brief   Resets the SPI5 peripheral.
 *
 * @api
 */
#define rccResetSPI5() rccResetAPB3(RCC_APB3RSTR_SPI5RST)

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
#define rccEnableTIM2(lp) rccEnableAPB1L(RCC_APB1LENR_TIM2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM2() rccDisableAPB1L(RCC_APB1LENR_TIM2EN)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rccResetTIM2() rccResetAPB1L(RCC_APB1LRSTR_TIM2RST)

/**
 * @brief   Enables the TIM3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM3(lp) rccEnableAPB1L(RCC_APB1LENR_TIM3EN, lp)

/**
 * @brief   Disables the TIM3 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM3() rccDisableAPB1L(RCC_APB1LENR_TIM3EN)

/**
 * @brief   Resets the TIM3 peripheral.
 *
 * @api
 */
#define rccResetTIM3() rccResetAPB1L(RCC_APB1LRSTR_TIM3RST)

/**
 * @brief   Enables the TIM4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM4(lp) rccEnableAPB1L(RCC_APB1LENR_TIM4EN, lp)

/**
 * @brief   Disables the TIM4 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM4() rccDisableAPB1L(RCC_APB1LENR_TIM4EN)

/**
 * @brief   Resets the TIM4 peripheral.
 *
 * @api
 */
#define rccResetTIM4() rccResetAPB1L(RCC_APB1LRSTR_TIM4RST)

/**
 * @brief   Enables the TIM5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM5(lp) rccEnableAPB1L(RCC_APB1LENR_TIM5EN, lp)

/**
 * @brief   Disables the TIM5 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM5() rccDisableAPB1L(RCC_APB1LENR_TIM5EN)

/**
 * @brief   Resets the TIM5 peripheral.
 *
 * @api
 */
#define rccResetTIM5() rccResetAPB1L(RCC_APB1LRSTR_TIM5RST)

/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM6(lp) rccEnableAPB1L(RCC_APB1LENR_TIM6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM6() rccDisableAPB1L(RCC_APB1LENR_TIM6EN)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rccResetTIM6() rccResetAPB1L(RCC_APB1LRSTR_TIM6RST)

/**
 * @brief   Enables the TIM7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM7(lp) rccEnableAPB1L(RCC_APB1LENR_TIM7EN, lp)

/**
 * @brief   Disables the TIM7 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM7() rccDisableAPB1L(RCC_APB1LENR_TIM7EN)

/**
 * @brief   Resets the TIM7 peripheral.
 *
 * @api
 */
#define rccResetTIM7() rccResetAPB1L(RCC_APB1LRSTR_TIM7RST)

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
 * @brief   Enables the TIM12 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM12(lp) rccEnableAPB1L(RCC_APB1LENR_TIM12EN, lp)

/**
 * @brief   Disables the TIM12 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM12() rccDisableAPB1L(RCC_APB1LENR_TIM12EN)

/**
 * @brief   Resets the TIM12 peripheral.
 *
 * @api
 */
#define rccResetTIM12() rccResetAPB1L(RCC_APB1LRSTR_TIM12RST)

/**
 * @brief   Enables the TIM13 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM13(lp) rccEnableAPB1L(RCC_APB1LENR_TIM13EN, lp)

/**
 * @brief   Disables the TIM13 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM13() rccDisableAPB1L(RCC_APB1LENR_TIM13EN)

/**
 * @brief   Resets the TIM13 peripheral.
 *
 * @api
 */
#define rccResetTIM13() rccResetAPB1L(RCC_APB1LRSTR_TIM13RST)

/**
 * @brief   Enables the TIM14 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM14(lp) rccEnableAPB1L(RCC_APB1LENR_TIM14EN, lp)

/**
 * @brief   Disables the TIM14 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM14() rccDisableAPB1L(RCC_APB1LENR_TIM14EN)

/**
 * @brief   Resets the TIM14 peripheral.
 *
 * @api
 */
#define rccResetTIM14() rccResetAPB1L(RCC_APB1LRSTR_TIM14RST)

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
 * @brief   Enables the USART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART2(lp) rccEnableAPB1L(RCC_APB1LENR_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART2() rccDisableAPB1L(RCC_APB1LENR_USART2EN)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rccResetUSART2() rccResetAPB1L(RCC_APB1LRSTR_USART2RST)

/**
 * @brief   Enables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART3(lp) rccEnableAPB1L(RCC_APB1LENR_USART3EN, lp)

/**
 * @brief   Disables the USART3 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART3() rccDisableAPB1L(RCC_APB1LENR_USART3EN)

/**
 * @brief   Resets the USART3 peripheral.
 *
 * @api
 */
#define rccResetUSART3() rccResetAPB1L(RCC_APB1LRSTR_USART3RST)

/**
 * @brief   Enables the UART4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART4(lp) rccEnableAPB1L(RCC_APB1LENR_UART4EN, lp)

/**
 * @brief   Disables the UART4 peripheral clock.
 *
 * @api
 */
#define rccDisableUART4() rccDisableAPB1L(RCC_APB1LENR_UART4EN)

/**
 * @brief   Resets the UART4 peripheral.
 *
 * @api
 */
#define rccResetUART4() rccResetAPB1L(RCC_APB1LRSTR_UART4RST)

/**
 * @brief   Enables the UART5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART5(lp) rccEnableAPB1L(RCC_APB1LENR_UART5EN, lp)

/**
 * @brief   Disables the UART5 peripheral clock.
 *
 * @api
 */
#define rccDisableUART5() rccDisableAPB1L(RCC_APB1LENR_UART5EN)

/**
 * @brief   Resets the UART5 peripheral.
 *
 * @api
 */
#define rccResetUART5() rccResetAPB1L(RCC_APB1LRSTR_UART5RST)

/**
 * @brief   Enables the USART6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART6(lp) rccEnableAPB1L(RCC_APB1LENR_USART6EN, lp)

/**
 * @brief   Disables the USART6 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART6() rccDisableAPB1L(RCC_APB1LENR_USART6EN)

/**
 * @brief   Resets the USART6 peripheral.
 *
 * @api
 */
#define rccResetUSART6() rccResetAPB1L(RCC_APB1LRSTR_USART6RST)

/**
 * @brief   Enables the UART7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART7(lp) rccEnableAPB1L(RCC_APB1LENR_UART7EN, lp)

/**
 * @brief   Disables the UART7 peripheral clock.
 *
 * @api
 */
#define rccDisableUART7() rccDisableAPB1L(RCC_APB1LENR_UART7EN)

/**
 * @brief   Resets the UART7 peripheral.
 *
 * @api
 */
#define rccResetUART7() rccResetAPB1L(RCC_APB1LRSTR_UART7RST)

/**
 * @brief   Enables the UART8 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART8(lp) rccEnableAPB1L(RCC_APB1LENR_UART8EN, lp)

/**
 * @brief   Disables the UART8 peripheral clock.
 *
 * @api
 */
#define rccDisableUART8() rccDisableAPB1L(RCC_APB1LENR_UART8EN)

/**
 * @brief   Resets the UART8 peripheral.
 *
 * @api
 */
#define rccResetUART8() rccResetAPB1L(RCC_APB1LRSTR_UART8RST)

/**
 * @brief   Enables the UART9 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART9(lp) rccEnableAPB1H(RCC_APB1HENR_UART9EN, lp)

/**
 * @brief   Disables the UART9 peripheral clock.
 *
 * @api
 */
#define rccDisableUART9() rccDisableAPB1H(RCC_APB1HENR_UART9EN)

/**
 * @brief   Resets the UART9 peripheral.
 *
 * @api
 */
#define rccResetUART9() rccResetAPB1H(RCC_APB1HRSTR_UART9RST)

/**
 * @brief   Enables the USART10 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART10(lp) rccEnableAPB1L(RCC_APB1LENR_USART10EN, lp)

/**
 * @brief   Disables the USART10 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART10() rccDisableAPB1L(RCC_APB1LENR_USART10EN)

/**
 * @brief   Resets the USART10 peripheral.
 *
 * @api
 */
#define rccResetUSART10() rccResetAPB1L(RCC_APB1LRSTR_USART10RST)

/**
 * @brief   Enables the USART11 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART11(lp) rccEnableAPB1L(RCC_APB1LENR_USART11EN, lp)

/**
 * @brief   Disables the USART11 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART11() rccDisableAPB1L(RCC_APB1LENR_USART11EN)

/**
 * @brief   Resets the USART11 peripheral.
 *
 * @api
 */
#define rccResetUSART11() rccResetAPB1L(RCC_APB1LRSTR_USART11RST)

/**
 * @brief   Enables the UART12 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART12(lp) rccEnableAPB1H(RCC_APB1HENR_UART12EN, lp)

/**
 * @brief   Disables the UART12 peripheral clock.
 *
 * @api
 */
#define rccDisableUART12() rccDisableAPB1H(RCC_APB1HENR_UART12EN)

/**
 * @brief   Resets the UART12 peripheral.
 *
 * @api
 */
#define rccResetUART12() rccResetAPB1H(RCC_APB1HRSTR_UART12RST)

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
#define rccEnableUSB(lp) rccEnableAPB2(RCC_APB2ENR_USBEN, lp)

/**
 * @brief   Disables the USB peripheral clock.
 *
 * @api
 */
#define rccDisableUSB() rccDisableAPB2(RCC_APB2ENR_USBEN)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rccResetUSB() rccResetAPB2(RCC_APB2RSTR_USBRST)
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
