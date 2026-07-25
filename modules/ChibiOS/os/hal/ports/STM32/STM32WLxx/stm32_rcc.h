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
 * @file    STM32WLxx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32wlxx.h.
 *
 * @addtogroup STM32WLxx_RCC
 * @{
 */
#ifndef STM32_RCC_H
#define STM32_RCC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Mapping RCC registers depends on target core.*/
#if  STM32_TARGET_CORE == 1
#define STM32_RCC_AHB1ENR           RCC->AHB1ENR
#define STM32_RCC_AHB2ENR           RCC->AHB2ENR
#define STM32_RCC_AHB3ENR           RCC->AHB3ENR
#define STM32_RCC_APB1ENR1          RCC->APB1ENR1
#define STM32_RCC_APB1ENR2          RCC->APB1ENR2
#define STM32_RCC_APB2ENR           RCC->APB2ENR
#define STM32_RCC_APB3ENR           RCC->APB3ENR
#define STM32_RCC_AHB1SMENR         RCC->AHB1SMENR
#define STM32_RCC_AHB2SMENR         RCC->AHB2SMENR
#define STM32_RCC_AHB3SMENR         RCC->AHB3SMENR
#define STM32_RCC_APB1SMENR1        RCC->APB1SMENR1
#define STM32_RCC_APB1SMENR2        RCC->APB1SMENR2
#define STM32_RCC_APB2SMENR         RCC->APB2SMENR
#define STM32_RCC_APB3SMENR         RCC->APB3SMENR
#else /*  STM32_TARGET_CORE == 2 */
#define STM32_RCC_AHB1ENR           RCC->C2AHB1ENR
#define STM32_RCC_AHB2ENR           RCC->C2AHB2ENR
#define STM32_RCC_AHB3ENR           RCC->C2AHB3ENR
#define STM32_RCC_APB1ENR1          RCC->C2APB1ENR1
#define STM32_RCC_APB1ENR2          RCC->C2APB1ENR2
#define STM32_RCC_APB2ENR           RCC->C2APB2ENR
#define STM32_RCC_APB3ENR           RCC->C2APB3ENR
#define STM32_RCC_AHB1SMENR         RCC->C2AHB1SMENR
#define STM32_RCC_AHB2SMENR         RCC->C2AHB2SMENR
#define STM32_RCC_AHB3SMENR         RCC->C2AHB3SMENR
#define STM32_RCC_APB1SMENR1        RCC->C2APB1SMENR1
#define STM32_RCC_APB1SMENR2        RCC->C2APB1SMENR2
#define STM32_RCC_APB2SMENR         RCC->C2APB2SMENR
#define STM32_RCC_APB3SMENR         RCC->C2APB3SMENR
#endif /* STM32_TARGET_CORE == 1 */

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
  STM32_RCC_APB1ENR1 |= (mask);                                             \
  if (lp)                                                                   \
    STM32_RCC_APB1SMENR1 |= (mask);                                         \
  else                                                                      \
    STM32_RCC_APB1SMENR1 &= ~(mask);                                        \
  (void)STM32_RCC_APB1SMENR1;                                               \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (R1).
 *
 * @param[in] mask      APB1 R1 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1R1(mask) {                                            \
  STM32_RCC_APB1ENR1 &= ~(mask);                                            \
  STM32_RCC_APB1SMENR1 &= ~(mask);                                          \
  (void)STM32_RCC_APB1SMENR1;                                               \
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
  STM32_RCC_APB1ENR2 |= (mask);                                             \
  if (lp)                                                                   \
    STM32_RCC_APB1SMENR2 |= (mask);                                         \
  else                                                                      \
    STM32_RCC_APB1SMENR2 &= ~(mask);                                        \
  (void)STM32_RCC_APB1SMENR2;                                               \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus (R2).
 *
 * @param[in] mask      APB1 R2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1R2(mask) {                                            \
  STM32_RCC_APB1ENR2 &= ~(mask);                                            \
  STM32_RCC_APB1SMENR2 &= ~(mask);                                          \
  (void)STM32_RCC_APB1SMENR2;                                               \
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
  STM32_RCC_APB2ENR |= (mask);                                              \
  if (lp)                                                                   \
    STM32_RCC_APB2SMENR |= (mask);                                          \
  else                                                                      \
    STM32_RCC_APB2SMENR &= ~(mask);                                         \
  (void)STM32_RCC_APB2SMENR;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB2(mask) {                                              \
  STM32_RCC_APB2ENR &= ~(mask);                                             \
  STM32_RCC_APB2SMENR &= ~(mask);                                           \
  (void)STM32_RCC_APB2SMENR;                                                \
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
  STM32_RCC_APB3ENR |= (mask);                                              \
  if (lp)                                                                   \
    STM32_RCC_APB3SMENR |= (mask);                                          \
  else                                                                      \
    STM32_RCC_APB3SMENR &= ~(mask);                                         \
  (void)STM32_RCC_APB3SMENR;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB3 bus.
 *
 * @param[in] mask      APB3 peripherals mask
 *
 * @api
 */
#define rccDisableAPB3(mask) {                                              \
  STM32_RCC_APB3ENR &= ~(mask);                                             \
  STM32_RCC_APB3SMENR &= ~(mask);                                           \
  (void)STM32_RCC_APB3SMENR;                                                \
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
  STM32_RCC_AHB1ENR |= (mask);                                              \
  if (lp)                                                                   \
    STM32_RCC_AHB1SMENR |= (mask);                                          \
  else                                                                      \
    STM32_RCC_AHB1SMENR &= ~(mask);                                         \
  (void)STM32_RCC_AHB1SMENR;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB1 bus.
 *
 * @param[in] mask      AHB1 peripherals mask
 *
 * @api
 */
#define rccDisableAHB1(mask) {                                              \
  STM32_RCC_AHB1ENR &= ~(mask);                                             \
  STM32_RCC_AHB1SMENR &= ~(mask);                                           \
  (void)STM32_RCC_AHB1SMENR;                                                \
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
  STM32_RCC_AHB2ENR |= (mask);                                              \
  if (lp)                                                                   \
    STM32_RCC_AHB2SMENR |= (mask);                                          \
  else                                                                      \
    STM32_RCC_AHB2SMENR &= ~(mask);                                         \
  (void)STM32_RCC_AHB2SMENR;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB2 bus.
 *
 * @param[in] mask      AHB2 peripherals mask
 *
 * @api
 */
#define rccDisableAHB2(mask) {                                              \
  STM32_RCC_AHB2ENR &= ~(mask);                                             \
  STM32_RCC_AHB2SMENR &= ~(mask);                                           \
  (void)STM32_RCC_AHB2SMENR;                                                \
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
  STM32_RCC_AHB3ENR |= (mask);                                              \
  if (lp)                                                                   \
    STM32_RCC_AHB3SMENR |= (mask);                                          \
  else                                                                      \
    STM32_RCC_AHB3SMENR &= ~(mask);                                         \
  (void)STM32_RCC_AHB3SMENR;                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB3 (FSMC) bus.
 *
 * @param[in] mask      AHB3 peripherals mask
 *
 * @api
 */
#define rccDisableAHB3(mask) {                                              \
  STM32_RCC_AHB3ENR &= ~(mask);                                             \
  STM32_RCC_AHB3SMENR &= ~(mask);                                           \
  (void)STM32_RCC_AHB3SMENR;                                                \
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
 * @brief   Enables the ADC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC1(lp) rccEnableAPB2(RCC_APB2ENR_ADCEN, lp)

/**
 * @brief   Disables the ADC1 peripheral clock.
 *
 * @api
 */
#define rccDisableADC1() rccDisableAPB2(RCC_APB2RSTR_ADCRST)

/**
 * @brief   Resets the ADC1 peripheral.
 *
 * @api
 */
#define rccResetADC1() rccResetAPB2(RCC_APB2RSTR_ADCRST)
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
#define rccEnableDAC1(lp) rccEnableAPB1R1(RCC_APB1ENR1_DACEN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC1() rccDisableAPB1R1(RCC_APB1ENR1_DACEN)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAPB1R1(RCC_APB1RSTR1_DACRST)
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
 * @name    DMAMUX peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DMAMUX peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMAMUX(lp) rccEnableAHB1(RCC_AHB1ENR_DMAMUX1EN, lp)

/**
 * @brief   Disables the DMAMUX peripheral clock.
 *
 * @api
 */
#define rccDisableDMAMUX() rccDisableAHB1(RCC_AHB1ENR_DMAMUX1EN)

/**
 * @brief   Resets the DMAMUX peripheral.
 *
 * @api
 */
#define rccResetDMAMUX() rccResetAHB1(RCC_AHB1RSTR_DMAMUX1RST)
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
#define rccEnableI2C3(lp) rccEnableAPB1R1(RCC_APB1ENR1_I2C3EN, lp)

/**
 * @brief   Disables the I2C3 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C3() rccDisableAPB1R1(RCC_APB1ENR1_I2C3EN)

/**
 * @brief   Resets the I2C3 peripheral.
 *
 * @api
 */
#define rccResetI2C3() rccResetAPB1R1(RCC_APB1RSTR1_I2C3RST)
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
#define rccEnableRNG(lp) rccEnableAHB3(RCC_AHB3ENR_RNGEN, lp)

/**
 * @brief   Disables the RNG peripheral clock.
 *
 * @api
 */
#define rccDisableRNG() rccDisableAHB3(RCC_AHB3ENR_RNGEN)

/**
 * @brief   Resets the RNG peripheral.
 *
 * @api
 */
#define rccResetRNG() rccResetAHB3(RCC_AHB3RSTR_RNGRST)
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
#define rccEnableSPI3(lp) rccEnableAPB3(RCC_APB3ENR_SUBGHZSPIEN, lp)

/**
 * @brief   Disables the SPI3 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI3() rccDisableAPB3(RCC_APB3ENR_SUBGHZSPIEN)

/**
 * @brief   Resets the SPI3 peripheral.
 *
 * @api
 */
#define rccResetSPI3() rccResetAPB3(RCC_APB3RSTR_SUBGHZSPIRST)
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
#define rccEnableUSART2(lp) rccEnableAPB1R1(RCC_APB1ENR1_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART2() rccDisableAPB1R1(RCC_APB1ENR1_USART2EN)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rccResetUSART2() rccResetAPB1R1(RCC_APB1RSTR1_USART2RST)

/**
 * @brief   Enables the LPUART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableLPUART1(lp) rccEnableAPB1R2(RCC_APB1ENR2_LPUART1EN, lp)

/**
 * @brief   Disables the LPUART1 peripheral clock.
 *
 * @api
 */
#define rccDisableLPUART1() rccDisableAPB1R2(RCC_APB1ENR2_LPUART1EN)

/**
 * @brief   Resets the LPUART1 peripheral.
 *
 * @api
 */
#define rccResetLPUART1() rccResetAPB1R2(RCC_APB1RSTR2_LPUART1RST)
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
