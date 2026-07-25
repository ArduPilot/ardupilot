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
 * @file    STM32G0xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32g0xx.h.
 *
 * @addtogroup STM32G0xx_RCC
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
 * @brief   Enables the clock of one or more peripheral on the APB bus (R1).
 *
 * @param[in] mask      APB R1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPBR1(mask, lp) {                                          \
  RCC->APBENR1 |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APBSMENR1 |= (mask);                                               \
  else                                                                      \
    RCC->APBSMENR1 &= ~(mask);                                              \
  (void)RCC->APBSMENR1;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB bus (R1).
 *
 * @param[in] mask      APB R1 peripherals mask
 *
 * @api
 */
#define rccDisableAPBR1(mask) {                                             \
  RCC->APBENR1 &= ~(mask);                                                  \
  RCC->APBSMENR1 &= ~(mask);                                                \
  (void)RCC->APBSMENR1;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the APB bus (R1).
 *
 * @param[in] mask      APB R1 peripherals mask
 *
 * @api
 */
#define rccResetAPBR1(mask) {                                               \
  RCC->APBRSTR1 |= (mask);                                                  \
  RCC->APBRSTR1 &= ~(mask);                                                 \
  (void)RCC->APBRSTR1;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB bus (R2).
 *
 * @param[in] mask      APB R2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPBR2(mask, lp) {                                          \
  RCC->APBENR2 |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APBSMENR2 |= (mask);                                               \
  else                                                                      \
    RCC->APBSMENR2 &= ~(mask);                                              \
  (void)RCC->APBSMENR2;                                                     \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB bus (R2).
 *
 * @param[in] mask      APB R2 peripherals mask
 *
 * @api
 */
#define rccDisableAPBR2(mask) {                                             \
  RCC->APBENR2 &= ~(mask);                                                  \
  RCC->APBSMENR2 &= ~(mask);                                                \
  (void)RCC->APBSMENR2;                                                     \
}

/**
 * @brief   Resets one or more peripheral on the APB bus (R2).
 *
 * @param[in] mask      APB R2 peripherals mask
 *
 * @api
 */
#define rccResetAPBR2(mask) {                                               \
  RCC->APBRSTR2 |= (mask);                                                  \
  RCC->APBRSTR2 &= ~(mask);                                                 \
  (void)RCC->APBRSTR2;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the IOP bus.
 *
 * @param[in] mask      IOP peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableIOP(mask, lp) {                                            \
  RCC->IOPENR |= (mask);                                                    \
  if (lp)                                                                   \
    RCC->IOPSMENR |= (mask);                                                \
  else                                                                      \
    RCC->IOPSMENR &= ~(mask);                                               \
  (void)RCC->IOPSMENR;                                                      \
}

/**
 * @brief   Disables the clock of one or more peripheral on the IOP bus.
 *
 * @param[in] mask      IOP peripherals mask
 *
 * @api
 */
#define rccDisableIOP(mask) {                                               \
  RCC->IOPENR &= ~(mask);                                                   \
  RCC->IOPSMENR &= ~(mask);                                                 \
  (void)RCC->IOPSMENR;                                                      \
}

/**
 * @brief   Resets one or more peripheral on the IOP bus.
 *
 * @param[in] mask      IOP peripherals mask
 *
 * @api
 */
#define rccResetIOP(mask) {                                                 \
  RCC->IOPRSTR |= (mask);                                                   \
  RCC->IOPRSTR &= ~(mask);                                                  \
  (void)RCC->IOPRSTR;                                                       \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB(mask, lp) {                                            \
  RCC->AHBENR |= (mask);                                                    \
  if (lp)                                                                   \
    RCC->AHBSMENR |= (mask);                                                \
  else                                                                      \
    RCC->AHBSMENR &= ~(mask);                                               \
  (void)RCC->AHBSMENR;                                                      \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rccDisableAHB(mask) {                                               \
  RCC->AHBENR &= ~(mask);                                                   \
  RCC->AHBSMENR &= ~(mask);                                                 \
  (void)RCC->AHBSMENR;                                                      \
}

/**
 * @brief   Resets one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rccResetAHB(mask) {                                                 \
  RCC->AHBRSTR |= (mask);                                                   \
  RCC->AHBRSTR &= ~(mask);                                                  \
  (void)RCC->AHBRSTR;                                                       \
}
/** @} */

/**
 * @name    ADC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the ADC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC1(lp) rccEnableAPBR2(RCC_APBENR2_ADCEN, lp)

/**
 * @brief   Disables the ADC peripheral clock.
 *
 * @api
 */
#define rccDisableADC1() rccDisableAPBR2(RCC_APBENR2_ADCEN)

/**
 * @brief   Resets the ADC peripheral.
 *
 * @api
 */
#define rccResetADC1() rccResetAPBR2(RCC_APBRSTR2_ADCRST)
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
#define rccEnableDAC1(lp) rccEnableAPBR1(RCC_APBENR1_DAC1EN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @api
 */
#define rccDisableDAC1() rccDisableAPBR1(RCC_APBENR1_DAC1EN)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAPBR1(RCC_APBRSTR1_DAC1RST)
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
#define rccEnableDMA1(lp) rccEnableAHB(RCC_AHBENR_DMA1EN, lp)

/**
 * @brief   Disables the DMA1 peripheral clock.
 *
 * @api
 */
#define rccDisableDMA1() rccDisableAHB(RCC_AHBENR_DMA1EN)

/**
 * @brief   Resets the DMA1 peripheral.
 *
 * @api
 */
#define rccResetDMA1() rccResetAHB(RCC_AHBRSTR_DMA1RST)

/**
 * @brief   Enables the DMA2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA2(lp) rccEnableAHB(RCC_AHBENR_DMA2EN, lp)

/**
 * @brief   Disables the DMA2 peripheral clock.
 *
 * @api
 */
#define rccDisableDMA2() rccDisableAHB(RCC_AHBENR_DMA2EN)

/**
 * @brief   Resets the DMA2 peripheral.
 *
 * @api
 */
#define rccResetDMA2() rccResetAHB(RCC_AHBRSTR_DMA2RST)
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
#define rccEnableDMAMUX(lp)

/**
 * @brief   Disables the DMAMUX peripheral clock.
 *
 * @api
 */
#define rccDisableDMAMUX()

/**
 * @brief   Resets the DMAMUX peripheral.
 *
 * @api
 */
#define rccResetDMAMUX()
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
#define rccEnablePWRInterface(lp) rccEnableAPBR1(RCC_APBENR1_PWREN, lp)

/**
 * @brief   Disables PWR interface clock.
 *
 * @api
 */
#define rccDisablePWRInterface() rccDisableAPBR1(RCC_APBENR1_PWREN)

/**
 * @brief   Resets the PWR interface.
 *
 * @api
 */
#define rccResetPWRInterface() rccResetAPBR1(RCC_APBRSTR1_PWRRST)
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
#define rccEnableI2C1(lp) rccEnableAPBR1(RCC_APBENR1_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C1() rccDisableAPBR1(RCC_APBENR1_I2C1EN)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rccResetI2C1() rccResetAPBR1(RCC_APBRSTR1_I2C1RST)

/**
 * @brief   Enables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C2(lp) rccEnableAPBR1(RCC_APBENR1_I2C2EN, lp)

/**
 * @brief   Disables the I2C2 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C2() rccDisableAPBR1(RCC_APBENR1_I2C2EN)

/**
 * @brief   Resets the I2C2 peripheral.
 *
 * @api
 */
#define rccResetI2C2() rccResetAPBR1(RCC_APBRSTR1_I2C2RST)

/**
 * @brief   Enables the I2C3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C3(lp) rccEnableAPBR1(RCC_APBENR1_I2C3EN, lp)

/**
 * @brief   Disables the I2C3 peripheral clock.
 *
 * @api
 */
#define rccDisableI2C3() rccDisableAPBR1(RCC_APBENR1_I2C3EN)

/**
 * @brief   Resets the I2C3 peripheral.
 *
 * @api
 */
#define rccResetI2C3() rccResetAPBR1(RCC_APBRSTR1_I2C3RST)
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
#define rccEnableRNG(lp) rccEnableAHB(RCC_AHBENR_RNGEN, lp)

/**
 * @brief   Disables the RNG peripheral clock.
 *
 * @api
 */
#define rccDisableRNG() rccDisableAHB(RCC_AHBENR_RNGEN)

/**
 * @brief   Resets the RNG peripheral.
 *
 * @api
 */
#define rccResetRNG() rccResetAHB(RCC_AHBRSTR_RNGRST)
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
#define rccEnableSPI1(lp) rccEnableAPBR2(RCC_APBENR2_SPI1EN, lp)

/**
 * @brief   Disables the SPI1 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI1() rccDisableAPBR2(RCC_APBENR2_SPI1EN)

/**
 * @brief   Resets the SPI1 peripheral.
 *
 * @api
 */
#define rccResetSPI1() rccResetAPBR2(RCC_APBRSTR2_SPI1RST)

/**
 * @brief   Enables the SPI2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI2(lp) rccEnableAPBR1(RCC_APBENR1_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI2() rccDisableAPBR1(RCC_APBENR1_SPI2EN)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rccResetSPI2() rccResetAPBR1(RCC_APBRSTR1_SPI2RST)

/**
 * @brief   Enables the SPI3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI3(lp) rccEnableAPBR1(RCC_APBENR1_SPI3EN, lp)

/**
 * @brief   Disables the SPI3 peripheral clock.
 *
 * @api
 */
#define rccDisableSPI3() rccDisableAPBR1(RCC_APBENR1_SPI3EN)

/**
 * @brief   Resets the SPI3 peripheral.
 *
 * @api
 */
#define rccResetSPI3() rccResetAPBR1(RCC_APBRSTR1_SPI3RST)
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
#define rccEnableTIM1(lp) rccEnableAPBR2(RCC_APBENR2_TIM1EN, lp)

/**
 * @brief   Disables the TIM1 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM1() rccDisableAPBR2(RCC_APBENR2_TIM1EN)

/**
 * @brief   Resets the TIM1 peripheral.
 *
 * @api
 */
#define rccResetTIM1() rccResetAPBR2(RCC_APBRSTR2_TIM1RST)

/**
 * @brief   Enables the TIM2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM2(lp) rccEnableAPBR1(RCC_APBENR1_TIM2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM2() rccDisableAPBR1(RCC_APBENR1_TIM2EN)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rccResetTIM2() rccResetAPBR1(RCC_APBRSTR1_TIM2RST)

/**
 * @brief   Enables the TIM3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM3(lp) rccEnableAPBR1(RCC_APBENR1_TIM3EN, lp)

/**
 * @brief   Disables the TIM3 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM3() rccDisableAPBR1(RCC_APBENR1_TIM3EN)

/**
 * @brief   Resets the TIM3 peripheral.
 *
 * @api
 */
#define rccResetTIM3() rccResetAPBR1(RCC_APBRSTR1_TIM3RST)

/**
 * @brief   Enables the TIM4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM4(lp) rccEnableAPBR1(RCC_APBENR1_TIM4EN, lp)

/**
 * @brief   Disables the TIM4 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM4() rccDisableAPBR1(RCC_APBENR1_TIM4EN)

/**
 * @brief   Resets the TIM4 peripheral.
 *
 * @api
 */
#define rccResetTIM4() rccResetAPBR1(RCC_APBRSTR1_TIM4RST)

/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM6(lp) rccEnableAPBR1(RCC_APBENR1_TIM6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM6() rccDisableAPBR1(RCC_APBENR1_TIM6EN)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rccResetTIM6() rccResetAPBR1(RCC_APBRSTR1_TIM6RST)

/**
 * @brief   Enables the TIM7 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM7(lp) rccEnableAPBR1(RCC_APBENR1_TIM7EN, lp)

/**
 * @brief   Disables the TIM7 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM7() rccDisableAPBR1(RCC_APBENR1_TIM7EN)

/**
 * @brief   Resets the TIM7 peripheral.
 *
 * @api
 */
#define rccResetTIM7() rccResetAPBR1(RCC_APBRSTR1_TIM7RST)

/**
 * @brief   Enables the TIM14 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM14(lp) rccEnableAPBR2(RCC_APBENR2_TIM14EN, lp)

/**
 * @brief   Disables the TIM14 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM14() rccDisableAPBR2(RCC_APBENR2_TIM14EN)

/**
 * @brief   Resets the TIM14 peripheral.
 *
 * @api
 */
#define rccResetTIM14() rccResetAPBR2(RCC_APBRSTR2_TIM14RST)

/**
 * @brief   Enables the TIM15 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM15(lp) rccEnableAPBR2(RCC_APBENR2_TIM15EN, lp)

/**
 * @brief   Disables the TIM15 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM15() rccDisableAPBR2(RCC_APBENR2_TIM15EN)

/**
 * @brief   Resets the TIM15 peripheral.
 *
 * @api
 */
#define rccResetTIM15() rccResetAPBR2(RCC_APBRSTR2_TIM15RST)

/**
 * @brief   Enables the TIM16 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM16(lp) rccEnableAPBR2(RCC_APBENR2_TIM16EN, lp)

/**
 * @brief   Disables the TIM16 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM16() rccDisableAPBR2(RCC_APBENR2_TIM16EN)

/**
 * @brief   Resets the TIM16 peripheral.
 *
 * @api
 */
#define rccResetTIM16() rccResetAPBR2(RCC_APBRSTR2_TIM16RST)

/**
 * @brief   Enables the TIM17 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM17(lp) rccEnableAPBR2(RCC_APBENR2_TIM17EN, lp)

/**
 * @brief   Disables the TIM17 peripheral clock.
 *
 * @api
 */
#define rccDisableTIM17() rccDisableAPBR2(RCC_APBENR2_TIM17EN)

/**
 * @brief   Resets the TIM17 peripheral.
 *
 * @api
 */
#define rccResetTIM17() rccResetAPBR2(RCC_APBRSTR2_TIM17RST)
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
#define rccEnableUSART1(lp) rccEnableAPBR2(RCC_APBENR2_USART1EN, lp)

/**
 * @brief   Disables the USART1 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART1() rccDisableAPBR2(RCC_APBENR2_USART1EN)

/**
 * @brief   Resets the USART1 peripheral.
 *
 * @api
 */
#define rccResetUSART1() rccResetAPBR2(RCC_APBRSTR2_USART1RST)

/**
 * @brief   Enables the USART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART2(lp) rccEnableAPBR1(RCC_APBENR1_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART2() rccDisableAPBR1(RCC_APBENR1_USART2EN)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rccResetUSART2() rccResetAPBR1(RCC_APBRSTR1_USART2RST)

/**
 * @brief   Enables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART3(lp) rccEnableAPBR1(RCC_APBENR1_USART3EN, lp)

/**
 * @brief   Disables the USART3 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART3() rccDisableAPBR1(RCC_APBENR1_USART3EN)

/**
 * @brief   Resets the USART3 peripheral.
 *
 * @api
 */
#define rccResetUSART3() rccResetAPBR1(RCC_APBRSTR1_USART3RST)

/**
 * @brief   Enables the UART4 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART4(lp) rccEnableAPBR1(RCC_APBENR1_USART4EN, lp)

/**
 * @brief   Disables the UART4 peripheral clock.
 *
 * @api
 */
#define rccDisableUART4() rccDisableAPBR1(RCC_APBENR1_USART4EN)

/**
 * @brief   Resets the UART4 peripheral.
 *
 * @api
 */
#define rccResetUART4() rccResetAPBR1(RCC_APBRSTR1_USART4RST)

/**
 * @brief   Enables the UART5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUART5(lp) rccEnableAPBR1(RCC_APBENR1_USART5EN, lp)

/**
 * @brief   Disables the UART5 peripheral clock.
 *
 * @api
 */
#define rccDisableUART5() rccDisableAPBR1(RCC_APBENR1_USART5EN)

/**
 * @brief   Resets the UART5 peripheral.
 *
 * @api
 */
#define rccResetUART5() rccResetAPBR1(RCC_APBRSTR1_USART5RST)

/**
 * @brief   Enables the USART6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART6(lp) rccEnableAPBR1(RCC_APBENR1_USART6EN, lp)

/**
 * @brief   Disables the USART6 peripheral clock.
 *
 * @api
 */
#define rccDisableUSART6() rccDisableAPBR1(RCC_APBENR1_USART6EN)

/**
 * @brief   Resets the USART6 peripheral.
 *
 * @api
 */
#define rccResetUSART6() rccResetAPBR1(RCC_APBRSTR1_USART6RST)

/**
 * @brief   Enables the LPUART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableLPUART1(lp) rccEnableAPBR1(RCC_APBENR1_LPUART1EN, lp)

/**
 * @brief   Disables the LPUART1 peripheral clock.
 *
 * @api
 */
#define rccDisableLPUART1() rccDisableAPBR1(RCC_APBENR1_LPUART1EN)

/**
 * @brief   Resets the LPUART1 peripheral.
 *
 * @api
 */
#define rccResetLPUART1() rccResetAPBR1(RCC_APBRSTR1_LPUART1RST)

/**
 * @brief   Enables the LPUART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableLPUART2(lp) rccEnableAPBR1(RCC_APBENR1_LPUART2EN, lp)

/**
 * @brief   Disables the LPUART2 peripheral clock.
 *
 * @api
 */
#define rccDisableLPUART2() rccDisableAPBR1(RCC_APBENR1_LPUART2EN)

/**
 * @brief   Resets the LPUART2 peripheral.
 *
 * @api
 */
#define rccResetLPUART2() rccResetAPBR1(RCC_APBRSTR1_LPUART2RST)
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
#define rccEnableUSB(lp) rccEnableAPBR1(RCC_APBENR1_USBEN, lp)

/**
 * @brief   Disables the USB peripheral clock.
 *
 * @api
 */
#define rccDisableUSB() rccDisableAPBR1(RCC_APBENR1_USBEN)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rccResetUSB() rccResetAPBR1(RCC_APBRSTR1_USBRST)
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
#define rccEnableCRC(lp) rccEnableAHB(RCC_AHBENR_CRCEN, lp)

/**
 * @brief   Disables the CRC peripheral clock.
 *
 * @api
 */
#define rccDisableCRC() rccDisableAHB(RCC_AHBENR_CRCEN)

/**
 * @brief   Resets the CRC peripheral.
 *
 * @api
 */
#define rccResetCRC() rccResetAHB(RCC_AHBRSTR_CRCRST)
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
