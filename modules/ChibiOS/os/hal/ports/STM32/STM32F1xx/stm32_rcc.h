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
 * @file    STM32F1xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32f10x.h.
 *
 * @addtogroup STM32F1xx_RCC
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
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1(mask, lp) {                                           \
  RCC->APB1ENR |= (mask);                                                   \
  (void)RCC->APB1ENR;                                                       \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rccDisableAPB1(mask) {                                              \
  RCC->APB1ENR &= ~(mask);                                                  \
  (void)RCC->APB1ENR;                                                       \
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
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB2(mask, lp) {                                           \
  RCC->APB2ENR |= (mask);                                                   \
  (void)RCC->APB2ENR;                                                       \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB2 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccDisableAPB2(mask) {                                              \
  RCC->APB2ENR &= ~(mask);                                                  \
  (void)RCC->APB2ENR;                                                       \
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
 * @brief   Enables the clock of one or more peripheral on the AHB bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      AHB peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB(mask, lp) {                                            \
  RCC->AHBENR |= (mask);                                                    \
  (void)RCC->AHBENR;                                                        \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rccDisableAHB(mask) {                                               \
  RCC->AHBENR &= ~(mask);                                                   \
  (void)RCC->AHBENR;                                                        \
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
 * @brief   Enables the ADC1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @brief   Resets the ADC1 peripheral.
 *
 * @api
 */
#define rccResetADC1() rccResetAPB2(RCC_APB2RSTR_ADC1RST)
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
 * @name    Backup domain interface specific RCC operations
 * @{
 */
/**
 * @brief   Enables the BKP interface clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableBKPInterface(lp) rccEnableAPB1((RCC_APB1ENR_BKPEN), lp)

/**
 * @brief   Disables BKP interface clock.
 *
 * @api
 */
#define rccDisableBKPInterface() rccDisableAPB1(RCC_APB1ENR_BKPEN)

/**
 * @brief   Resets the Backup Domain interface.
 *
 * @api
 */
#define rccResetBKPInterface() rccResetAPB1(RCC_APB1ENR_BKPRST)

/**
 * @brief   Resets the entire Backup Domain.
 *
 * @api
 */
#define rccResetBKP() (RCC->BDCR |= RCC_BDCR_BDRST)
/** @} */

/**
 * @name    PWR interface specific RCC operations
 * @{
 */
/**
 * @brief   Enables the PWR interface clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
/** @} */

/**
 * @name    DMA peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DMA1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    Not supported in this family, does nothing.
 *
 * @api
 */
#define rccResetDMA1()

/**
 * @brief   Enables the DMA2 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @brief   Resets the DMA1 peripheral.
 * @note    Not supported in this family, does nothing.
 *
 * @api
 */
#define rccResetDMA2()
/** @} */

/**
 * @name    ETH peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the ETH peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableETH(lp) rccEnableAHB(RCC_AHBENR_ETHMACEN |                 \
                                      RCC_AHBENR_ETHMACTXEN |               \
                                      RCC_AHBENR_ETHMACRXEN, lp)

/**
 * @brief   Disables the ETH peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableETH() rccDisableAHB(RCC_AHBENR_ETHMACEN |                 \
                                      RCC_AHBENR_ETHMACTXEN |               \
                                      RCC_AHBENR_ETHMACRXEN)

/**
 * @brief   Resets the ETH peripheral.
 *
 * @api
 */
#define rccResetETH() rccResetAHB(RCC_AHBRSTR_ETHMACRST)
/** @} */

/**
 * @name    I2C peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the I2C1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
#define rccEnableOTG_FS(lp) rccEnableAHB(RCC_AHBENR_OTGFSEN, lp)

/**
 * @brief   Disables the OTG_FS peripheral clock.
 *
 * @api
 */
#define rccDisableOTG_FS() rccDisableAHB(RCC_AHBENR_OTGFSEN)

/**
 * @brief   Resets the OTG_FS peripheral.
 *
 * @api
 */
#define rccResetOTG_FS() rccResetAHB(RCC_AHBRSTR_OTGFSRST)
/** @} */

/**
 * @name    SDIO peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SDIO peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSDIO(lp) rccEnableAHB(RCC_AHBENR_SDIOEN, lp)

/**
 * @brief   Disables the SDIO peripheral clock.
 *
 * @api
 */
#define rccDisableSDIO() rccDisableAHB(RCC_AHBENR_SDIOEN)

/**
 * @brief   Resets the SDIO peripheral.
 * @note    Not supported in this family, does nothing.
 *
 * @api
 */
#define rccResetSDIO()
/** @} */

/**
 * @name    SPI peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SPI1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
/** @} */

/**
 * @name    TIM peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the TIM1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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

 * @note    The @p lp parameter is ignored in this family.

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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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

/**
 * @brief   Enables the TIM15 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
 * @note    The @p lp parameter is ignored in this family.
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
/** @} */

/**
 * @name    USB peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the USB peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSB(lp) rccEnableAPB1(RCC_APB1ENR_USBEN, lp)

/**
 * @brief   Disables the USB peripheral clock
 *
 * @api
 */
#define rccDisableUSB() rccDisableAPB1(RCC_APB1ENR_USBEN)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rccResetUSB() rccResetAPB1(RCC_APB1RSTR_USBRST)
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
#define rccEnableFSMC(lp) rccEnableAHB(RCC_AHBENR_FSMCEN, lp)

/**
 * @brief   Disables the FSMC peripheral clock.
 *
 * @api
 */
#define rccDisableFSMC() rccDisableAHB(RCC_AHBENR_FSMCEN)
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
