[#ftl]
[#--
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  --]
[@pp.dropOutputFile /]
[#import "/@lib/libutils.ftl" as utils /]
[#import "/@lib/liblicense.ftl" as license /]
[@pp.changeOutputFile name="mcuconf.h" /]
/*
[@license.EmitLicenseAsText /]
*/

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * STM32U3xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define STM32U3xx_MCUCONF
#define STM32U375_MCUCONF
#define STM32U385_MCUCONF

/*
 * HAL driver general settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_CLOCK_DYNAMIC                 ${doc.STM32_CLOCK_DYNAMIC!"FALSE"}

/*
 * ICache settings.
 */
#define STM32_ICACHE_CR                     ${doc.STM32_ICACHE_CR!"(ICACHE_CR_WAYSEL | ICACHE_CR_EN)"}
#define STM32_ICACHE_CRR0                   ${doc.STM32_ICACHE_CRR0!"(0U)"}
#define STM32_ICACHE_CRR1                   ${doc.STM32_ICACHE_CRR1!"(0U)"}
#define STM32_ICACHE_CRR2                   ${doc.STM32_ICACHE_CRR2!"(0U)"}
#define STM32_ICACHE_CRR3                   ${doc.STM32_ICACHE_CRR3!"(0U)"}

/*
 * PWR settings.
 */
#define STM32_PWR_VOSR                      ${doc.STM32_PWR_VOSR!"(PWR_VOSR_RANGE1)"}
#define STM32_PWR_CR3                       ${doc.STM32_PWR_CR3!"(PWR_CR3_FSTEN | PWR_CR3_REGSEL)"}
#define STM32_PWR_SVMCR                     ${doc.STM32_PWR_SVMCR!"(PWR_SVMCR_ASV | PWR_SVMCR_USV | PWR_SVMCR_AVM1EN | PWR_SVMCR_AVM2EN | PWR_SVMCR_UVMEN)"}
#define STM32_PWR_WUCR1                     ${doc.STM32_PWR_WUCR1!"(0U)"}
#define STM32_PWR_WUCR2                     ${doc.STM32_PWR_WUCR2!"(0U)"}
#define STM32_PWR_WUCR3                     ${doc.STM32_PWR_WUCR3!"(0U)"}
#define STM32_PWR_BDCR                      ${doc.STM32_PWR_BDCR!"(0U)"}
#define STM32_PWR_APCR                      ${doc.STM32_PWR_APCR!"(0U)"}
#define STM32_PWR_PUCRA                     ${doc.STM32_PWR_PUCRA!"(0U)"}
#define STM32_PWR_PDCRA                     ${doc.STM32_PWR_PDCRA!"(0U)"}
#define STM32_PWR_PUCRB                     ${doc.STM32_PWR_PUCRB!"(0U)"}
#define STM32_PWR_PDCRB                     ${doc.STM32_PWR_PDCRB!"(0U)"}
#define STM32_PWR_PUCRC                     ${doc.STM32_PWR_PUCRC!"(0U)"}
#define STM32_PWR_PDCRC                     ${doc.STM32_PWR_PDCRC!"(0U)"}
#define STM32_PWR_PUCRD                     ${doc.STM32_PWR_PUCRD!"(0U)"}
#define STM32_PWR_PDCRD                     ${doc.STM32_PWR_PDCRD!"(0U)"}
#define STM32_PWR_PUCRE                     ${doc.STM32_PWR_PUCRE!"(0U)"}
#define STM32_PWR_PDCRE                     ${doc.STM32_PWR_PDCRE!"(0U)"}
#define STM32_PWR_PUCRG                     ${doc.STM32_PWR_PUCRG!"(0U)"}
#define STM32_PWR_PDCRG                     ${doc.STM32_PWR_PDCRG!"(0U)"}
#define STM32_PWR_PUCRH                     ${doc.STM32_PWR_PUCRH!"(0U)"}
#define STM32_PWR_PDCRH                     ${doc.STM32_PWR_PDCRH!"(0U)"}
#define STM32_PWR_I3CPUCR1                  ${doc.STM32_PWR_I3CPUCR1!"(0U)"}
#define STM32_PWR_I3CPUCR2                  ${doc.STM32_PWR_I3CPUCR2!"(0U)"}
#define STM32_PWR_SECCFGR                   ${doc.STM32_PWR_SECCFGR!"(0U)"}
#define STM32_PWR_PRIVCFGR                  ${doc.STM32_PWR_PRIVCFGR!"(0U)"}

/*
 * FLASH settings.
 */
#define STM32_FLASH_ACR                     ${doc.STM32_FLASH_ACR!"(FLASH_ACR_PRFTEN)"}

/*
 * Clock settings.
 */
#define STM32_HSI16_ENABLED                 ${doc.STM32_HSI16_ENABLED!"FALSE"}
#define STM32_HSIKERON_ENABLED              ${doc.STM32_HSIKERON_ENABLED!"FALSE"}
#define STM32_HSI48_ENABLED                 ${doc.STM32_HSI48_ENABLED!"FALSE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"FALSE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"FALSE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"TRUE"}
#define STM32_MSIRC0_MODE                   ${doc.STM32_MSIRC0_MODE!"RCC_MSIRC0_PLL_LSE"}
#define STM32_MSIRC1_MODE                   ${doc.STM32_MSIRC1_MODE!"RCC_MSIRC1_FREE"}
#define STM32_MSIPLL1N_VALUE                ${doc.STM32_MSIPLL1N_VALUE!"0"}
#define STM32_MSIS_SRCDIV                   ${doc.STM32_MSIS_SRCDIV!"RCC_ICSCR1_MSIS_IRC0_DIV1"}
#define STM32_MSIK_SRCDIV                   ${doc.STM32_MSIK_SRCDIV!"RCC_ICSCR1_MSIK_IRC0_DIV1"}
#define STM32_MSIBIAS                       ${doc.STM32_MSIBIAS!"RCC_ICSCR1_MSIBIAS_CONTINUOUS"}
#define STM32_SW                            ${doc.STM32_SW!"RCC_CFGR1_SW_MSIS"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"RCC_CFGR2_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"RCC_CFGR2_PPRE1_DIV1"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"RCC_CFGR2_PPRE2_DIV1"}
#define STM32_PPRE3                         ${doc.STM32_PPRE3!"RCC_CFGR3_PPRE3_DIV1"}
#define STM32_STOPWUCK                      ${doc.STM32_STOPWUCK!"RCC_CFGR1_STOPWUCK_MSIS"}
#define STM32_STOPKERWUCK                   ${doc.STM32_STOPKERWUCK!"RCC_CFGR1_STOPKERWUCK_MSIK"}
#define STM32_MCO1SEL                       ${doc.STM32_MCO1SEL!"RCC_CFGR1_MCO1SEL_OFF"}
#define STM32_MCO1PRE_VALUE                 ${doc.STM32_MCO1PRE_VALUE!"1"}
#define STM32_MCO2SEL                       ${doc.STM32_MCO2SEL!"RCC_CFGR1_MCO2SEL_OFF"}
#define STM32_MCO2PRE_VALUE                 ${doc.STM32_MCO2PRE_VALUE!"1"}
#define STM32_LSCOSEL                       ${doc.STM32_LSCOSEL!"RCC_BDCR_LSCOSEL_NOCLOCK"}

/*
 * Peripherals clock sources.
 */
#define STM32_TIMICSEL                      ${doc.STM32_TIMICSEL!"RCC_CCIPR1_TIMICSEL_NOCLOCK"}
#define STM32_USART1SEL                     ${doc.STM32_USART1SEL!"RCC_CCIPR1_USART1SEL_PCLK2"}
#define STM32_USART3SEL                     ${doc.STM32_USART3SEL!"RCC_CCIPR1_USART3SEL_PCLK1"}
#define STM32_UART4SEL                      ${doc.STM32_UART4SEL!"RCC_CCIPR1_UART4SEL_PCLK1"}
#define STM32_UART5SEL                      ${doc.STM32_UART5SEL!"RCC_CCIPR1_UART5SEL_PCLK1"}
#define STM32_LPUART1SEL                    ${doc.STM32_LPUART1SEL!"RCC_CCIPR3_LPUART1SEL_PCLK3"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"RCC_CCIPR3_LPTIM1SEL_MSIK"}
#define STM32_LPTIM2SEL                     ${doc.STM32_LPTIM2SEL!"RCC_CCIPR1_LPTIM2SEL_PCLK1"}
#define STM32_LPTIM34SEL                    ${doc.STM32_LPTIM34SEL!"RCC_CCIPR3_LPTIM34SEL_MSIK"}
#define STM32_SPI1SEL                       ${doc.STM32_SPI1SEL!"RCC_CCIPR1_SPI1SEL_PCLK2"}
#define STM32_SPI2SEL                       ${doc.STM32_SPI2SEL!"RCC_CCIPR1_SPI2SEL_PCLK2"}
#define STM32_SPI3SEL                       ${doc.STM32_SPI3SEL!"RCC_CCIPR2_SPI3SEL_PCLK1"}
#define STM32_OCTOSPISEL                    ${doc.STM32_OCTOSPISEL!"RCC_CCIPR2_OCTOSPISEL_SYSCLK"}
#define STM32_SYSTICKSEL                    ${doc.STM32_SYSTICKSEL!"RCC_CCIPR1_SYSTICKSEL_HCLKDIV8"}
#define STM32_ICLKSEL                       ${doc.STM32_ICLKSEL!"RCC_CCIPR1_ICLKSEL_SYSCLK"}
#define STM32_USB1SEL                       ${doc.STM32_USB1SEL!"RCC_CCIPR1_USB1SEL_ICLKDIV2"}
#define STM32_I2C1SEL                       ${doc.STM32_I2C1SEL!"RCC_CCIPR1_I2C1SEL_PCLK1"}
#define STM32_I2C2SEL                       ${doc.STM32_I2C2SEL!"RCC_CCIPR1_I2C2SEL_PCLK1"}
#define STM32_I2C3SEL                       ${doc.STM32_I2C3SEL!"RCC_CCIPR3_I2C3SEL_PCLK3"}
#define STM32_I3C1SEL                       ${doc.STM32_I3C1SEL!"RCC_CCIPR1_I3C1SEL_PCLK1"}
#define STM32_I3C2SEL                       ${doc.STM32_I3C2SEL!"RCC_CCIPR1_I3C2SEL_PCLK1"}
#define STM32_ADCDACSEL                     ${doc.STM32_ADCDACSEL!"RCC_CCIPR2_ADCDACSEL_HCLK"}
#define STM32_DAC1SHSEL                     ${doc.STM32_DAC1SHSEL!"RCC_CCIPR2_DAC1SHSEL_IGNORE"}
#define STM32_ADCDACPRE                     ${doc.STM32_ADCDACPRE!"RCC_CCIPR2_ADCDACPRE_ICLK"}
#define STM32_RNGSEL                        ${doc.STM32_RNGSEL!"RCC_CCIPR2_RNGSEL_IGNORE"}
#define STM32_FDCAN1SEL                     ${doc.STM32_FDCAN1SEL!"RCC_CCIPR1_FDCAN1SEL_SYSCLK"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"RCC_CCIPR2_SAI1SEL_MSIK"}
#define STM32_ADF1SEL                       ${doc.STM32_ADF1SEL!"RCC_CCIPR2_ADF1SEL_HCLK"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"RCC_BDCR_RTCSEL_NOCLOCK"}

/*
 * IRQ system settings.
 */
#define STM32_IRQ_DAC1_PRIORITY             ${doc.STM32_IRQ_DAC1_PRIORITY!"9"}

#define STM32_IRQ_EXTI0_PRIORITY            ${doc.STM32_IRQ_EXTI0_PRIORITY!"6"}
#define STM32_IRQ_EXTI1_PRIORITY            ${doc.STM32_IRQ_EXTI1_PRIORITY!"6"}
#define STM32_IRQ_EXTI2_PRIORITY            ${doc.STM32_IRQ_EXTI2_PRIORITY!"6"}
#define STM32_IRQ_EXTI3_PRIORITY            ${doc.STM32_IRQ_EXTI3_PRIORITY!"6"}
#define STM32_IRQ_EXTI4_PRIORITY            ${doc.STM32_IRQ_EXTI4_PRIORITY!"6"}
#define STM32_IRQ_EXTI5_PRIORITY            ${doc.STM32_IRQ_EXTI5_PRIORITY!"6"}
#define STM32_IRQ_EXTI6_PRIORITY            ${doc.STM32_IRQ_EXTI6_PRIORITY!"6"}
#define STM32_IRQ_EXTI7_PRIORITY            ${doc.STM32_IRQ_EXTI7_PRIORITY!"6"}
#define STM32_IRQ_EXTI8_PRIORITY            ${doc.STM32_IRQ_EXTI8_PRIORITY!"6"}
#define STM32_IRQ_EXTI9_PRIORITY            ${doc.STM32_IRQ_EXTI9_PRIORITY!"6"}
#define STM32_IRQ_EXTI10_PRIORITY           ${doc.STM32_IRQ_EXTI10_PRIORITY!"6"}
#define STM32_IRQ_EXTI11_PRIORITY           ${doc.STM32_IRQ_EXTI11_PRIORITY!"6"}
#define STM32_IRQ_EXTI12_PRIORITY           ${doc.STM32_IRQ_EXTI12_PRIORITY!"6"}
#define STM32_IRQ_EXTI13_PRIORITY           ${doc.STM32_IRQ_EXTI13_PRIORITY!"6"}
#define STM32_IRQ_EXTI14_PRIORITY           ${doc.STM32_IRQ_EXTI14_PRIORITY!"6"}
#define STM32_IRQ_EXTI15_PRIORITY           ${doc.STM32_IRQ_EXTI15_PRIORITY!"6"}

#define STM32_IRQ_FDCAN1_PRIORITY           ${doc.STM32_IRQ_FDCAN1_PRIORITY!"10"}

#define STM32_IRQ_I2C1_PRIORITY             ${doc.STM32_IRQ_I2C1_PRIORITY!"5"}
#define STM32_IRQ_I2C2_PRIORITY             ${doc.STM32_IRQ_I2C2_PRIORITY!"5"}
#define STM32_IRQ_I2C3_PRIORITY             ${doc.STM32_IRQ_I2C3_PRIORITY!"5"}

#define STM32_IRQ_SPI1_PRIORITY             ${doc.STM32_IRQ_SPI1_PRIORITY!"10"}
#define STM32_IRQ_SPI2_PRIORITY             ${doc.STM32_IRQ_SPI2_PRIORITY!"10"}
#define STM32_IRQ_SPI3_PRIORITY             ${doc.STM32_IRQ_SPI3_PRIORITY!"10"}

#define STM32_IRQ_TIM1_BRK_PRIORITY         ${doc.STM32_IRQ_TIM1_BRK_PRIORITY!"7"}
#define STM32_IRQ_TIM1_UP_PRIORITY          ${doc.STM32_IRQ_TIM1_UP_PRIORITY!"7"}
#define STM32_IRQ_TIM1_TRGCO_PRIORITY       ${doc.STM32_IRQ_TIM1_TRGCO_PRIORITY!"7"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"7"}
#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"7"}
#define STM32_IRQ_TIM3_PRIORITY             ${doc.STM32_IRQ_TIM3_PRIORITY!"7"}
#define STM32_IRQ_TIM4_PRIORITY             ${doc.STM32_IRQ_TIM4_PRIORITY!"7"}
#define STM32_IRQ_TIM6_PRIORITY             ${doc.STM32_IRQ_TIM6_PRIORITY!"7"}
#define STM32_IRQ_TIM7_PRIORITY             ${doc.STM32_IRQ_TIM7_PRIORITY!"7"}
#define STM32_IRQ_TIM15_PRIORITY            ${doc.STM32_IRQ_TIM15_PRIORITY!"7"}
#define STM32_IRQ_TIM16_PRIORITY            ${doc.STM32_IRQ_TIM16_PRIORITY!"7"}
#define STM32_IRQ_TIM17_PRIORITY            ${doc.STM32_IRQ_TIM17_PRIORITY!"7"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"12"}
#define STM32_IRQ_USART3_PRIORITY           ${doc.STM32_IRQ_USART3_PRIORITY!"12"}
#define STM32_IRQ_UART4_PRIORITY            ${doc.STM32_IRQ_UART4_PRIORITY!"12"}
#define STM32_IRQ_UART5_PRIORITY            ${doc.STM32_IRQ_UART5_PRIORITY!"12"}
#define STM32_IRQ_LPUART1_PRIORITY          ${doc.STM32_IRQ_LPUART1_PRIORITY!"12"}

#define STM32_IRQ_USB1_PRIORITY             ${doc.STM32_IRQ_USB1_PRIORITY!"13"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_USE_ADC2                  ${doc.STM32_ADC_USE_ADC2!"FALSE"}
#define STM32_ADC_DUAL_MODE                 ${doc.STM32_ADC_DUAL_MODE!"FALSE"}
#define STM32_ADC_COMPACT_SAMPLES           ${doc.STM32_ADC_COMPACT_SAMPLES!"FALSE"}
#define STM32_ADC_ADC1_DMA3_CHANNEL         ${doc.STM32_ADC_ADC1_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_ADC_ADC2_DMA3_CHANNEL         ${doc.STM32_ADC_ADC2_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC2_DMA_PRIORITY         ${doc.STM32_ADC_ADC2_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC2_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC1_IRQ_PRIORITY         ${doc.STM32_ADC_ADC1_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC2_IRQ_PRIORITY         ${doc.STM32_ADC_ADC2_IRQ_PRIORITY!"5"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_FDCAN1                ${doc.STM32_CAN_USE_FDCAN1!"FALSE"}

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 ${doc.STM32_DAC_DUAL_MODE!"FALSE"}
#define STM32_DAC_USE_DAC1_CH1              ${doc.STM32_DAC_USE_DAC1_CH1!"FALSE"}
#define STM32_DAC_USE_DAC1_CH2              ${doc.STM32_DAC_USE_DAC1_CH2!"FALSE"}
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH2_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH1_DMA3_CHANNEL     ${doc.STM32_DAC_DAC1_CH1_DMA_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_DAC_DAC1_CH2_DMA3_CHANNEL     ${doc.STM32_DAC_DAC1_CH2_DMA_CHANNEL!"STM32_DMA3_MASK_FIFO2"}

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  ${doc.STM32_GPT_USE_TIM1!"FALSE"}
#define STM32_GPT_USE_TIM2                  ${doc.STM32_GPT_USE_TIM2!"FALSE"}
#define STM32_GPT_USE_TIM3                  ${doc.STM32_GPT_USE_TIM3!"FALSE"}
#define STM32_GPT_USE_TIM4                  ${doc.STM32_GPT_USE_TIM4!"FALSE"}
#define STM32_GPT_USE_TIM6                  ${doc.STM32_GPT_USE_TIM6!"FALSE"}
#define STM32_GPT_USE_TIM7                  ${doc.STM32_GPT_USE_TIM7!"FALSE"}
#define STM32_GPT_USE_TIM15                 ${doc.STM32_GPT_USE_TIM15!"FALSE"}
#define STM32_GPT_USE_TIM16                 ${doc.STM32_GPT_USE_TIM16!"FALSE"}
#define STM32_GPT_USE_TIM17                 ${doc.STM32_GPT_USE_TIM17!"FALSE"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_USE_I2C2                  ${doc.STM32_I2C_USE_I2C2!"FALSE"}
#define STM32_I2C_USE_I2C3                  ${doc.STM32_I2C_USE_I2C3!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C2_DMA_PRIORITY         ${doc.STM32_I2C_I2C2_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C3_DMA_PRIORITY         ${doc.STM32_I2C_I2C3_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C1_DMA3_CHANNEL         ${doc.STM32_I2C_I2C1_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_I2C_I2C2_DMA3_CHANNEL         ${doc.STM32_I2C_I2C2_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_I2C_I2C3_DMA3_CHANNEL         ${doc.STM32_I2C_I2C3_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      ${doc.STM32_I2C_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  ${doc.STM32_ICU_USE_TIM1!"FALSE"}
#define STM32_ICU_USE_TIM2                  ${doc.STM32_ICU_USE_TIM2!"FALSE"}
#define STM32_ICU_USE_TIM3                  ${doc.STM32_ICU_USE_TIM3!"FALSE"}
#define STM32_ICU_USE_TIM4                  ${doc.STM32_ICU_USE_TIM4!"FALSE"}
#define STM32_ICU_USE_TIM15                 ${doc.STM32_ICU_USE_TIM15!"FALSE"}
#define STM32_ICU_USE_TIM16                 ${doc.STM32_ICU_USE_TIM16!"FALSE"}
#define STM32_ICU_USE_TIM17                 ${doc.STM32_ICU_USE_TIM17!"FALSE"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  ${doc.STM32_PWM_USE_TIM1!"FALSE"}
#define STM32_PWM_USE_TIM2                  ${doc.STM32_PWM_USE_TIM2!"FALSE"}
#define STM32_PWM_USE_TIM3                  ${doc.STM32_PWM_USE_TIM3!"FALSE"}
#define STM32_PWM_USE_TIM4                  ${doc.STM32_PWM_USE_TIM4!"FALSE"}
#define STM32_PWM_USE_TIM15                 ${doc.STM32_PWM_USE_TIM15!"FALSE"}
#define STM32_PWM_USE_TIM16                 ${doc.STM32_PWM_USE_TIM16!"FALSE"}
#define STM32_PWM_USE_TIM17                 ${doc.STM32_PWM_USE_TIM17!"FALSE"}

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               ${doc.STM32_RTC_PRESA_VALUE!"32"}
#define STM32_RTC_PRESS_VALUE               ${doc.STM32_RTC_PRESS_VALUE!"1024"}
#define STM32_RTC_CR_INIT                   ${doc.STM32_RTC_CR_INIT!"0"}

/*
 * SDC driver system settings.
 */
#define STM32_SDC_USE_SDMMC1                ${doc.STM32_SDC_USE_SDMMC1!"FALSE"}
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   ${doc.STM32_SDC_SDMMC_UNALIGNED_SUPPORT!"TRUE"}
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       ${doc.STM32_SDC_SDMMC_WRITE_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_READ_TIMEOUT        ${doc.STM32_SDC_SDMMC_READ_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_CLOCK_DELAY         ${doc.STM32_SDC_SDMMC_CLOCK_DELAY!"10"}
#define STM32_SDC_SDMMC_PWRSAV              ${doc.STM32_SDC_SDMMC_PWRSAV!"TRUE"}

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART3             ${doc.STM32_SERIAL_USE_USART3!"FALSE"}
#define STM32_SERIAL_USE_UART4              ${doc.STM32_SERIAL_USE_UART4!"FALSE"}
#define STM32_SERIAL_USE_UART5              ${doc.STM32_SERIAL_USE_UART5!"FALSE"}
#define STM32_SERIAL_USE_LPUART1            ${doc.STM32_SERIAL_USE_LPUART1!"FALSE"}

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                ${doc.STM32_SIO_USE_USART1!"FALSE"}
#define STM32_SIO_USE_USART3                ${doc.STM32_SIO_USE_USART3!"FALSE"}
#define STM32_SIO_USE_UART4                 ${doc.STM32_SIO_USE_UART4!"FALSE"}
#define STM32_SIO_USE_UART5                 ${doc.STM32_SIO_USE_UART5!"FALSE"}
#define STM32_SIO_USE_LPUART1               ${doc.STM32_SIO_USE_LPUART1!"FALSE"}

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_USE_SPI2                  ${doc.STM32_SPI_USE_SPI2!"FALSE"}
#define STM32_SPI_USE_SPI3                  ${doc.STM32_SPI_USE_SPI3!"FALSE"}
#define STM32_SPI_SPI1_RX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI1_RX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI1_TX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI1_TX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI2_RX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI2_RX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI2_TX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI2_TX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI3_RX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI3_RX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI3_TX_DMA3_CHANNEL      ${doc.STM32_SPI_SPI3_TX_DMA3_CHANNEL!"STM32_DMA3_MASK_FIFO2"}
#define STM32_SPI_SPI1_DMA_PRIORITY         ${doc.STM32_SPI_SPI1_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI2_DMA_PRIORITY         ${doc.STM32_SPI_SPI2_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI3_DMA_PRIORITY         ${doc.STM32_SPI_SPI3_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI1_IRQ_PRIORITY         ${doc.STM32_SPI_SPI1_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI2_IRQ_PRIORITY         ${doc.STM32_SPI_SPI2_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI3_IRQ_PRIORITY         ${doc.STM32_SPI_SPI3_IRQ_PRIORITY!"10"}
#define STM32_SPI_DMA_ERROR_HOOK(spip)      ${doc.STM32_SPI_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               ${doc.STM32_ST_IRQ_PRIORITY!"8"}
#define STM32_ST_USE_TIMER                  ${doc.STM32_ST_USE_TIMER!"2"}

/*
 * TRNG driver system settings.
 */
#define STM32_TRNG_USE_RNG1                 ${doc.STM32_TRNG_USE_RNG1!"FALSE"}

/*
 * UART driver system settings.
 */

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_USB1                  ${doc.STM32_USB_USE_USB1!"FALSE"}
#define STM32_USB_USE_ISOCHRONOUS           ${doc.STM32_USB_USE_ISOCHRONOUS!"FALSE"}
#define STM32_USB_USE_FAST_COPY             ${doc.STM32_USB_USE_FAST_COPY!"FALSE"}
#define STM32_USB_HOST_WAKEUP_DURATION      ${doc.STM32_USB_HOST_WAKEUP_DURATION!"2"}
#define STM32_USB_48MHZ_DELTA               ${doc.STM32_USB_48MHZ_DELTA!"120000"}

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

/*
 * WSPI driver system settings.
 */

#endif /* MCUCONF_H */
