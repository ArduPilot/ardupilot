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
 * STM32H723/33/25/35 drivers configuration.
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

#define STM32H7xx_MCUCONF
#define STM32H723_MCUCONF
#define STM32H733_MCUCONF
#define STM32H725_MCUCONF
#define STM32H735_MCUCONF

/*
 * General settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}

/*
 * Memory attributes settings.
 */
#define STM32_NOCACHE_ENABLE                ${doc.STM32_NOCACHE_ENABLE!"FALSE"}
#define STM32_NOCACHE_MPU_REGION            ${doc.STM32_NOCACHE_MPU_REGION!"MPU_REGION_6"}
#define STM32_NOCACHE_RBAR                  ${doc.STM32_NOCACHE_RBAR!"0x24000000U"}
#define STM32_NOCACHE_RASR                  ${doc.STM32_NOCACHE_RASR!"MPU_RASR_SIZE_16K"}

/*
 * PWR system settings.
 * Reading STM32 Reference Manual is required, settings in PWR_CR3 are
 * very critical.
 * Register constants are taken from the ST header.
 */
#define STM32_VOS                           ${doc.STM32_VOS!"STM32_VOS_SCALE0"}
#define STM32_PWR_CR1                       ${doc.STM32_PWR_CR1!"(PWR_CR1_SVOS_1 | PWR_CR1_SVOS_0)"}
#define STM32_PWR_CR2                       ${doc.STM32_PWR_CR2!"(PWR_CR2_BREN)"}
#define STM32_PWR_CR3                       ${doc.STM32_PWR_CR3!"(PWR_CR3_LDOEN | PWR_CR3_USB33DEN)"}
#define STM32_PWR_CPUCR                     ${doc.STM32_PWR_CPUCR!"0"}

/*
 * Clock tree static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_HSI_ENABLED                   ${doc.STM32_HSI_ENABLED!"TRUE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"TRUE"}
#define STM32_CSI_ENABLED                   ${doc.STM32_CSI_ENABLED!"TRUE"}
#define STM32_HSI48_ENABLED                 ${doc.STM32_HSI48_ENABLED!"TRUE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"TRUE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"TRUE"}
#define STM32_HSIDIV                        ${doc.STM32_HSIDIV!"STM32_HSIDIV_DIV1"}

/*
 * PLLs static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_HSE_CK"}
#define STM32_PLLCFGR_MASK                  ${doc.STM32_PLLCFGR_MASK!"~0"}
#define STM32_PLL1_ENABLED                  ${doc.STM32_PLL1_ENABLED!"TRUE"}
#define STM32_PLL1_P_ENABLED                ${doc.STM32_PLL1_P_ENABLED!"TRUE"}
#define STM32_PLL1_Q_ENABLED                ${doc.STM32_PLL1_Q_ENABLED!"TRUE"}
#define STM32_PLL1_R_ENABLED                ${doc.STM32_PLL1_R_ENABLED!"TRUE"}
#define STM32_PLL1_DIVM_VALUE               ${doc.STM32_PLL1_DIVM_VALUE!"4"}
#define STM32_PLL1_DIVN_VALUE               ${doc.STM32_PLL1_DIVN_VALUE!"260"}
#define STM32_PLL1_FRACN_VALUE              ${doc.STM32_PLL1_FRACN_VALUE!"0"}
#define STM32_PLL1_DIVP_VALUE               ${doc.STM32_PLL1_DIVP_VALUE!"1"}
#define STM32_PLL1_DIVQ_VALUE               ${doc.STM32_PLL1_DIVQ_VALUE!"10"}
#define STM32_PLL1_DIVR_VALUE               ${doc.STM32_PLL1_DIVR_VALUE!"4"}
#define STM32_PLL2_ENABLED                  ${doc.STM32_PLL2_ENABLED!"TRUE"}
#define STM32_PLL2_P_ENABLED                ${doc.STM32_PLL2_P_ENABLED!"TRUE"}
#define STM32_PLL2_Q_ENABLED                ${doc.STM32_PLL2_Q_ENABLED!"TRUE"}
#define STM32_PLL2_R_ENABLED                ${doc.STM32_PLL2_R_ENABLED!"TRUE"}
#define STM32_PLL2_DIVM_VALUE               ${doc.STM32_PLL2_DIVM_VALUE!"4"}
#define STM32_PLL2_DIVN_VALUE               ${doc.STM32_PLL2_DIVN_VALUE!"400"}
#define STM32_PLL2_FRACN_VALUE              ${doc.STM32_PLL2_FRACN_VALUE!"0"}
#define STM32_PLL2_DIVP_VALUE               ${doc.STM32_PLL2_DIVP_VALUE!"40"}
#define STM32_PLL2_DIVQ_VALUE               ${doc.STM32_PLL2_DIVQ_VALUE!"8"}
#define STM32_PLL2_DIVR_VALUE               ${doc.STM32_PLL2_DIVR_VALUE!"8"}
#define STM32_PLL3_ENABLED                  ${doc.STM32_PLL3_ENABLED!"TRUE"}
#define STM32_PLL3_P_ENABLED                ${doc.STM32_PLL3_P_ENABLED!"TRUE"}
#define STM32_PLL3_Q_ENABLED                ${doc.STM32_PLL3_Q_ENABLED!"TRUE"}
#define STM32_PLL3_R_ENABLED                ${doc.STM32_PLL3_R_ENABLED!"TRUE"}
#define STM32_PLL3_DIVM_VALUE               ${doc.STM32_PLL3_DIVM_VALUE!"4"}
#define STM32_PLL3_DIVN_VALUE               ${doc.STM32_PLL3_DIVN_VALUE!"240"}
#define STM32_PLL3_FRACN_VALUE              ${doc.STM32_PLL3_FRACN_VALUE!"0"}
#define STM32_PLL3_DIVP_VALUE               ${doc.STM32_PLL3_DIVP_VALUE!"10"}
#define STM32_PLL3_DIVQ_VALUE               ${doc.STM32_PLL3_DIVQ_VALUE!"10"}
#define STM32_PLL3_DIVR_VALUE               ${doc.STM32_PLL3_DIVR_VALUE!"10"}

/*
 * Core clocks dynamic settings (can be changed at runtime).
 * Reading STM32 Reference Manual is required.
 */
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL1_P_CK"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSE_CK"}
#define STM32_D1CPRE                        ${doc.STM32_D1CPRE!"STM32_D1CPRE_DIV1"}
#define STM32_D1HPRE                        ${doc.STM32_D1HPRE!"STM32_D1HPRE_DIV2"}
#define STM32_D1PPRE3                       ${doc.STM32_D1PPRE3!"STM32_D1PPRE3_DIV2"}
#define STM32_D2PPRE1                       ${doc.STM32_D2PPRE1!"STM32_D2PPRE1_DIV2"}
#define STM32_D2PPRE2                       ${doc.STM32_D2PPRE2!"STM32_D2PPRE2_DIV2"}
#define STM32_D3PPRE4                       ${doc.STM32_D3PPRE4!"STM32_D3PPRE4_DIV2"}

/*
 * Peripherals clocks static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_MCO1SEL                       ${doc.STM32_MCO1SEL!"STM32_MCO1SEL_HSI_CK"}
#define STM32_MCO1PRE_VALUE                 ${doc.STM32_MCO1PRE_VALUE!"4"}
#define STM32_MCO2SEL                       ${doc.STM32_MCO2SEL!"STM32_MCO2SEL_SYS_CK"}
#define STM32_MCO2PRE_VALUE                 ${doc.STM32_MCO2PRE_VALUE!"4"}
#define STM32_TIMPRE_ENABLE                 ${doc.STM32_TIMPRE_ENABLE!"TRUE"}
#define STM32_HRTIMSEL                      ${doc.STM32_HRTIMSEL!"0"}
#define STM32_STOPKERWUCK                   ${doc.STM32_STOPKERWUCK!"0"}
#define STM32_STOPWUCK                      ${doc.STM32_STOPWUCK!"0"}
#define STM32_RTCPRE_VALUE                  ${doc.STM32_RTCPRE_VALUE!"8"}
#define STM32_CKPERSEL                      ${doc.STM32_CKPERSEL!"STM32_CKPERSEL_HSE_CK"}
#define STM32_SDMMCSEL                      ${doc.STM32_SDMMCSEL!"STM32_SDMMCSEL_PLL1_Q_CK"}
#define STM32_OCTOSPISEL                    ${doc.STM32_OCTOSPISEL!"STM32_OCTOSPISEL_HCLK"}
#define STM32_FMCSEL                        ${doc.STM32_FMCSEL!"STM32_FMCSEL_HCLK"}
#define STM32_SWPSEL                        ${doc.STM32_SWPSEL!"STM32_SWPSEL_PCLK1"}
#define STM32_FDCANSEL                      ${doc.STM32_FDCANSEL!"STM32_FDCANSEL_HSE_CK"}
#define STM32_DFSDM1SEL                     ${doc.STM32_DFSDM1SEL!"STM32_DFSDM1SEL_PCLK2"}
#define STM32_SPDIFSEL                      ${doc.STM32_SPDIFSEL!"STM32_SPDIFSEL_PLL1_Q_CK"}
#define STM32_SPI45SEL                      ${doc.STM32_SPI45SEL!"STM32_SPI45SEL_PCLK2"}
#define STM32_SPI123SEL                     ${doc.STM32_SPI123SEL!"STM32_SPI123SEL_PLL1_Q_CK"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"STM32_SAI1SEL_PLL1_Q_CK"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"STM32_LPTIM1SEL_PCLK1"}
#define STM32_CECSEL                        ${doc.STM32_CECSEL!"STM32_CECSEL_LSE_CK"}
#define STM32_USBSEL                        ${doc.STM32_USBSEL!"STM32_USBSEL_PLL3_Q_CK"}
#define STM32_I2C1235SEL                    ${doc.STM32_I2C1235SEL!"STM32_I2C1235SEL_PCLK1"}
#define STM32_RNGSEL                        ${doc.STM32_RNGSEL!"STM32_RNGSEL_HSI48_CK"}
#define STM32_USART16910SEL                 ${doc.STM32_USART16910SEL!"STM32_USART16910SEL_PCLK2"}
#define STM32_USART234578SEL                ${doc.STM32_USART234578SEL!"STM32_USART234578SEL_PCLK1"}
#define STM32_SPI6SEL                       ${doc.STM32_SPI6SEL!"STM32_SPI6SEL_PCLK4"}
#define STM32_SAI4BSEL                      ${doc.STM32_SAI4BSEL!"STM32_SAI4BSEL_PLL1_Q_CK"}
#define STM32_SAI4ASEL                      ${doc.STM32_SAI4ASEL!"STM32_SAI4ASEL_PLL1_Q_CK"}
#define STM32_ADCSEL                        ${doc.STM32_ADCSEL!"STM32_ADCSEL_PLL2_P_CK"}
#define STM32_LPTIM345SEL                   ${doc.STM32_LPTIM345SEL!"STM32_LPTIM345SEL_PCLK4"}
#define STM32_LPTIM2SEL                     ${doc.STM32_LPTIM2SEL!"STM32_LPTIM2SEL_PCLK4"}
#define STM32_I2C4SEL                       ${doc.STM32_I2C4SEL!"STM32_I2C4SEL_PCLK4"}
#define STM32_LPUART1SEL                    ${doc.STM32_LPUART1SEL!"STM32_LPUART1SEL_PCLK4"}

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            ${doc.STM32_IRQ_EXTI0_PRIORITY!"6"}
#define STM32_IRQ_EXTI1_PRIORITY            ${doc.STM32_IRQ_EXTI1_PRIORITY!"6"}
#define STM32_IRQ_EXTI2_PRIORITY            ${doc.STM32_IRQ_EXTI2_PRIORITY!"6"}
#define STM32_IRQ_EXTI3_PRIORITY            ${doc.STM32_IRQ_EXTI3_PRIORITY!"6"}
#define STM32_IRQ_EXTI4_PRIORITY            ${doc.STM32_IRQ_EXTI4_PRIORITY!"6"}
#define STM32_IRQ_EXTI5_9_PRIORITY          ${doc.STM32_IRQ_EXTI5_9_PRIORITY!"6"}
#define STM32_IRQ_EXTI10_15_PRIORITY        ${doc.STM32_IRQ_EXTI10_15_PRIORITY!"6"}
#define STM32_IRQ_EXTI16_PRIORITY           ${doc.STM32_IRQ_EXTI16_PRIORITY!"6"}
#define STM32_IRQ_EXTI17_PRIORITY           ${doc.STM32_IRQ_EXTI17_PRIORITY!"6"}
#define STM32_IRQ_EXTI18_PRIORITY           ${doc.STM32_IRQ_EXTI18_PRIORITY!"6"}
#define STM32_IRQ_EXTI19_PRIORITY           ${doc.STM32_IRQ_EXTI19_PRIORITY!"6"}
#define STM32_IRQ_EXTI20_21_PRIORITY        ${doc.STM32_IRQ_EXTI20_21_PRIORITY!"6"}

#define STM32_IRQ_FDCAN1_PRIORITY           ${doc.STM32_IRQ_FDCAN1_PRIORITY!"10"}
#define STM32_IRQ_FDCAN2_PRIORITY           ${doc.STM32_IRQ_FDCAN2_PRIORITY!"10"}
#define STM32_IRQ_FDCAN3_PRIORITY           ${doc.STM32_IRQ_FDCAN3_PRIORITY!"10"}

#define STM32_IRQ_MDMA_PRIORITY             ${doc.STM32_IRQ_MDMA_PRIORITY!"9"}

#define STM32_IRQ_OCTOSPI1_PRIORITY         ${doc.STM32_IRQ_OCTOSPI1_PRIORITY!"10"}
#define STM32_IRQ_OCTOSPI2_PRIORITY         ${doc.STM32_IRQ_OCTOSPI2_PRIORITY!"10"}

#define STM32_IRQ_SDMMC1_PRIORITY           ${doc.STM32_IRQ_SDMMC1_PRIORITY!"9"}
#define STM32_IRQ_SDMMC2_PRIORITY           ${doc.STM32_IRQ_SDMMC2_PRIORITY!"9"}

#define STM32_IRQ_TIM1_UP_PRIORITY          ${doc.STM32_IRQ_TIM1_UP_PRIORITY!"7"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"7"}
#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"7"}
#define STM32_IRQ_TIM3_PRIORITY             ${doc.STM32_IRQ_TIM3_PRIORITY!"7"}
#define STM32_IRQ_TIM4_PRIORITY             ${doc.STM32_IRQ_TIM4_PRIORITY!"7"}
#define STM32_IRQ_TIM5_PRIORITY             ${doc.STM32_IRQ_TIM5_PRIORITY!"7"}
#define STM32_IRQ_TIM6_PRIORITY             ${doc.STM32_IRQ_TIM6_PRIORITY!"7"}
#define STM32_IRQ_TIM7_PRIORITY             ${doc.STM32_IRQ_TIM7_PRIORITY!"7"}
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   ${doc.STM32_IRQ_TIM8_BRK_TIM12_PRIORITY!"7"}
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    ${doc.STM32_IRQ_TIM8_UP_TIM13_PRIORITY!"7"}
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY ${doc.STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY!"7"}
#define STM32_IRQ_TIM8_CC_PRIORITY          ${doc.STM32_IRQ_TIM8_CC_PRIORITY!"7"}
#define STM32_IRQ_TIM15_PRIORITY            ${doc.STM32_IRQ_TIM15_PRIORITY!"7"}
#define STM32_IRQ_TIM16_PRIORITY            ${doc.STM32_IRQ_TIM16_PRIORITY!"7"}
#define STM32_IRQ_TIM17_PRIORITY            ${doc.STM32_IRQ_TIM17_PRIORITY!"7"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"12"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"12"}
#define STM32_IRQ_USART3_PRIORITY           ${doc.STM32_IRQ_USART3_PRIORITY!"12"}
#define STM32_IRQ_UART4_PRIORITY            ${doc.STM32_IRQ_UART4_PRIORITY!"12"}
#define STM32_IRQ_UART5_PRIORITY            ${doc.STM32_IRQ_UART5_PRIORITY!"12"}
#define STM32_IRQ_USART6_PRIORITY           ${doc.STM32_IRQ_USART6_PRIORITY!"12"}
#define STM32_IRQ_UART7_PRIORITY            ${doc.STM32_IRQ_UART7_PRIORITY!"12"}
#define STM32_IRQ_UART8_PRIORITY            ${doc.STM32_IRQ_UART8_PRIORITY!"12"}
#define STM32_IRQ_UART9_PRIORITY            ${doc.STM32_IRQ_UART9_PRIORITY!"12"}
#define STM32_IRQ_USART10_PRIORITY          ${doc.STM32_IRQ_USART10_PRIORITY!"12"}
#define STM32_IRQ_LPUART1_PRIORITY          ${doc.STM32_IRQ_LPUART1_PRIORITY!"12"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_DUAL_MODE                 ${doc.STM32_ADC_DUAL_MODE!"FALSE"}
#define STM32_ADC_SAMPLES_SIZE              ${doc.STM32_ADC_SAMPLES_SIZE!"16"}
#define STM32_ADC_USE_ADC12                 ${doc.STM32_ADC_USE_ADC12!"TRUE"}
#define STM32_ADC_ADC12_DMA_STREAM          ${doc.STM32_ADC_ADC12_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_ADC_ADC12_DMA_PRIORITY        ${doc.STM32_ADC_ADC12_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC12_IRQ_PRIORITY        ${doc.STM32_ADC_ADC12_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC12_CLOCK_MODE          ${doc.STM32_ADC_ADC12_CLOCK_MODE!"ADC_CCR_CKMODE_ADCCK"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_FDCAN1                ${doc.STM32_CAN_USE_FDCAN1!"FALSE"}
#define STM32_CAN_USE_FDCAN2                ${doc.STM32_CAN_USE_FDCAN2!"FALSE"}
#define STM32_CAN_USE_FDCAN3                ${doc.STM32_CAN_USE_FDCAN3!"FALSE"}

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 ${doc.STM32_DAC_DUAL_MODE!"FALSE"}
#define STM32_DAC_USE_DAC1_CH1              ${doc.STM32_DAC_USE_DAC1_CH1!"FALSE"}
#define STM32_DAC_USE_DAC1_CH2              ${doc.STM32_DAC_USE_DAC1_CH2!"FALSE"}
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_IRQ_PRIORITY!"10"}
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     ${doc.STM32_DAC_DAC1_CH2_IRQ_PRIORITY!"10"}
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH2_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH1_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH1_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_DAC_DAC1_CH2_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH2_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  ${doc.STM32_GPT_USE_TIM1!"FALSE"}
#define STM32_GPT_USE_TIM2                  ${doc.STM32_GPT_USE_TIM2!"FALSE"}
#define STM32_GPT_USE_TIM3                  ${doc.STM32_GPT_USE_TIM3!"FALSE"}
#define STM32_GPT_USE_TIM4                  ${doc.STM32_GPT_USE_TIM4!"FALSE"}
#define STM32_GPT_USE_TIM5                  ${doc.STM32_GPT_USE_TIM5!"FALSE"}
#define STM32_GPT_USE_TIM6                  ${doc.STM32_GPT_USE_TIM6!"FALSE"}
#define STM32_GPT_USE_TIM7                  ${doc.STM32_GPT_USE_TIM7!"FALSE"}
#define STM32_GPT_USE_TIM8                  ${doc.STM32_GPT_USE_TIM8!"FALSE"}
#define STM32_GPT_USE_TIM12                 ${doc.STM32_GPT_USE_TIM12!"FALSE"}
#define STM32_GPT_USE_TIM13                 ${doc.STM32_GPT_USE_TIM13!"FALSE"}
#define STM32_GPT_USE_TIM14                 ${doc.STM32_GPT_USE_TIM14!"FALSE"}
#define STM32_GPT_USE_TIM15                 ${doc.STM32_GPT_USE_TIM15!"FALSE"}
#define STM32_GPT_USE_TIM16                 ${doc.STM32_GPT_USE_TIM16!"FALSE"}
#define STM32_GPT_USE_TIM17                 ${doc.STM32_GPT_USE_TIM17!"FALSE"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_USE_I2C2                  ${doc.STM32_I2C_USE_I2C2!"FALSE"}
#define STM32_I2C_USE_I2C3                  ${doc.STM32_I2C_USE_I2C3!"FALSE"}
#define STM32_I2C_USE_I2C4                  ${doc.STM32_I2C_USE_I2C4!"FALSE"}
#define STM32_I2C_USE_I2C5                  ${doc.STM32_I2C_USE_I2C5!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_RX_DMA_STREAM        ${doc.STM32_I2C_I2C1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C1_TX_DMA_STREAM        ${doc.STM32_I2C_I2C1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C2_RX_DMA_STREAM        ${doc.STM32_I2C_I2C2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C2_TX_DMA_STREAM        ${doc.STM32_I2C_I2C2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C3_RX_DMA_STREAM        ${doc.STM32_I2C_I2C3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C3_TX_DMA_STREAM        ${doc.STM32_I2C_I2C3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C4_RX_BDMA_STREAM       ${doc.STM32_I2C_I2C4_RX_BDMA_STREAM!"STM32_BDMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C4_TX_BDMA_STREAM       ${doc.STM32_I2C_I2C4_TX_BDMA_STREAM!"STM32_BDMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C5_RX_DMA_STREAM        ${doc.STM32_I2C_I2C5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C5_TX_DMA_STREAM        ${doc.STM32_I2C_I2C5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C1_IRQ_PRIORITY         ${doc.STM32_I2C_I2C1_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C2_IRQ_PRIORITY         ${doc.STM32_I2C_I2C2_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C3_IRQ_PRIORITY         ${doc.STM32_I2C_I2C3_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C4_IRQ_PRIORITY         ${doc.STM32_I2C_I2C4_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C5_IRQ_PRIORITY         ${doc.STM32_I2C_I2C5_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C2_DMA_PRIORITY         ${doc.STM32_I2C_I2C2_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C3_DMA_PRIORITY         ${doc.STM32_I2C_I2C3_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C4_DMA_PRIORITY         ${doc.STM32_I2C_I2C4_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C5_DMA_PRIORITY         ${doc.STM32_I2C_I2C5_DMA_PRIORITY!"3"}
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      ${doc.STM32_I2C_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  ${doc.STM32_ICU_USE_TIM1!"FALSE"}
#define STM32_ICU_USE_TIM2                  ${doc.STM32_ICU_USE_TIM2!"FALSE"}
#define STM32_ICU_USE_TIM3                  ${doc.STM32_ICU_USE_TIM3!"FALSE"}
#define STM32_ICU_USE_TIM4                  ${doc.STM32_ICU_USE_TIM4!"FALSE"}
#define STM32_ICU_USE_TIM5                  ${doc.STM32_ICU_USE_TIM5!"FALSE"}
#define STM32_ICU_USE_TIM8                  ${doc.STM32_ICU_USE_TIM8!"FALSE"}
#define STM32_ICU_USE_TIM12                 ${doc.STM32_ICU_USE_TIM12!"FALSE"}
#define STM32_ICU_USE_TIM13                 ${doc.STM32_ICU_USE_TIM13!"FALSE"}
#define STM32_ICU_USE_TIM14                 ${doc.STM32_ICU_USE_TIM14!"FALSE"}
#define STM32_ICU_USE_TIM15                 ${doc.STM32_ICU_USE_TIM15!"FALSE"}
#define STM32_ICU_USE_TIM16                 ${doc.STM32_ICU_USE_TIM16!"FALSE"}
#define STM32_ICU_USE_TIM17                 ${doc.STM32_ICU_USE_TIM17!"FALSE"}

/*
 * MAC driver system settings.
 */
#define STM32_MAC_TRANSMIT_BUFFERS          ${doc.STM32_MAC_TRANSMIT_BUFFERS!"2"}
#define STM32_MAC_RECEIVE_BUFFERS           ${doc.STM32_MAC_RECEIVE_BUFFERS!"4"}
#define STM32_MAC_BUFFERS_SIZE              ${doc.STM32_MAC_BUFFERS_SIZE!"1522"}
#define STM32_MAC_PHY_TIMEOUT               ${doc.STM32_MAC_PHY_TIMEOUT!"100"}
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     ${doc.STM32_MAC_ETH1_CHANGE_PHY_STATE!"TRUE"}
#define STM32_MAC_ETH1_IRQ_PRIORITY         ${doc.STM32_MAC_ETH1_IRQ_PRIORITY!"13"}
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       ${doc.STM32_MAC_IP_CHECKSUM_OFFLOAD!"FALSE"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  ${doc.STM32_PWM_USE_TIM1!"FALSE"}
#define STM32_PWM_USE_TIM2                  ${doc.STM32_PWM_USE_TIM2!"FALSE"}
#define STM32_PWM_USE_TIM3                  ${doc.STM32_PWM_USE_TIM3!"FALSE"}
#define STM32_PWM_USE_TIM4                  ${doc.STM32_PWM_USE_TIM4!"FALSE"}
#define STM32_PWM_USE_TIM5                  ${doc.STM32_PWM_USE_TIM5!"FALSE"}
#define STM32_PWM_USE_TIM8                  ${doc.STM32_PWM_USE_TIM8!"FALSE"}
#define STM32_PWM_USE_TIM12                 ${doc.STM32_PWM_USE_TIM12!"FALSE"}
#define STM32_PWM_USE_TIM13                 ${doc.STM32_PWM_USE_TIM13!"FALSE"}
#define STM32_PWM_USE_TIM14                 ${doc.STM32_PWM_USE_TIM14!"FALSE"}
#define STM32_PWM_USE_TIM15                 ${doc.STM32_PWM_USE_TIM15!"FALSE"}
#define STM32_PWM_USE_TIM16                 ${doc.STM32_PWM_USE_TIM16!"FALSE"}
#define STM32_PWM_USE_TIM17                 ${doc.STM32_PWM_USE_TIM17!"FALSE"}

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               ${doc.STM32_RTC_PRESA_VALUE!"32"}
#define STM32_RTC_PRESS_VALUE               ${doc.STM32_RTC_PRESS_VALUE!"1024"}
#define STM32_RTC_CR_INIT                   ${doc.STM32_RTC_CR_INIT!"0"}
#define STM32_RTC_TAMPCR_INIT               ${doc.STM32_RTC_TAMPCR_INIT!"0"}

/*
 * SDC driver system settings.
 */
#define STM32_SDC_USE_SDMMC1                ${doc.STM32_SDC_USE_SDMMC1!"FALSE"}
#define STM32_SDC_USE_SDMMC2                ${doc.STM32_SDC_USE_SDMMC2!"FALSE"}
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   ${doc.STM32_SDC_SDMMC_UNALIGNED_SUPPORT!"TRUE"}
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       ${doc.STM32_SDC_SDMMC_WRITE_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_READ_TIMEOUT        ${doc.STM32_SDC_SDMMC_READ_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_CLOCK_DELAY         ${doc.STM32_SDC_SDMMC_CLOCK_DELAY!"10"}
#define STM32_SDC_SDMMC_PWRSAV              ${doc.STM32_SDC_SDMMC_PWRSAV!"TRUE"}

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"FALSE"}
#define STM32_SERIAL_USE_USART3             ${doc.STM32_SERIAL_USE_USART3!"FALSE"}
#define STM32_SERIAL_USE_UART4              ${doc.STM32_SERIAL_USE_UART4!"FALSE"}
#define STM32_SERIAL_USE_UART5              ${doc.STM32_SERIAL_USE_UART5!"FALSE"}
#define STM32_SERIAL_USE_USART6             ${doc.STM32_SERIAL_USE_USART6!"FALSE"}
#define STM32_SERIAL_USE_UART7              ${doc.STM32_SERIAL_USE_UART7!"FALSE"}
#define STM32_SERIAL_USE_UART8              ${doc.STM32_SERIAL_USE_UART8!"FALSE"}
#define STM32_SERIAL_USE_UART9              ${doc.STM32_SERIAL_USE_UART9!"FALSE"}
#define STM32_SERIAL_USE_USART10            ${doc.STM32_SERIAL_USE_USART10!"FALSE"}
#define STM32_SERIAL_USE_LPUART1            ${doc.STM32_SERIAL_USE_LPUART1!"FALSE"}

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                ${doc.STM32_SIO_USE_USART1!"FALSE"}
#define STM32_SIO_USE_USART2                ${doc.STM32_SIO_USE_USART2!"FALSE"}
#define STM32_SIO_USE_USART3                ${doc.STM32_SIO_USE_USART3!"FALSE"}
#define STM32_SIO_USE_UART4                 ${doc.STM32_SIO_USE_UART4!"FALSE"}
#define STM32_SIO_USE_UART5                 ${doc.STM32_SIO_USE_UART5!"FALSE"}
#define STM32_SIO_USE_USART6                ${doc.STM32_SIO_USE_USART6!"FALSE"}
#define STM32_SIO_USE_UART7                 ${doc.STM32_SIO_USE_UART7!"FALSE"}
#define STM32_SIO_USE_UART8                 ${doc.STM32_SIO_USE_UART8!"FALSE"}
#define STM32_SIO_USE_UART9                 ${doc.STM32_SIO_USE_UART9!"FALSE"}
#define STM32_SIO_USE_USART10               ${doc.STM32_SIO_USE_USART10!"FALSE"}
#define STM32_SIO_USE_LPUART1               ${doc.STM32_SIO_USE_LPUART1!"FALSE"}

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_USE_SPI2                  ${doc.STM32_SPI_USE_SPI2!"FALSE"}
#define STM32_SPI_USE_SPI3                  ${doc.STM32_SPI_USE_SPI3!"FALSE"}
#define STM32_SPI_USE_SPI4                  ${doc.STM32_SPI_USE_SPI4!"FALSE"}
#define STM32_SPI_USE_SPI5                  ${doc.STM32_SPI_USE_SPI5!"FALSE"}
#define STM32_SPI_USE_SPI6                  ${doc.STM32_SPI_USE_SPI6!"FALSE"}
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI2_RX_DMA_STREAM        ${doc.STM32_SPI_SPI2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI2_TX_DMA_STREAM        ${doc.STM32_SPI_SPI2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI3_RX_DMA_STREAM        ${doc.STM32_SPI_SPI3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI3_TX_DMA_STREAM        ${doc.STM32_SPI_SPI3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI4_RX_DMA_STREAM        ${doc.STM32_SPI_SPI4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI4_TX_DMA_STREAM        ${doc.STM32_SPI_SPI4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI5_RX_DMA_STREAM        ${doc.STM32_SPI_SPI5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI5_TX_DMA_STREAM        ${doc.STM32_SPI_SPI5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI6_RX_BDMA_STREAM       ${doc.STM32_SPI_SPI6_RX_BDMA_STREAM!"STM32_BDMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI6_TX_BDMA_STREAM       ${doc.STM32_SPI_SPI6_TX_BDMA_STREAM!"STM32_BDMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI1_DMA_PRIORITY         ${doc.STM32_SPI_SPI1_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI2_DMA_PRIORITY         ${doc.STM32_SPI_SPI2_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI3_DMA_PRIORITY         ${doc.STM32_SPI_SPI3_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI4_DMA_PRIORITY         ${doc.STM32_SPI_SPI4_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI5_DMA_PRIORITY         ${doc.STM32_SPI_SPI5_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI6_DMA_PRIORITY         ${doc.STM32_SPI_SPI6_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI1_IRQ_PRIORITY         ${doc.STM32_SPI_SPI1_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI2_IRQ_PRIORITY         ${doc.STM32_SPI_SPI2_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI3_IRQ_PRIORITY         ${doc.STM32_SPI_SPI3_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI4_IRQ_PRIORITY         ${doc.STM32_SPI_SPI4_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI5_IRQ_PRIORITY         ${doc.STM32_SPI_SPI5_IRQ_PRIORITY!"10"}
#define STM32_SPI_SPI6_IRQ_PRIORITY         ${doc.STM32_SPI_SPI6_IRQ_PRIORITY!"10"}
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
#define STM32_UART_USE_USART1               ${doc.STM32_UART_USE_USART1!"FALSE"}
#define STM32_UART_USE_USART2               ${doc.STM32_UART_USE_USART2!"FALSE"}
#define STM32_UART_USE_USART3               ${doc.STM32_UART_USE_USART3!"FALSE"}
#define STM32_UART_USE_UART4                ${doc.STM32_UART_USE_UART4!"FALSE"}
#define STM32_UART_USE_UART5                ${doc.STM32_UART_USE_UART5!"FALSE"}
#define STM32_UART_USE_USART6               ${doc.STM32_UART_USE_USART6!"FALSE"}
#define STM32_UART_USE_UART7                ${doc.STM32_UART_USE_UART7!"FALSE"}
#define STM32_UART_USE_UART8                ${doc.STM32_UART_USE_UART8!"FALSE"}
#define STM32_UART_USE_UART9                ${doc.STM32_UART_USE_UART9!"FALSE"}
#define STM32_UART_USE_USART10              ${doc.STM32_UART_USE_USART10!"FALSE"}
#define STM32_UART_USART1_RX_DMA_STREAM     ${doc.STM32_UART_USART1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART1_TX_DMA_STREAM     ${doc.STM32_UART_USART1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART2_RX_DMA_STREAM     ${doc.STM32_UART_USART2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART2_TX_DMA_STREAM     ${doc.STM32_UART_USART2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART3_RX_DMA_STREAM     ${doc.STM32_UART_USART3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART3_TX_DMA_STREAM     ${doc.STM32_UART_USART3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART4_RX_DMA_STREAM      ${doc.STM32_UART_UART4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART4_TX_DMA_STREAM      ${doc.STM32_UART_UART4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART5_RX_DMA_STREAM      ${doc.STM32_UART_UART5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART5_TX_DMA_STREAM      ${doc.STM32_UART_UART5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART6_RX_DMA_STREAM     ${doc.STM32_UART_USART6_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART6_TX_DMA_STREAM     ${doc.STM32_UART_USART6_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART7_RX_DMA_STREAM      ${doc.STM32_UART_UART7_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART7_TX_DMA_STREAM      ${doc.STM32_UART_UART7_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART8_RX_DMA_STREAM      ${doc.STM32_UART_UART8_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART8_TX_DMA_STREAM      ${doc.STM32_UART_UART8_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART9_RX_DMA_STREAM      ${doc.STM32_UART_UART9_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_UART9_TX_DMA_STREAM      ${doc.STM32_UART_UART9_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART10_RX_DMA_STREAM    ${doc.STM32_UART_USART10_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART10_TX_DMA_STREAM    ${doc.STM32_UART_USART10_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_USART3_DMA_PRIORITY      ${doc.STM32_UART_USART3_DMA_PRIORITY!"0"}
#define STM32_UART_UART4_DMA_PRIORITY       ${doc.STM32_UART_UART4_DMA_PRIORITY!"0"}
#define STM32_UART_UART5_DMA_PRIORITY       ${doc.STM32_UART_UART5_DMA_PRIORITY!"0"}
#define STM32_UART_USART6_DMA_PRIORITY      ${doc.STM32_UART_USART6_DMA_PRIORITY!"0"}
#define STM32_UART_UART7_DMA_PRIORITY       ${doc.STM32_UART_UART7_DMA_PRIORITY!"0"}
#define STM32_UART_UART8_DMA_PRIORITY       ${doc.STM32_UART_UART8_DMA_PRIORITY!"0"}
#define STM32_UART_UART9_DMA_PRIORITY       ${doc.STM32_UART_UART9_DMA_PRIORITY!"0"}
#define STM32_UART_USART10_DMA_PRIORITY     ${doc.STM32_UART_USART10_DMA_PRIORITY!"0"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG2                  ${doc.STM32_USB_USE_OTG2!"FALSE"}
#define STM32_USB_OTG2_IRQ_PRIORITY         ${doc.STM32_USB_OTG2_IRQ_PRIORITY!"14"}
#define STM32_USB_OTG2_RX_FIFO_SIZE         ${doc.STM32_USB_OTG2_RX_FIFO_SIZE!"1024"}
#define STM32_USB_HOST_WAKEUP_DURATION      ${doc.STM32_USB_HOST_WAKEUP_DURATION!"2"}

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

/*
 * WSPI driver system settings.
 */
#define STM32_WSPI_USE_OCTOSPI1             ${doc.STM32_WSPI_USE_OCTOSPI1!"FALSE"}
#define STM32_WSPI_USE_OCTOSPI2             ${doc.STM32_WSPI_USE_OCTOSPI2!"FALSE"}
#define STM32_WSPI_OCTOSPI1_PRESCALER_VALUE ${doc.STM32_WSPI_OCTOSPI1_PRESCALER_VALUE!"1"}
#define STM32_WSPI_OCTOSPI2_PRESCALER_VALUE ${doc.STM32_WSPI_OCTOSPI2_PRESCALER_VALUE!"1"}
#define STM32_WSPI_OCTOSPI1_SSHIFT          ${doc.STM32_WSPI_OCTOSPI1_SSHIFT!"FALSE"}
#define STM32_WSPI_OCTOSPI2_SSHIFT          ${doc.STM32_WSPI_OCTOSPI2_SSHIFT!"FALSE"}
#define STM32_WSPI_OCTOSPI1_DHQC            ${doc.STM32_WSPI_OCTOSPI1_DHQC!"FALSE"}
#define STM32_WSPI_OCTOSPI2_DHQC            ${doc.STM32_WSPI_OCTOSPI2_DHQC!"FALSE"}
#define STM32_WSPI_OCTOSPI1_MDMA_CHANNEL    ${doc.STM32_WSPI_OCTOSPI1_MDMA_CHANNEL!"STM32_MDMA_CHANNEL_ID_ANY"}
#define STM32_WSPI_OCTOSPI2_MDMA_CHANNEL    ${doc.STM32_WSPI_OCTOSPI2_MDMA_CHANNEL!"STM32_MDMA_CHANNEL_ID_ANY"}
#define STM32_WSPI_OCTOSPI1_MDMA_PRIORITY   ${doc.STM32_WSPI_OCTOSPI1_MDMA_PRIORITY!"1"}
#define STM32_WSPI_OCTOSPI2_MDMA_PRIORITY   ${doc.STM32_WSPI_OCTOSPI2_MDMA_PRIORITY!"1"}
#define STM32_WSPI_OCTOSPI1_MDMA_IRQ_PRIORITY ${doc.STM32_WSPI_OCTOSPI1_MDMA_IRQ_PRIORITY!"10"}
#define STM32_WSPI_OCTOSPI2_MDMA_IRQ_PRIORITY ${doc.STM32_WSPI_OCTOSPI2_MDMA_IRQ_PRIORITY!"10"}
#define STM32_WSPI_DMA_ERROR_HOOK(wspip)    ${doc.STM32_WSPI_DMA_ERROR_HOOK!"osalSysHalt(\"MDMA failure\")"}

#endif /* MCUCONF_H */
