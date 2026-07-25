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

/*
 * STM32L4xx drivers configuration.
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

#ifndef MCUCONF_H
#define MCUCONF_H

#define STM32L4xx_MCUCONF
#define STM32L496_MCUCONF
#define STM32L4A6_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_VOS                           ${doc.STM32_VOS!"STM32_VOS_RANGE1"}
#define STM32_PVD_ENABLE                    ${doc.STM32_PVD_ENABLE!"FALSE"}
#define STM32_PLS                           ${doc.STM32_PLS!"STM32_PLS_LEV0"}
#define STM32_HSI16_ENABLED                 ${doc.STM32_HSI16_ENABLED!"FALSE"}
#define STM32_HSI48_ENABLED                 ${doc.STM32_HSI48_ENABLED!"FALSE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"TRUE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"FALSE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"FALSE"}
#define STM32_MSIPLL_ENABLED                ${doc.STM32_MSIPLL_ENABLED!"FALSE"}
#define STM32_MSIRANGE                      ${doc.STM32_MSIRANGE!"STM32_MSIRANGE_4M"}
#define STM32_MSISRANGE                     ${doc.STM32_MSISRANGE!"STM32_MSISRANGE_4M"}
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL"}
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_MSI"}
#define STM32_PLLM_VALUE                    ${doc.STM32_PLLM_VALUE!"1"}
#define STM32_PLLN_VALUE                    ${doc.STM32_PLLN_VALUE!"80"}
#define STM32_PLLPDIV_VALUE                 ${doc.STM32_PLLPDIV_VALUE!"0"}
#define STM32_PLLP_VALUE                    ${doc.STM32_PLLP_VALUE!"7"}
#define STM32_PLLQ_VALUE                    ${doc.STM32_PLLQ_VALUE!"6"}
#define STM32_PLLR_VALUE                    ${doc.STM32_PLLR_VALUE!"4"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV1"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV1"}
#define STM32_STOPWUCK                      ${doc.STM32_STOPWUCK!"STM32_STOPWUCK_MSI"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"STM32_MCOSEL_NOCLOCK"}
#define STM32_MCOPRE                        ${doc.STM32_MCOPRE!"STM32_MCOPRE_DIV1"}
#define STM32_LSCOSEL                       ${doc.STM32_LSCOSEL!"STM32_LSCOSEL_NOCLOCK"}
#define STM32_PLLSAI1N_VALUE                ${doc.STM32_PLLSAI1N_VALUE!"48"}
#define STM32_PLLSAI1PDIV_VALUE             ${doc.STM32_PLLSAI1PDIV_VALUE!"6"}
#define STM32_PLLSAI1P_VALUE                ${doc.STM32_PLLSAI1P_VALUE!"7"}
#define STM32_PLLSAI1Q_VALUE                ${doc.STM32_PLLSAI1Q_VALUE!"4"}
#define STM32_PLLSAI1R_VALUE                ${doc.STM32_PLLSAI1R_VALUE!"6"}
#define STM32_PLLSAI2N_VALUE                ${doc.STM32_PLLSAI2N_VALUE!"48"}
#define STM32_PLLSAI2PDIV_VALUE             ${doc.STM32_PLLSAI2PDIV_VALUE!"6"}
#define STM32_PLLSAI2P_VALUE                ${doc.STM32_PLLSAI2P_VALUE!"7"}
#define STM32_PLLSAI2R_VALUE                ${doc.STM32_PLLSAI2R_VALUE!"6"}

/*
 * Peripherals clock sources.
 */
#define STM32_USART1SEL                     ${doc.STM32_USART1SEL!"STM32_USART1SEL_SYSCLK"}
#define STM32_USART2SEL                     ${doc.STM32_USART2SEL!"STM32_USART2SEL_SYSCLK"}
#define STM32_USART3SEL                     ${doc.STM32_USART3SEL!"STM32_USART3SEL_SYSCLK"}
#define STM32_UART4SEL                      ${doc.STM32_UART4SEL!"STM32_UART4SEL_SYSCLK"}
#define STM32_UART5SEL                      ${doc.STM32_UART5SEL!"STM32_UART5SEL_SYSCLK"}
#define STM32_LPUART1SEL                    ${doc.STM32_LPUART1SEL!"STM32_LPUART1SEL_SYSCLK"}
#define STM32_I2C1SEL                       ${doc.STM32_I2C1SEL!"STM32_I2C1SEL_SYSCLK"}
#define STM32_I2C2SEL                       ${doc.STM32_I2C2SEL!"STM32_I2C2SEL_SYSCLK"}
#define STM32_I2C3SEL                       ${doc.STM32_I2C3SEL!"STM32_I2C3SEL_SYSCLK"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"STM32_LPTIM1SEL_PCLK1"}
#define STM32_LPTIM2SEL                     ${doc.STM32_LPTIM2SEL!"STM32_LPTIM2SEL_PCLK1"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"STM32_SAI1SEL_OFF"}
#define STM32_SAI2SEL                       ${doc.STM32_SAI2SEL!"STM32_SAI2SEL_OFF"}
#define STM32_CLK48SEL                      ${doc.STM32_CLK48SEL!"STM32_CLK48SEL_PLL"}
#define STM32_ADCSEL                        ${doc.STM32_ADCSEL!"STM32_ADCSEL_SYSCLK"}
#define STM32_SWPMI1SEL                     ${doc.STM32_SWPMI1SEL!"STM32_SWPMI1SEL_PCLK1"}
#define STM32_DFSDMSEL                      ${doc.STM32_DFSDMSEL!"STM32_DFSDMSEL_PCLK2"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSI"}

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
#define STM32_IRQ_EXTI1635_38_PRIORITY      ${doc.STM32_IRQ_EXTI1635_38_PRIORITY!"6"}
#define STM32_IRQ_EXTI18_PRIORITY           ${doc.STM32_IRQ_EXTI18_PRIORITY!"6"}
#define STM32_IRQ_EXTI19_PRIORITY           ${doc.STM32_IRQ_EXTI19_PRIORITY!"6"}
#define STM32_IRQ_EXTI20_PRIORITY           ${doc.STM32_IRQ_EXTI20_PRIORITY!"6"}
#define STM32_IRQ_EXTI21_22_PRIORITY        ${doc.STM32_IRQ_EXTI21_22_PRIORITY!"6"}

#define STM32_IRQ_TIM1_BRK_TIM15_PRIORITY   ${doc.STM32_IRQ_TIM1_BRK_TIM15_PRIORITY!"7"}
#define STM32_IRQ_TIM1_UP_TIM16_PRIORITY    ${doc.STM32_IRQ_TIM1_UP_TIM16_PRIORITY!"7"}
#define STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY ${doc.STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY!"7"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"7"}
#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"7"}
#define STM32_IRQ_TIM3_PRIORITY             ${doc.STM32_IRQ_TIM3_PRIORITY!"7"}
#define STM32_IRQ_TIM4_PRIORITY             ${doc.STM32_IRQ_TIM4_PRIORITY!"7"}
#define STM32_IRQ_TIM5_PRIORITY             ${doc.STM32_IRQ_TIM5_PRIORITY!"7"}
#define STM32_IRQ_TIM6_PRIORITY             ${doc.STM32_IRQ_TIM6_PRIORITY!"7"}
#define STM32_IRQ_TIM7_PRIORITY             ${doc.STM32_IRQ_TIM7_PRIORITY!"7"}
#define STM32_IRQ_TIM8_UP_PRIORITY          ${doc.STM32_IRQ_TIM8_UP_PRIORITY!"7"}
#define STM32_IRQ_TIM8_CC_PRIORITY          ${doc.STM32_IRQ_TIM8_CC_PRIORITY!"7"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"12"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"12"}
#define STM32_IRQ_USART3_PRIORITY           ${doc.STM32_IRQ_USART3_PRIORITY!"12"}
#define STM32_IRQ_UART4_PRIORITY            ${doc.STM32_IRQ_UART4_PRIORITY!"12"}
#define STM32_IRQ_UART5_PRIORITY            ${doc.STM32_IRQ_UART5_PRIORITY!"12"}
#define STM32_IRQ_LPUART1_PRIORITY          ${doc.STM32_IRQ_LPUART1_PRIORITY!"12"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_COMPACT_SAMPLES           ${doc.STM32_ADC_COMPACT_SAMPLES!"FALSE"}
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_USE_ADC2                  ${doc.STM32_ADC_USE_ADC2!"FALSE"}
#define STM32_ADC_USE_ADC3                  ${doc.STM32_ADC_USE_ADC3!"FALSE"}
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 1)"}
#define STM32_ADC_ADC2_DMA_STREAM           ${doc.STM32_ADC_ADC2_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_ADC_ADC3_DMA_STREAM           ${doc.STM32_ADC_ADC3_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC2_DMA_PRIORITY         ${doc.STM32_ADC_ADC2_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC3_DMA_PRIORITY         ${doc.STM32_ADC_ADC3_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC12_IRQ_PRIORITY        ${doc.STM32_ADC_ADC12_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC3_IRQ_PRIORITY         ${doc.STM32_ADC_ADC3_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC2_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC3_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC123_CLOCK_MODE         ${doc.STM32_ADC_ADC123_CLOCK_MODE!"ADC_CCR_CKMODE_AHB_DIV1"}
#define STM32_ADC_ADC123_PRESC              ${doc.STM32_ADC_ADC123_PRESC!"ADC_CCR_PRESC_DIV2"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  ${doc.STM32_CAN_USE_CAN1!"FALSE"}
#define STM32_CAN_USE_CAN2                  ${doc.STM32_CAN_USE_CAN2!"FALSE"}
#define STM32_CAN_CAN1_IRQ_PRIORITY         ${doc.STM32_CAN_CAN1_IRQ_PRIORITY!"11"}
#define STM32_CAN_CAN2_IRQ_PRIORITY         ${doc.STM32_CAN_CAN2_IRQ_PRIORITY!"11"}

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
#define STM32_DAC_DAC1_CH1_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 4)"}
#define STM32_DAC_DAC1_CH2_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH2_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}

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
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_RX_DMA_STREAM        ${doc.STM32_I2C_I2C1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_I2C_I2C1_TX_DMA_STREAM        ${doc.STM32_I2C_I2C1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_I2C_I2C2_RX_DMA_STREAM        ${doc.STM32_I2C_I2C2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_I2C_I2C2_TX_DMA_STREAM        ${doc.STM32_I2C_I2C2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_I2C_I2C3_RX_DMA_STREAM        ${doc.STM32_I2C_I2C3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_I2C_I2C3_TX_DMA_STREAM        ${doc.STM32_I2C_I2C3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_I2C_I2C4_RX_DMA_STREAM        ${doc.STM32_I2C_I2C4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
#define STM32_I2C_I2C4_TX_DMA_STREAM        ${doc.STM32_I2C_I2C4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_I2C_I2C1_IRQ_PRIORITY         ${doc.STM32_I2C_I2C1_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C2_IRQ_PRIORITY         ${doc.STM32_I2C_I2C2_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C3_IRQ_PRIORITY         ${doc.STM32_I2C_I2C3_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C4_IRQ_PRIORITY         ${doc.STM32_I2C_I2C4_IRQ_PRIORITY!"5"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C2_DMA_PRIORITY         ${doc.STM32_I2C_I2C2_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C3_DMA_PRIORITY         ${doc.STM32_I2C_I2C3_DMA_PRIORITY!"3"}
#define STM32_I2C_I2C4_DMA_PRIORITY         ${doc.STM32_I2C_I2C4_DMA_PRIORITY!"3"}
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
#define STM32_PWM_USE_TIM5                  ${doc.STM32_PWM_USE_TIM5!"FALSE"}
#define STM32_PWM_USE_TIM8                  ${doc.STM32_PWM_USE_TIM8!"FALSE"}
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
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   ${doc.STM32_SDC_SDMMC_UNALIGNED_SUPPORT!"TRUE"}
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       ${doc.STM32_SDC_SDMMC_WRITE_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_READ_TIMEOUT        ${doc.STM32_SDC_SDMMC_READ_TIMEOUT!"10000"}
#define STM32_SDC_SDMMC_CLOCK_DELAY         ${doc.STM32_SDC_SDMMC_CLOCK_DELAY!"10"}
#define STM32_SDC_SDMMC1_DMA_PRIORITY       ${doc.STM32_SDC_SDMMC1_DMA_PRIORITY!"3"}
#define STM32_SDC_SDMMC1_IRQ_PRIORITY       ${doc.STM32_SDC_SDMMC1_IRQ_PRIORITY!"9"}
#define STM32_SDC_SDMMC1_DMA_STREAM         ${doc.STM32_SDC_SDMMC1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 4)"}

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"FALSE"}
#define STM32_SERIAL_USE_USART3             ${doc.STM32_SERIAL_USE_USART3!"FALSE"}
#define STM32_SERIAL_USE_UART4              ${doc.STM32_SERIAL_USE_UART4!"FALSE"}
#define STM32_SERIAL_USE_UART5              ${doc.STM32_SERIAL_USE_UART5!"FALSE"}
#define STM32_SERIAL_USE_LPUART1            ${doc.STM32_SERIAL_USE_LPUART1!"FALSE"}

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                ${doc.STM32_SIO_USE_USART1!"FALSE"}
#define STM32_SIO_USE_USART2                ${doc.STM32_SIO_USE_USART2!"FALSE"}
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
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 3)"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 4)"}
#define STM32_SPI_SPI2_RX_DMA_STREAM        ${doc.STM32_SPI_SPI2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_SPI_SPI2_TX_DMA_STREAM        ${doc.STM32_SPI_SPI2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_SPI_SPI3_RX_DMA_STREAM        ${doc.STM32_SPI_SPI3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_SPI_SPI3_TX_DMA_STREAM        ${doc.STM32_SPI_SPI3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
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
#define STM32_UART_USE_USART1               ${doc.STM32_UART_USE_USART1!"FALSE"}
#define STM32_UART_USE_USART2               ${doc.STM32_UART_USE_USART2!"FALSE"}
#define STM32_UART_USE_USART3               ${doc.STM32_UART_USE_USART3!"FALSE"}
#define STM32_UART_USE_UART4                ${doc.STM32_UART_USE_UART4!"FALSE"}
#define STM32_UART_USE_UART5                ${doc.STM32_UART_USE_UART5!"FALSE"}
#define STM32_UART_USART1_RX_DMA_STREAM     ${doc.STM32_UART_USART1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 7)"}
#define STM32_UART_USART1_TX_DMA_STREAM     ${doc.STM32_UART_USART1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 6)"}
#define STM32_UART_USART2_RX_DMA_STREAM     ${doc.STM32_UART_USART2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_UART_USART2_TX_DMA_STREAM     ${doc.STM32_UART_USART2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_UART_USART3_RX_DMA_STREAM     ${doc.STM32_UART_USART3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_UART_USART3_TX_DMA_STREAM     ${doc.STM32_UART_USART3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_UART_UART4_RX_DMA_STREAM      ${doc.STM32_UART_UART4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 5)"}
#define STM32_UART_UART4_TX_DMA_STREAM      ${doc.STM32_UART_UART4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 3)"}
#define STM32_UART_UART5_RX_DMA_STREAM      ${doc.STM32_UART_UART5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
#define STM32_UART_UART5_TX_DMA_STREAM      ${doc.STM32_UART_UART5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_USART3_DMA_PRIORITY      ${doc.STM32_UART_USART3_DMA_PRIORITY!"0"}
#define STM32_UART_UART4_DMA_PRIORITY       ${doc.STM32_UART_UART4_DMA_PRIORITY!"0"}
#define STM32_UART_UART5_DMA_PRIORITY       ${doc.STM32_UART_UART5_DMA_PRIORITY!"0"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG1                  ${doc.STM32_USB_USE_OTG1!"FALSE"}
#define STM32_USB_OTG1_IRQ_PRIORITY         ${doc.STM32_USB_OTG1_IRQ_PRIORITY!"14"}
#define STM32_USB_OTG1_RX_FIFO_SIZE         ${doc.STM32_USB_OTG1_RX_FIFO_SIZE!"512"}

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

/*
 * WSPI driver system settings.
 */
#define STM32_WSPI_USE_QUADSPI1             ${doc.STM32_WSPI_USE_QUADSPI1!"FALSE"}
#define STM32_WSPI_QUADSPI1_DMA_STREAM      ${doc.STM32_WSPI_QUADSPI1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 7)"}
#define STM32_WSPI_QUADSPI1_PRESCALER_VALUE ${doc.STM32_WSPI_QUADSPI1_PRESCALER_VALUE!"1"}

#endif /* MCUCONF_H */
