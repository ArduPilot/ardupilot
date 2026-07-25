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
#define STM32L4P5_MCUCONF
#define STM32L4Q5_MCUCONF
#define STM32L4R5_MCUCONF
#define STM32L4S5_MCUCONF
#define STM32L4R7_MCUCONF
#define STM32L4S7_MCUCONF
#define STM32L4R9_MCUCONF
#define STM32L4S9_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_CLOCK_DYNAMIC                 ${doc.STM32_CLOCK_DYNAMIC!"FALSE"}
#define STM32_VOS                           ${doc.STM32_VOS!"STM32_VOS_RANGE1"}
#define STM32_PWR_BOOST                     ${doc.STM32_PWR_BOOST!"TRUE"}
#define STM32_PWR_CR2                       ${doc.STM32_PWR_CR2!"(PWR_CR2_PLS_LEV0 | PWR_CR2_IOSV)"}
#define STM32_PWR_CR3                       ${doc.STM32_PWR_CR3!"(PWR_CR3_EIWF)"}
#define STM32_PWR_CR4                       ${doc.STM32_PWR_CR4!"(0U)"}
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
#define STM32_PWR_PUCRF                     ${doc.STM32_PWR_PUCRF!"(0U)"}
#define STM32_PWR_PDCRF                     ${doc.STM32_PWR_PDCRF!"(0U)"}
#define STM32_PWR_PUCRG                     ${doc.STM32_PWR_PUCRG!"(0U)"}
#define STM32_PWR_PDCRG                     ${doc.STM32_PWR_PDCRG!"(0U)"}
#define STM32_PWR_PUCRH                     ${doc.STM32_PWR_PUCRH!"(0U)"}
#define STM32_PWR_PDCRH                     ${doc.STM32_PWR_PDCRH!"(0U)"}
#define STM32_PWR_PUCRI                     ${doc.STM32_PWR_PUCRI!"(0U)"}
#define STM32_PWR_PDCRI                     ${doc.STM32_PWR_PDCRI!"(0U)"}
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
#define STM32_PLLN_VALUE                    ${doc.STM32_PLLN_VALUE!"60"}
#define STM32_PLLPDIV_VALUE                 ${doc.STM32_PLLPDIV_VALUE!"0"}
#define STM32_PLLP_VALUE                    ${doc.STM32_PLLP_VALUE!"7"}
#define STM32_PLLQ_VALUE                    ${doc.STM32_PLLQ_VALUE!"4"}
#define STM32_PLLR_VALUE                    ${doc.STM32_PLLR_VALUE!"2"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV1"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV1"}
#define STM32_STOPWUCK                      ${doc.STM32_STOPWUCK!"STM32_STOPWUCK_MSI"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"STM32_MCOSEL_NOCLOCK"}
#define STM32_MCOPRE                        ${doc.STM32_MCOPRE!"STM32_MCOPRE_DIV1"}
#define STM32_LSCOSEL                       ${doc.STM32_LSCOSEL!"STM32_LSCOSEL_NOCLOCK"}
#define STM32_PLLSAI1M_VALUE                ${doc.STM32_PLLSAI1M_VALUE!"1"}
#define STM32_PLLSAI1N_VALUE                ${doc.STM32_PLLSAI1N_VALUE!"72"}
#define STM32_PLLSAI1PDIV_VALUE             ${doc.STM32_PLLSAI1PDIV_VALUE!"6"}
#define STM32_PLLSAI1P_VALUE                ${doc.STM32_PLLSAI1P_VALUE!"7"}
#define STM32_PLLSAI1Q_VALUE                ${doc.STM32_PLLSAI1Q_VALUE!"6"}
#define STM32_PLLSAI1R_VALUE                ${doc.STM32_PLLSAI1R_VALUE!"6"}
#define STM32_PLLSAI2M_VALUE                ${doc.STM32_PLLSAI2M_VALUE!"1"}
#define STM32_PLLSAI2N_VALUE                ${doc.STM32_PLLSAI2N_VALUE!"72"}
#define STM32_PLLSAI2PDIV_VALUE             ${doc.STM32_PLLSAI2PDIV_VALUE!"6"}
#define STM32_PLLSAI2P_VALUE                ${doc.STM32_PLLSAI2P_VALUE!"7"}
#define STM32_PLLSAI2Q_VALUE                ${doc.STM32_PLLSAI2Q_VALUE!"6"}
#define STM32_PLLSAI2R_VALUE                ${doc.STM32_PLLSAI2R_VALUE!"6"}
#define STM32_PLLSAI2DIVR                   ${doc.STM32_PLLSAI2DIVR!"STM32_PLLSAI2DIVR_DIV16"}

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
#define STM32_I2C4SEL                       ${doc.STM32_I2C4SEL!"STM32_I2C4SEL_SYSCLK"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"STM32_LPTIM1SEL_PCLK1"}
#define STM32_LPTIM2SEL                     ${doc.STM32_LPTIM2SEL!"STM32_LPTIM2SEL_PCLK1"}
#define STM32_CLK48SEL                      ${doc.STM32_CLK48SEL!"STM32_CLK48SEL_PLLSAI1"}
#define STM32_ADCSEL                        ${doc.STM32_ADCSEL!"STM32_ADCSEL_SYSCLK"}
#define STM32_DFSDMSEL                      ${doc.STM32_DFSDMSEL!"STM32_DFSDMSEL_PCLK2"}
#define STM32_ADFSDMSEL                     ${doc.STM32_ADFSDMSEL!"STM32_ADFSDMSEL_SAI1CLK"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"STM32_SAI1SEL_OFF"}
#define STM32_SAI2SEL                       ${doc.STM32_SAI2SEL!"STM32_SAI2SEL_OFF"}
#define STM32_DSISEL                        ${doc.STM32_DSISEL!"STM32_DSISEL_DSIPHY"}
#define STM32_SDMMCSEL                      ${doc.STM32_SDMMC!"STM32_SDMMCSEL_48CLK"}
#define STM32_OSPISEL                       ${doc.STM32_OSPISEL!"STM32_OSPISEL_SYSCLK"}
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

#define STM32_IRQ_SDMMC1_PRIORITY           ${doc.STM32_IRQ_SDMMC1_PRIORITY!"9"}

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
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC12_IRQ_PRIORITY        ${doc.STM32_ADC_ADC12_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC123_CLOCK_MODE         ${doc.STM32_ADC_ADC123_CLOCK_MODE!"ADC_CCR_CKMODE_AHB_DIV2"}
#define STM32_ADC_ADC123_PRESC              ${doc.STM32_ADC_ADC123_PRESC!"ADC_CCR_PRESC_DIV2"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  ${doc.STM32_CAN_USE_CAN1!"FALSE"}
#define STM32_CAN_CAN1_IRQ_PRIORITY         ${doc.STM32_CAN_CAN1_IRQ_PRIORITY!"11"}

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
#define STM32_I2C_I2C1_RX_DMA_STREAM        ${doc.STM32_I2C_I2C1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C1_TX_DMA_STREAM        ${doc.STM32_I2C_I2C1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C2_RX_DMA_STREAM        ${doc.STM32_I2C_I2C2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C2_TX_DMA_STREAM        ${doc.STM32_I2C_I2C2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C3_RX_DMA_STREAM        ${doc.STM32_I2C_I2C3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C3_TX_DMA_STREAM        ${doc.STM32_I2C_I2C3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C4_RX_DMA_STREAM        ${doc.STM32_I2C_I2C4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C4_TX_DMA_STREAM        ${doc.STM32_I2C_I2C4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
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
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       ${doc.STM32_SDC_SDMMC_WRITE_TIMEOUT!"1000000"}
#define STM32_SDC_SDMMC_READ_TIMEOUT        ${doc.STM32_SDC_SDMMC_READ_TIMEOUT!"1000000"}
#define STM32_SDC_SDMMC_CLOCK_DELAY         ${doc.STM32_SDC_SDMMC_CLOCK_DELAY!"10"}
#define STM32_SDC_SDMMC_PWRSAV              ${doc.STM32_SDC_SDMMC_PWRSAV!"TRUE"}
#define STM32_SDC_SDMMC1_IRQ_PRIORITY       ${doc.STM32_SDC_SDMMC1_IRQ_PRIORITY!"9"}

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
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI2_RX_DMA_STREAM        ${doc.STM32_SPI_SPI2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI2_TX_DMA_STREAM        ${doc.STM32_SPI_SPI2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI3_RX_DMA_STREAM        ${doc.STM32_SPI_SPI3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI3_TX_DMA_STREAM        ${doc.STM32_SPI_SPI3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
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
#define STM32_WSPI_USE_OCTOSPI1             ${doc.STM32_WSPI_USE_OCTOSPI1!"FALSE"}
#define STM32_WSPI_USE_OCTOSPI2             ${doc.STM32_WSPI_USE_OCTOSPI2!"FALSE"}
#define STM32_WSPI_OCTOSPI1_PRESCALER_VALUE ${doc.STM32_WSPI_OCTOSPI1_PRESCALER_VALUE!"1"}
#define STM32_WSPI_OCTOSPI2_PRESCALER_VALUE ${doc.STM32_WSPI_OCTOSPI2_PRESCALER_VALUE!"1"}
#define STM32_WSPI_OCTOSPI1_IRQ_PRIORITY    ${doc.STM32_WSPI_OCTOSPI1_IRQ_PRIORITY!"10"}
#define STM32_WSPI_OCTOSPI2_IRQ_PRIORITY    ${doc.STM32_WSPI_OCTOSPI2_IRQ_PRIORITY!"10"}
#define STM32_WSPI_OCTOSPI1_DMA_STREAM      ${doc.STM32_WSPI_OCTOSPI1_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_WSPI_OCTOSPI2_DMA_STREAM      ${doc.STM32_WSPI_OCTOSPI2_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_WSPI_OCTOSPI1_DMA_PRIORITY    ${doc.STM32_WSPI_OCTOSPI1_DMA_PRIORITY!"1"}
#define STM32_WSPI_OCTOSPI2_DMA_PRIORITY    ${doc.STM32_WSPI_OCTOSPI2_DMA_PRIORITY!"1"}
#define STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY ${doc.STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY!"10"}
#define STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY ${doc.STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY!"10"}
#define STM32_WSPI_DMA_ERROR_HOOK(wspip)    ${doc.STM32_WSPI_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

#endif /* MCUCONF_H */
