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
 * STM32F7xx drivers configuration.
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

#define STM32F7xx_MCUCONF
#define STM32F746_MCUCONF
#define STM32F756_MCUCONF

/*
 * General settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}

/*
 * Memory attributes settings.
 */
#define STM32_NOCACHE_ENABLE                ${doc.STM32_NOCACHE_ENABLE!"FALSE"}
#define STM32_NOCACHE_MPU_REGION            ${doc.STM32_NOCACHE_MPU_REGION!"MPU_REGION_6"}
#define STM32_NOCACHE_RBAR                  ${doc.STM32_NOCACHE_RBAR!"0x2004C000U"}
#define STM32_NOCACHE_RASR                  ${doc.STM32_NOCACHE_RASR!"MPU_RASR_SIZE_16K"}

/*
 * HAL driver system settings.
 */
#define STM32_PVD_ENABLE                    ${doc.STM32_PVD_ENABLE!"FALSE"}
#define STM32_PLS                           ${doc.STM32_PLS!"STM32_PLS_LEV0"}
#define STM32_BKPRAM_ENABLE                 ${doc.STM32_BKPRAM_ENABLE!"FALSE"}
#define STM32_HSI_ENABLED                   ${doc.STM32_HSI_ENABLED!"TRUE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"FALSE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"TRUE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"TRUE"}
#define STM32_CLOCK48_REQUIRED              ${doc.STM32_CLOCK48_REQUIRED!"TRUE"}
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL"}
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_HSE"}
#define STM32_PLLM_VALUE                    ${doc.STM32_PLLM_VALUE!"8"}
#define STM32_PLLN_VALUE                    ${doc.STM32_PLLN_VALUE!"432"}
#define STM32_PLLP_VALUE                    ${doc.STM32_PLLP_VALUE!"2"}
#define STM32_PLLQ_VALUE                    ${doc.STM32_PLLQ_VALUE!"9"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV4"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV2"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSE"}
#define STM32_RTCPRE_VALUE                  ${doc.STM32_RTCPRE_VALUE!"25"}
#define STM32_MCO1SEL                       ${doc.STM32_MCO1SEL!"STM32_MCO1SEL_HSI"}
#define STM32_MCO1PRE                       ${doc.STM32_MCO1PRE!"STM32_MCO1PRE_DIV1"}
#define STM32_MCO2SEL                       ${doc.STM32_MCO2SEL!"STM32_MCO2SEL_SYSCLK"}
#define STM32_MCO2PRE                       ${doc.STM32_MCO2PRE!"STM32_MCO2PRE_DIV4"}
#define STM32_TIMPRE_ENABLE                 ${doc.STM32_TIMPRE_ENABLE!"FALSE"}
#define STM32_I2SSRC                        ${doc.STM32_I2SSRC!"STM32_I2SSRC_OFF"}
#define STM32_PLLI2SN_VALUE                 ${doc.STM32_PLLI2SN_VALUE!"192"}
#define STM32_PLLI2SP_VALUE                 ${doc.STM32_PLLI2SP_VALUE!"4"}
#define STM32_PLLI2SQ_VALUE                 ${doc.STM32_PLLI2SQ_VALUE!"4"}
#define STM32_PLLI2SR_VALUE                 ${doc.STM32_PLLI2SR_VALUE!"4"}
#define STM32_PLLI2SDIVQ_VALUE              ${doc.STM32_PLLI2SDIVQ_VALUE!"2"}
#define STM32_PLLSAIN_VALUE                 ${doc.STM32_PLLSAIN_VALUE!"192"}
#define STM32_PLLSAIP_VALUE                 ${doc.STM32_PLLSAIP_VALUE!"4"}
#define STM32_PLLSAIQ_VALUE                 ${doc.STM32_PLLSAIQ_VALUE!"4"}
#define STM32_PLLSAIR_VALUE                 ${doc.STM32_PLLSAIR_VALUE!"4"}
#define STM32_PLLSAIDIVQ_VALUE              ${doc.STM32_PLLSAIDIVQ_VALUE!"2"}
#define STM32_PLLSAIDIVR_VALUE              ${doc.STM32_PLLSAIDIVR_VALUE!"2"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"STM32_SAI1SEL_OFF"}
#define STM32_SAI2SEL                       ${doc.STM32_SAI2SEL!"STM32_SAI2SEL_OFF"}
#define STM32_LCDTFT_REQUIRED               ${doc.STM32_LCDTFT_REQUIRED!"FALSE"}
#define STM32_USART1SEL                     ${doc.STM32_USART1SEL!"STM32_USART1SEL_PCLK2"}
#define STM32_USART2SEL                     ${doc.STM32_USART2SEL!"STM32_USART2SEL_PCLK1"}
#define STM32_USART3SEL                     ${doc.STM32_USART3SEL!"STM32_USART3SEL_PCLK1"}
#define STM32_UART4SEL                      ${doc.STM32_UART4SEL!"STM32_UART4SEL_PCLK1"}
#define STM32_UART5SEL                      ${doc.STM32_UART5SEL!"STM32_UART5SEL_PCLK1"}
#define STM32_USART6SEL                     ${doc.STM32_USART6SEL!"STM32_USART6SEL_PCLK2"}
#define STM32_UART7SEL                      ${doc.STM32_UART7SEL!"STM32_UART7SEL_PCLK1"}
#define STM32_UART8SEL                      ${doc.STM32_UART8SEL!"STM32_UART8SEL_PCLK1"}
#define STM32_I2C1SEL                       ${doc.STM32_I2C1SEL!"STM32_I2C1SEL_PCLK1"}
#define STM32_I2C2SEL                       ${doc.STM32_I2C2SEL!"STM32_I2C2SEL_PCLK1"}
#define STM32_I2C3SEL                       ${doc.STM32_I2C3SEL!"STM32_I2C3SEL_PCLK1"}
#define STM32_I2C4SEL                       ${doc.STM32_I2C4SEL!"STM32_I2C4SEL_PCLK1"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"STM32_LPTIM1SEL_PCLK1"}
#define STM32_CECSEL                        ${doc.STM32_CECSEL!"STM32_CECSEL_LSE"}
#define STM32_CK48MSEL                      ${doc.STM32_CK48MSEL!"STM32_CK48MSEL_PLL"}
#define STM32_SDMMC1SEL                     ${doc.STM32_SDMMC1SEL!"STM32_SDMMC1SEL_PLL48CLK"}

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
#define STM32_IRQ_EXTI20_PRIORITY           ${doc.STM32_IRQ_EXTI20_PRIORITY!"6"}
#define STM32_IRQ_EXTI21_PRIORITY           ${doc.STM32_IRQ_EXTI21_PRIORITY!"6"}
#define STM32_IRQ_EXTI22_PRIORITY           ${doc.STM32_IRQ_EXTI22_PRIORITY!"6"}
#define STM32_IRQ_EXTI23_PRIORITY           ${doc.STM32_IRQ_EXTI23_PRIORITY!"6"}

#define STM32_IRQ_TIM1_BRK_TIM9_PRIORITY    ${doc.STM32_IRQ_TIM1_BRK_TIM9_PRIORITY!"7"}
#define STM32_IRQ_TIM1_UP_TIM10_PRIORITY    ${doc.STM32_IRQ_TIM1_UP_TIM10_PRIORITY!"7"}
#define STM32_IRQ_TIM1_TRGCO_TIM11_PRIORITY ${doc.STM32_IRQ_TIM1_TRGCO_TIM11_PRIORITY!"7"}
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

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"12"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"12"}
#define STM32_IRQ_USART3_PRIORITY           ${doc.STM32_IRQ_USART3_PRIORITY!"12"}
#define STM32_IRQ_UART4_PRIORITY            ${doc.STM32_IRQ_UART4_PRIORITY!"12"}
#define STM32_IRQ_UART5_PRIORITY            ${doc.STM32_IRQ_UART5_PRIORITY!"12"}
#define STM32_IRQ_USART6_PRIORITY           ${doc.STM32_IRQ_USART6_PRIORITY!"12"}
#define STM32_IRQ_UART7_PRIORITY            ${doc.STM32_IRQ_UART7_PRIORITY!"12"}
#define STM32_IRQ_UART8_PRIORITY            ${doc.STM32_IRQ_UART8_PRIORITY!"12"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_ADCPRE                    ${doc.STM32_ADC_ADCPRE!"ADC_CCR_ADCPRE_DIV4"}
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_USE_ADC2                  ${doc.STM32_ADC_USE_ADC2!"FALSE"}
#define STM32_ADC_USE_ADC3                  ${doc.STM32_ADC_USE_ADC3!"FALSE"}
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 4)"}
#define STM32_ADC_ADC2_DMA_STREAM           ${doc.STM32_ADC_ADC2_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
#define STM32_ADC_ADC3_DMA_STREAM           ${doc.STM32_ADC_ADC3_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC2_DMA_PRIORITY         ${doc.STM32_ADC_ADC2_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC3_DMA_PRIORITY         ${doc.STM32_ADC_ADC3_DMA_PRIORITY!"2"}
#define STM32_ADC_IRQ_PRIORITY              ${doc.STM32_ADC_IRQ_PRIORITY!"6"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"6"}
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC2_DMA_IRQ_PRIORITY!"6"}
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC3_DMA_IRQ_PRIORITY!"6"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  ${doc.STM32_CAN_USE_CAN1!"FALSE"}
#define STM32_CAN_USE_CAN2                  ${doc.STM32_CAN_USE_CAN2!"FALSE"}
#define STM32_CAN_USE_CAN3                  ${doc.STM32_CAN_USE_CAN3!"FALSE"}
#define STM32_CAN_CAN1_IRQ_PRIORITY         ${doc.STM32_CAN_CAN1_IRQ_PRIORITY!"11"}
#define STM32_CAN_CAN2_IRQ_PRIORITY         ${doc.STM32_CAN_CAN2_IRQ_PRIORITY!"11"}
#define STM32_CAN_CAN3_IRQ_PRIORITY         ${doc.STM32_CAN_CAN3_IRQ_PRIORITY!"11"}

/*
 * CRY driver system settings.
 */
#define STM32_CRY_USE_CRYP1                 ${doc.STM32_CRY_USE_CRYP1!"FALSE"}
#define STM32_CRY_USE_HASH1                 ${doc.STM32_CRY_USE_HASH1!"FALSE"}
#define STM32_CRY_CRYP1_IRQ_PRIORITY        ${doc.STM32_CRY_CRYP1_IRQ_PRIORITY!"9"}
#define STM32_CRY_HASH1_IRQ_PRIORITY        ${doc.STM32_CRY_HASH1_IRQ_PRIORITY!"9"}
#define STM32_CRY_CRYP1_IN_DMA_STREAM       ${doc.STM32_CRY_CRYP1_IN_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 6)"}
#define STM32_CRY_CRYP1_OUT_DMA_STREAM      ${doc.STM32_CRY_CRYP1_OUT_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 5)"}
#define STM32_CRY_HASH1_DMA_STREAM          ${doc.STM32_CRY_HASH1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 7)"}
#define STM32_CRY_CRYP1_IN_DMA_PRIORITY     ${doc.STM32_CRY_CRYP1_IN_DMA_PRIORITY!"0"}
#define STM32_CRY_CRYP1_OUT_DMA_PRIORITY    ${doc.STM32_CRY_CRYP1_OUT_DMA_PRIORITY!"1"}
#define STM32_CRY_HASH1_DMA_PRIORITY        ${doc.STM32_CRY_HASH1_DMA_PRIORITY!"0"}
#define STM32_CRY_HASH_SIZE_THRESHOLD       ${doc.STM32_CRY_HASH_SIZE_THRESHOLD!"1024"}
#define STM32_CRY_CRYP_DMA_ERROR_HOOK(cryp) ${doc.STM32_CRY_CRYP_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}
#define STM32_CRY_HASH_DMA_ERROR_HOOK(cryp) ${doc.STM32_CRY_HASH_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

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
#define STM32_DAC_DAC1_CH1_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH1_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_DAC_DAC1_CH2_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH2_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}

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
#define STM32_GPT_USE_TIM9                  ${doc.STM32_GPT_USE_TIM9!"FALSE"}
#define STM32_GPT_USE_TIM10                 ${doc.STM32_GPT_USE_TIM10!"FALSE"}
#define STM32_GPT_USE_TIM11                 ${doc.STM32_GPT_USE_TIM11!"FALSE"}
#define STM32_GPT_USE_TIM12                 ${doc.STM32_GPT_USE_TIM12!"FALSE"}
#define STM32_GPT_USE_TIM13                 ${doc.STM32_GPT_USE_TIM13!"FALSE"}
#define STM32_GPT_USE_TIM14                 ${doc.STM32_GPT_USE_TIM14!"FALSE"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_USE_I2C2                  ${doc.STM32_I2C_USE_I2C2!"FALSE"}
#define STM32_I2C_USE_I2C3                  ${doc.STM32_I2C_USE_I2C3!"FALSE"}
#define STM32_I2C_USE_I2C4                  ${doc.STM32_I2C_USE_I2C4!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_RX_DMA_STREAM        ${doc.STM32_I2C_I2C1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 0)"}
#define STM32_I2C_I2C1_TX_DMA_STREAM        ${doc.STM32_I2C_I2C1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_I2C_I2C2_RX_DMA_STREAM        ${doc.STM32_I2C_I2C2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_I2C_I2C2_TX_DMA_STREAM        ${doc.STM32_I2C_I2C2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_I2C_I2C3_RX_DMA_STREAM        ${doc.STM32_I2C_I2C3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_I2C_I2C3_TX_DMA_STREAM        ${doc.STM32_I2C_I2C3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_I2C_I2C4_RX_DMA_STREAM        ${doc.STM32_I2C_I2C4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_I2C_I2C4_TX_DMA_STREAM        ${doc.STM32_I2C_I2C4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
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
#define STM32_ICU_USE_TIM9                  ${doc.STM32_ICU_USE_TIM9!"FALSE"}
#define STM32_ICU_USE_TIM10                 ${doc.STM32_ICU_USE_TIM10!"FALSE"}
#define STM32_ICU_USE_TIM11                 ${doc.STM32_ICU_USE_TIM11!"FALSE"}
#define STM32_ICU_USE_TIM12                 ${doc.STM32_ICU_USE_TIM12!"FALSE"}
#define STM32_ICU_USE_TIM13                 ${doc.STM32_ICU_USE_TIM13!"FALSE"}
#define STM32_ICU_USE_TIM14                 ${doc.STM32_ICU_USE_TIM14!"FALSE"}

/*
 * MAC driver system settings.
 */
#define STM32_MAC_TRANSMIT_BUFFERS          ${doc.STM32_MAC_TRANSMIT_BUFFERS!"2"}
#define STM32_MAC_RECEIVE_BUFFERS           ${doc.STM32_MAC_RECEIVE_BUFFERS!"4"}
#define STM32_MAC_BUFFERS_SIZE              ${doc.STM32_MAC_BUFFERS_SIZE!"1522"}
#define STM32_MAC_PHY_TIMEOUT               ${doc.STM32_MAC_PHY_TIMEOUT!"100"}
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     ${doc.STM32_MAC_ETH1_CHANGE_PHY_STATE!"TRUE"}
#define STM32_MAC_ETH1_IRQ_PRIORITY         ${doc.STM32_MAC_ETH1_IRQ_PRIORITY!"13"}
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       ${doc.STM32_MAC_IP_CHECKSUM_OFFLOAD!"0"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  ${doc.STM32_PWM_USE_TIM1!"FALSE"}
#define STM32_PWM_USE_TIM2                  ${doc.STM32_PWM_USE_TIM2!"FALSE"}
#define STM32_PWM_USE_TIM3                  ${doc.STM32_PWM_USE_TIM3!"FALSE"}
#define STM32_PWM_USE_TIM4                  ${doc.STM32_PWM_USE_TIM4!"FALSE"}
#define STM32_PWM_USE_TIM5                  ${doc.STM32_PWM_USE_TIM5!"FALSE"}
#define STM32_PWM_USE_TIM8                  ${doc.STM32_PWM_USE_TIM8!"FALSE"}
#define STM32_PWM_USE_TIM9                  ${doc.STM32_PWM_USE_TIM9!"FALSE"}
#define STM32_PWM_USE_TIM10                 ${doc.STM32_PWM_USE_TIM10!"FALSE"}
#define STM32_PWM_USE_TIM11                 ${doc.STM32_PWM_USE_TIM11!"FALSE"}
#define STM32_PWM_USE_TIM12                 ${doc.STM32_PWM_USE_TIM12!"FALSE"}
#define STM32_PWM_USE_TIM13                 ${doc.STM32_PWM_USE_TIM13!"FALSE"}
#define STM32_PWM_USE_TIM14                 ${doc.STM32_PWM_USE_TIM14!"FALSE"}

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
#define STM32_SDC_SDMMC1_DMA_STREAM         ${doc.STM32_SDC_SDMMC1_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 3)"}
#define STM32_SDC_SDMMC1_DMA_PRIORITY       ${doc.STM32_SDC_SDMMC1_DMA_PRIORITY!"3"}
#define STM32_SDC_SDMMC1_IRQ_PRIORITY       ${doc.STM32_SDC_SDMMC1_IRQ_PRIORITY!"9"}

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

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_USE_SPI2                  ${doc.STM32_SPI_USE_SPI2!"FALSE"}
#define STM32_SPI_USE_SPI3                  ${doc.STM32_SPI_USE_SPI3!"FALSE"}
#define STM32_SPI_USE_SPI4                  ${doc.STM32_SPI_USE_SPI4!"FALSE"}
#define STM32_SPI_USE_SPI5                  ${doc.STM32_SPI_USE_SPI5!"FALSE"}
#define STM32_SPI_USE_SPI6                  ${doc.STM32_SPI_USE_SPI6!"FALSE"}
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 0)"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 3)"}
#define STM32_SPI_SPI2_RX_DMA_STREAM        ${doc.STM32_SPI_SPI2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_SPI_SPI2_TX_DMA_STREAM        ${doc.STM32_SPI_SPI2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_SPI_SPI3_RX_DMA_STREAM        ${doc.STM32_SPI_SPI3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 0)"}
#define STM32_SPI_SPI3_TX_DMA_STREAM        ${doc.STM32_SPI_SPI3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_SPI_SPI4_RX_DMA_STREAM        ${doc.STM32_SPI_SPI4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 0)"}
#define STM32_SPI_SPI4_TX_DMA_STREAM        ${doc.STM32_SPI_SPI4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_SPI_SPI5_RX_DMA_STREAM        ${doc.STM32_SPI_SPI5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 3)"}
#define STM32_SPI_SPI5_TX_DMA_STREAM        ${doc.STM32_SPI_SPI5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 4)"}
#define STM32_SPI_SPI6_RX_DMA_STREAM        ${doc.STM32_SPI_SPI6_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 6)"}
#define STM32_SPI_SPI6_TX_DMA_STREAM        ${doc.STM32_SPI_SPI6_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 5)"}
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
#define STM32_UART_USART1_RX_DMA_STREAM     ${doc.STM32_UART_USART1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 5)"}
#define STM32_UART_USART1_TX_DMA_STREAM     ${doc.STM32_UART_USART1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 7)"}
#define STM32_UART_USART2_RX_DMA_STREAM     ${doc.STM32_UART_USART2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_UART_USART2_TX_DMA_STREAM     ${doc.STM32_UART_USART2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_UART_USART3_RX_DMA_STREAM     ${doc.STM32_UART_USART3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 1)"}
#define STM32_UART_USART3_TX_DMA_STREAM     ${doc.STM32_UART_USART3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_UART_UART4_RX_DMA_STREAM      ${doc.STM32_UART_UART4_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_UART_UART4_TX_DMA_STREAM      ${doc.STM32_UART_UART4_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_UART_UART5_RX_DMA_STREAM      ${doc.STM32_UART_UART5_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 0)"}
#define STM32_UART_UART5_TX_DMA_STREAM      ${doc.STM32_UART_UART5_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_UART_USART6_RX_DMA_STREAM     ${doc.STM32_UART_USART6_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
#define STM32_UART_USART6_TX_DMA_STREAM     ${doc.STM32_UART_USART6_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 7)"}
#define STM32_UART_UART7_RX_DMA_STREAM      ${doc.STM32_UART_UART7_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_UART_UART7_TX_DMA_STREAM      ${doc.STM32_UART_UART7_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 1)"}
#define STM32_UART_UART8_RX_DMA_STREAM      ${doc.STM32_UART_UART8_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_UART_UART8_TX_DMA_STREAM      ${doc.STM32_UART_UART8_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 0)"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_USART3_DMA_PRIORITY      ${doc.STM32_UART_USART3_DMA_PRIORITY!"0"}
#define STM32_UART_UART4_DMA_PRIORITY       ${doc.STM32_UART_UART4_DMA_PRIORITY!"0"}
#define STM32_UART_UART5_DMA_PRIORITY       ${doc.STM32_UART_UART5_DMA_PRIORITY!"0"}
#define STM32_UART_USART6_DMA_PRIORITY      ${doc.STM32_UART_USART6_DMA_PRIORITY!"0"}
#define STM32_UART_UART7_DMA_PRIORITY       ${doc.STM32_UART_UART7_DMA_PRIORITY!"0"}
#define STM32_UART_UART8_DMA_PRIORITY       ${doc.STM32_UART_UART8_DMA_PRIORITY!"0"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG1                  ${doc.STM32_USB_USE_OTG1!"FALSE"}
#define STM32_USB_USE_OTG2                  ${doc.STM32_USB_USE_OTG2!"FALSE"}
#define STM32_USB_OTG1_IRQ_PRIORITY         ${doc.STM32_USB_OTG1_IRQ_PRIORITY!"14"}
#define STM32_USB_OTG2_IRQ_PRIORITY         ${doc.STM32_USB_OTG2_IRQ_PRIORITY!"14"}
#define STM32_USB_OTG1_RX_FIFO_SIZE         ${doc.STM32_USB_OTG1_RX_FIFO_SIZE!"512"}
#define STM32_USB_OTG2_RX_FIFO_SIZE         ${doc.STM32_USB_OTG2_RX_FIFO_SIZE!"1024"}

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
