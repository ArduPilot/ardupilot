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
 * STM32F3xx drivers configuration.
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

#define STM32F3xx_MCUCONF
#define STM32F303_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_PVD_ENABLE                    ${doc.STM32_PVD_ENABLE!"FALSE"}
#define STM32_PLS                           ${doc.STM32_PLS!"STM32_PLS_LEV0"}
#define STM32_HSI_ENABLED                   ${doc.STM32_HSI_ENABLED!"TRUE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"TRUE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"TRUE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"FALSE"}
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL"}
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_HSE"}
#define STM32_PREDIV_VALUE                  ${doc.STM32_PREDIV_VALUE!"1"}
#define STM32_PLLMUL_VALUE                  ${doc.STM32_PLLMUL_VALUE!"9"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV2"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV2"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"STM32_MCOSEL_NOCLOCK"}
#define STM32_ADC12PRES                     ${doc.STM32_ADC12PRES!"STM32_ADC12PRES_DIV1"}
#define STM32_ADC34PRES                     ${doc.STM32_ADC34PRES!"STM32_ADC34PRES_DIV1"}
#define STM32_USART1SW                      ${doc.STM32_USART1SW!"STM32_USART1SW_PCLK"}
#define STM32_USART2SW                      ${doc.STM32_USART2SW!"STM32_USART2SW_PCLK"}
#define STM32_USART3SW                      ${doc.STM32_USART3SW!"STM32_USART3SW_PCLK"}
#define STM32_UART4SW                       ${doc.STM32_UART4SW!"STM32_UART4SW_PCLK"}
#define STM32_UART5SW                       ${doc.STM32_UART5SW!"STM32_UART5SW_PCLK"}
#define STM32_I2C1SW                        ${doc.STM32_I2C1SW!"STM32_I2C1SW_SYSCLK"}
#define STM32_I2C2SW                        ${doc.STM32_I2C2SW!"STM32_I2C2SW_SYSCLK"}
#define STM32_TIM1SW                        ${doc.STM32_TIM1SW!"STM32_TIM1SW_PCLK2"}
#define STM32_TIM8SW                        ${doc.STM32_TIM8SW!"STM32_TIM8SW_PCLK2"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSI"}
#define STM32_USB_CLOCK_REQUIRED            ${doc.STM32_USB_CLOCK_REQUIRED!"TRUE"}
#define STM32_USBPRE                        ${doc.STM32_USBPRE!"STM32_USBPRE_DIV1P5"}

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
#define STM32_IRQ_EXTI17_PRIORITY           ${doc.STM32_IRQ_EXTI17_PRIORITY!"15"}
#define STM32_IRQ_EXTI18_PRIORITY           ${doc.STM32_IRQ_EXTI18_PRIORITY!"6"}
#define STM32_IRQ_EXTI19_PRIORITY           ${doc.STM32_IRQ_EXTI19_PRIORITY!"15"}
#define STM32_IRQ_EXTI20_PRIORITY           ${doc.STM32_IRQ_EXTI20_PRIORITY!"15"}
#define STM32_IRQ_EXTI21_22_29_PRIORITY     ${doc.STM32_IRQ_EXTI21_22_29_PRIORITY!"6"}
#define STM32_IRQ_EXTI30_32_PRIORITY        ${doc.STM32_IRQ_EXTI30_32_PRIORITY!"6"}
#define STM32_IRQ_EXTI33_PRIORITY           ${doc.STM32_IRQ_EXTI33_PRIORITY!"6"}
#define STM32_IRQ_TIM1_BRK_TIM15_PRIORITY   ${doc.STM32_IRQ_TIM1_BRK_TIM15_PRIORITY!"7"}
#define STM32_IRQ_TIM1_UP_TIM16_PRIORITY    ${doc.STM32_IRQ_TIM1_UP_TIM16_PRIORITY!"7"}
#define STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY ${doc.STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY!"7"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"7"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_DUAL_MODE                 ${doc.STM32_ADC_DUAL_MODE!"FALSE"}
#define STM32_ADC_COMPACT_SAMPLES           ${doc.STM32_ADC_COMPACT_SAMPLES!"FALSE"}
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_USE_ADC2                  ${doc.STM32_ADC_USE_ADC2!"FALSE"}
#define STM32_ADC_USE_ADC3                  ${doc.STM32_ADC_USE_ADC3!"FALSE"}
#define STM32_ADC_USE_ADC4                  ${doc.STM32_ADC_USE_ADC4!"FALSE"}
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 1)"}
#define STM32_ADC_ADC2_DMA_STREAM           ${doc.STM32_ADC_ADC2_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 1)"}
#define STM32_ADC_ADC3_DMA_STREAM           ${doc.STM32_ADC_ADC3_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 5)"}
#define STM32_ADC_ADC4_DMA_STREAM           ${doc.STM32_ADC_ADC4_DMA_STREAM!"STM32_DMA_STREAM_ID(2, 2)"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC2_DMA_PRIORITY         ${doc.STM32_ADC_ADC2_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC3_DMA_PRIORITY         ${doc.STM32_ADC_ADC3_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC4_DMA_PRIORITY         ${doc.STM32_ADC_ADC4_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC12_IRQ_PRIORITY        ${doc.STM32_ADC_ADC12_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC3_IRQ_PRIORITY         ${doc.STM32_ADC_ADC3_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC4_IRQ_PRIORITY         ${doc.STM32_ADC_ADC4_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC2_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC3_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC4_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC4_DMA_IRQ_PRIORITY!"5"}
#define STM32_ADC_ADC12_CLOCK_MODE          ${doc.STM32_ADC_ADC12_CLOCK_MODE!"ADC_CCR_CKMODE_AHB_DIV1"}
#define STM32_ADC_ADC34_CLOCK_MODE          ${doc.STM32_ADC_ADC34_CLOCK_MODE!"ADC_CCR_CKMODE_AHB_DIV1"}

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  ${doc.STM32_CAN_USE_CAN1!"FALSE"}
#define STM32_CAN_CAN1_IRQ_PRIORITY         ${doc.STM32_CAN_CAN1_IRQ_PRIORITY!"11"}

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 ${doc.STM32_DAC_DUAL_MODE!"FALSE"}
#define STM32_DAC_USE_DAC1_CH1              ${doc.STM32_DAC_USE_DAC1_CH1!"TRUE"}
#define STM32_DAC_USE_DAC1_CH2              ${doc.STM32_DAC_USE_DAC1_CH2!"TRUE"}
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_IRQ_PRIORITY!"10"}
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     ${doc.STM32_DAC_DAC1_CH2_IRQ_PRIORITY!"10"}
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH2_DMA_PRIORITY!"2"}

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  ${doc.STM32_GPT_USE_TIM1!"FALSE"}
#define STM32_GPT_USE_TIM2                  ${doc.STM32_GPT_USE_TIM2!"FALSE"}
#define STM32_GPT_USE_TIM3                  ${doc.STM32_GPT_USE_TIM3!"FALSE"}
#define STM32_GPT_USE_TIM4                  ${doc.STM32_GPT_USE_TIM4!"FALSE"}
#define STM32_GPT_USE_TIM6                  ${doc.STM32_GPT_USE_TIM6!"FALSE"}
#define STM32_GPT_USE_TIM7                  ${doc.STM32_GPT_USE_TIM7!"FALSE"}
#define STM32_GPT_USE_TIM8                  ${doc.STM32_GPT_USE_TIM8!"FALSE"}
#define STM32_GPT_USE_TIM15                 ${doc.STM32_GPT_USE_TIM15!"FALSE"}
#define STM32_GPT_USE_TIM16                 ${doc.STM32_GPT_USE_TIM16!"FALSE"}
#define STM32_GPT_USE_TIM17                 ${doc.STM32_GPT_USE_TIM17!"FALSE"}
#define STM32_GPT_TIM1_IRQ_PRIORITY         ${doc.STM32_GPT_TIM1_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM2_IRQ_PRIORITY         ${doc.STM32_GPT_TIM2_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM3_IRQ_PRIORITY         ${doc.STM32_GPT_TIM3_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM4_IRQ_PRIORITY         ${doc.STM32_GPT_TIM4_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM6_IRQ_PRIORITY         ${doc.STM32_GPT_TIM6_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM7_IRQ_PRIORITY         ${doc.STM32_GPT_TIM7_IRQ_PRIORITY!"7"}
#define STM32_GPT_TIM8_IRQ_PRIORITY         ${doc.STM32_GPT_TIM8_IRQ_PRIORITY!"7"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_USE_I2C2                  ${doc.STM32_I2C_USE_I2C2!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_IRQ_PRIORITY         ${doc.STM32_I2C_I2C1_IRQ_PRIORITY!"10"}
#define STM32_I2C_I2C2_IRQ_PRIORITY         ${doc.STM32_I2C_I2C2_IRQ_PRIORITY!"10"}
#define STM32_I2C_USE_DMA                   ${doc.STM32_I2C_USE_DMA!"TRUE"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"1"}
#define STM32_I2C_I2C2_DMA_PRIORITY         ${doc.STM32_I2C_I2C2_DMA_PRIORITY!"1"}
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      ${doc.STM32_I2C_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  ${doc.STM32_ICU_USE_TIM1!"FALSE"}
#define STM32_ICU_USE_TIM2                  ${doc.STM32_ICU_USE_TIM2!"FALSE"}
#define STM32_ICU_USE_TIM3                  ${doc.STM32_ICU_USE_TIM3!"FALSE"}
#define STM32_ICU_USE_TIM4                  ${doc.STM32_ICU_USE_TIM4!"FALSE"}
#define STM32_ICU_USE_TIM8                  ${doc.STM32_ICU_USE_TIM8!"FALSE"}
#define STM32_ICU_USE_TIM15                 ${doc.STM32_ICU_USE_TIM15!"FALSE"}
#define STM32_ICU_TIM1_IRQ_PRIORITY         ${doc.STM32_ICU_TIM1_IRQ_PRIORITY!"7"}
#define STM32_ICU_TIM2_IRQ_PRIORITY         ${doc.STM32_ICU_TIM2_IRQ_PRIORITY!"7"}
#define STM32_ICU_TIM3_IRQ_PRIORITY         ${doc.STM32_ICU_TIM3_IRQ_PRIORITY!"7"}
#define STM32_ICU_TIM4_IRQ_PRIORITY         ${doc.STM32_ICU_TIM4_IRQ_PRIORITY!"7"}
#define STM32_ICU_TIM8_IRQ_PRIORITY         ${doc.STM32_ICU_TIM8_IRQ_PRIORITY!"7"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  ${doc.STM32_PWM_USE_TIM1!"FALSE"}
#define STM32_PWM_USE_TIM2                  ${doc.STM32_PWM_USE_TIM2!"FALSE"}
#define STM32_PWM_USE_TIM3                  ${doc.STM32_PWM_USE_TIM3!"FALSE"}
#define STM32_PWM_USE_TIM4                  ${doc.STM32_PWM_USE_TIM4!"FALSE"}
#define STM32_PWM_USE_TIM8                  ${doc.STM32_PWM_USE_TIM8!"FALSE"}
#define STM32_PWM_USE_TIM15                 ${doc.STM32_PWM_USE_TIM15!"FALSE"}
#define STM32_PWM_USE_TIM16                 ${doc.STM32_PWM_USE_TIM16!"FALSE"}
#define STM32_PWM_USE_TIM17                 ${doc.STM32_PWM_USE_TIM17!"FALSE"}
#define STM32_PWM_TIM1_IRQ_PRIORITY         ${doc.STM32_PWM_TIM1_IRQ_PRIORITY!"7"}
#define STM32_PWM_TIM2_IRQ_PRIORITY         ${doc.STM32_PWM_TIM2_IRQ_PRIORITY!"7"}
#define STM32_PWM_TIM3_IRQ_PRIORITY         ${doc.STM32_PWM_TIM3_IRQ_PRIORITY!"7"}
#define STM32_PWM_TIM4_IRQ_PRIORITY         ${doc.STM32_PWM_TIM4_IRQ_PRIORITY!"7"}
#define STM32_PWM_TIM8_IRQ_PRIORITY         ${doc.STM32_PWM_TIM8_IRQ_PRIORITY!"7"}

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               ${doc.STM32_RTC_PRESA_VALUE!"32"}
#define STM32_RTC_PRESS_VALUE               ${doc.STM32_RTC_PRESS_VALUE!"1024"}
#define STM32_RTC_CR_INIT                   ${doc.STM32_RTC_CR_INIT!"0"}
#define STM32_RTC_TAMPCR_INIT               ${doc.STM32_RTC_TAMPCR_INIT!"0"}

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"FALSE"}
#define STM32_SERIAL_USE_USART3             ${doc.STM32_SERIAL_USE_USART3!"FALSE"}
#define STM32_SERIAL_USE_UART4              ${doc.STM32_SERIAL_USE_UART4!"FALSE"}
#define STM32_SERIAL_USE_UART5              ${doc.STM32_SERIAL_USE_UART5!"FALSE"}
#define STM32_SERIAL_USART1_PRIORITY        ${doc.STM32_SERIAL_USART1_PRIORITY!"12"}
#define STM32_SERIAL_USART2_PRIORITY        ${doc.STM32_SERIAL_USART2_PRIORITY!"12"}
#define STM32_SERIAL_USART3_PRIORITY        ${doc.STM32_SERIAL_USART3_PRIORITY!"12"}
#define STM32_SERIAL_UART4_PRIORITY         ${doc.STM32_SERIAL_UART4_PRIORITY!"12"}
#define STM32_SERIAL_UART5_PRIORITY         ${doc.STM32_SERIAL_UART5_PRIORITY!"12"}

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_USE_SPI2                  ${doc.STM32_SPI_USE_SPI2!"FALSE"}
#define STM32_SPI_USE_SPI3                  ${doc.STM32_SPI_USE_SPI3!"FALSE"}
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
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               ${doc.STM32_UART_USE_USART1!"FALSE"}
#define STM32_UART_USE_USART2               ${doc.STM32_UART_USE_USART2!"FALSE"}
#define STM32_UART_USE_USART3               ${doc.STM32_UART_USE_USART3!"FALSE"}
#define STM32_UART_USART1_IRQ_PRIORITY      ${doc.STM32_UART_USART1_IRQ_PRIORITY!"12"}
#define STM32_UART_USART2_IRQ_PRIORITY      ${doc.STM32_UART_USART2_IRQ_PRIORITY!"12"}
#define STM32_UART_USART3_IRQ_PRIORITY      ${doc.STM32_UART_USART3_IRQ_PRIORITY!"12"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_USART3_DMA_PRIORITY      ${doc.STM32_UART_USART3_DMA_PRIORITY!"0"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_USB1                  ${doc.STM32_USB_USE_USB1!"FALSE"}
#define STM32_USB_LOW_POWER_ON_SUSPEND      ${doc.STM32_USB_LOW_POWER_ON_SUSPEND!"FALSE"}
#define STM32_USB_USB1_HP_IRQ_PRIORITY      ${doc.STM32_USB_USB1_HP_IRQ_PRIORITY!"13"}
#define STM32_USB_USB1_LP_IRQ_PRIORITY      ${doc.STM32_USB_USB1_LP_IRQ_PRIORITY!"14"}

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

#endif /* MCUCONF_H */
