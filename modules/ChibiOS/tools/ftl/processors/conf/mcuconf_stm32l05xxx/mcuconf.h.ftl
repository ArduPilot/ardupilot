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
 * STM32L0xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 3...0        Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define STM32L0xx_MCUCONF
#define STM32L052_MCUCONF
#define STM32L053_MCUCONF
#define STM32L062_MCUCONF
#define STM32L063_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_VOS                           ${doc.STM32_VOS!"STM32_VOS_1P8"}
#define STM32_PVD_ENABLE                    ${doc.STM32_PVD_ENABLE!"FALSE"}
#define STM32_PLS                           ${doc.STM32_PLS!"STM32_PLS_LEV0"}
#define STM32_HSI16_ENABLED                 ${doc.STM32_HSI16_ENABLED!"TRUE"}
#define STM32_HSI16_DIVIDER_ENABLED         ${doc.STM32_HSI16_DIVIDER_ENABLED!"FALSE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"FALSE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"FALSE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"TRUE"}
#define STM32_ADC_CLOCK_ENABLED             ${doc.STM32_ADC_CLOCK_ENABLED!"TRUE"}
#define STM32_USB_CLOCK_ENABLED             ${doc.STM32_USB_CLOCK_ENABLED!"TRUE"}
#define STM32_MSIRANGE                      ${doc.STM32_MSIRANGE!"STM32_MSIRANGE_2M"}
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL"}
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_HSI16"}
#define STM32_PLLMUL_VALUE                  ${doc.STM32_PLLMUL_VALUE!"4"}
#define STM32_PLLDIV_VALUE                  ${doc.STM32_PLLDIV_VALUE!"2"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV1"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV1"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"STM32_MCOSEL_NOCLOCK"}
#define STM32_MCOPRE                        ${doc.STM32_MCOPRE!"STM32_MCOPRE_DIV1"}

/*
 * Peripherals clock sources.
 */
#define STM32_USART1SEL                     ${doc.STM32_USART1SEL!"STM32_USART1SEL_APB"}
#define STM32_USART2SEL                     ${doc.STM32_USART2SEL!"STM32_USART2SEL_APB"}
#define STM32_LPUART1SEL                    ${doc.STM32_LPUART1SEL!"STM32_LPUART1SEL_APB"}
#define STM32_I2C1SEL                       ${doc.STM32_I2C1SEL!"STM32_I2C1SEL_APB"}
#define STM32_LPTIM1SEL                     ${doc.STM32_LPTIM1SEL!"STM32_LPTIM1SEL_APB"}
#define STM32_HSI48SEL                      ${doc.STM32_HSI48SEL!"STM32_HSI48SEL_HSI48"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSE"}
#define STM32_RTCPRE                        ${doc.STM32_RTCPRE!"STM32_RTCPRE_DIV2"}

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_1_PRIORITY          ${doc.STM32_IRQ_EXTI0_1_PRIORITY!"3"}
#define STM32_IRQ_EXTI2_3_PRIORITY          ${doc.STM32_IRQ_EXTI2_3_PRIORITY!"3"}
#define STM32_IRQ_EXTI4_15_PRIORITY         ${doc.STM32_IRQ_EXTI4_15_PRIORITY!"3"}
#define STM32_IRQ_EXTI16_PRIORITY           ${doc.STM32_IRQ_EXTI16_PRIORITY!"3"}
#define STM32_IRQ_EXTI17_20_PRIORITY        ${doc.STM32_IRQ_EXTI17_20_PRIORITY!"3"}
#define STM32_IRQ_EXTI21_22_PRIORITY        ${doc.STM32_IRQ_EXTI21_22_PRIORITY!"3"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"3"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"3"}
#define STM32_IRQ_LPUART1_PRIORITY          ${doc.STM32_IRQ_LPUART1_PRIORITY!"3"}

#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"1"}
#define STM32_IRQ_TIM6_PRIORITY             ${doc.STM32_IRQ_TIM6_PRIORITY!"1"}
#define STM32_IRQ_TIM21_PRIORITY            ${doc.STM32_IRQ_TIM21_PRIORITY!"1"}
#define STM32_IRQ_TIM22_PRIORITY            ${doc.STM32_IRQ_TIM22_PRIORITY!"1"}

/*
 * ADC driver system settings.
 * Note, IRQ is shared with EXT channels 21 and 22.
 */
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_ADC1_CFGR2                ${doc.STM32_ADC_ADC1_CFGR2!"ADC_CFGR2_CKMODE_ADCCLK"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"2"}
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 1)"}
#define STM32_ADC_PRESCALER_VALUE           ${doc.STM32_ADC_PRESCALER_VALUE!"2"}

/*
 * DAC driver system settings.
 */
#define STM32_DAC_USE_DAC1_CH1              ${doc.STM32_DAC_USE_DAC1_CH1!"FALSE"}
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_IRQ_PRIORITY!"3"}
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     ${doc.STM32_DAC_DAC1_CH1_DMA_PRIORITY!"2"}
#define STM32_DAC_DAC1_CH1_DMA_STREAM       ${doc.STM32_DAC_DAC1_CH1_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM2                  ${doc.STM32_GPT_USE_TIM2!"FALSE"}
#define STM32_GPT_USE_TIM6                  ${doc.STM32_GPT_USE_TIM6!"FALSE"}
#define STM32_GPT_USE_TIM21                 ${doc.STM32_GPT_USE_TIM21!"FALSE"}
#define STM32_GPT_USE_TIM22                 ${doc.STM32_GPT_USE_TIM22!"FALSE"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_USE_I2C2                  ${doc.STM32_I2C_USE_I2C2!"FALSE"}
#define STM32_I2C_USE_I2C3                  ${doc.STM32_I2C_USE_I2C3!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_IRQ_PRIORITY         ${doc.STM32_I2C_I2C1_IRQ_PRIORITY!"3"}
#define STM32_I2C_I2C2_IRQ_PRIORITY         ${doc.STM32_I2C_I2C2_IRQ_PRIORITY!"3"}
#define STM32_I2C_I2C3_IRQ_PRIORITY         ${doc.STM32_I2C_I2C3_IRQ_PRIORITY!"3"}
#define STM32_I2C_USE_DMA                   ${doc.STM32_I2C_USE_DMA!"TRUE"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"1"}
#define STM32_I2C_I2C2_DMA_PRIORITY         ${doc.STM32_I2C_I2C2_DMA_PRIORITY!"1"}
#define STM32_I2C_I2C3_DMA_PRIORITY         ${doc.STM32_I2C_I2C3_DMA_PRIORITY!"1"}
#define STM32_I2C_I2C1_RX_DMA_STREAM        ${doc.STM32_I2C_I2C1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_I2C_I2C1_TX_DMA_STREAM        ${doc.STM32_I2C_I2C1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_I2C_I2C2_RX_DMA_STREAM        ${doc.STM32_I2C_I2C2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_I2C_I2C2_TX_DMA_STREAM        ${doc.STM32_I2C_I2C2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_I2C_I2C3_RX_DMA_STREAM        ${doc.STM32_I2C_I2C3_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_I2C_I2C3_TX_DMA_STREAM        ${doc.STM32_I2C_I2C3_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      ${doc.STM32_I2C_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM2                  ${doc.STM32_ICU_USE_TIM2!"FALSE"}
#define STM32_ICU_USE_TIM21                 ${doc.STM32_ICU_USE_TIM21!"FALSE"}
#define STM32_ICU_USE_TIM22                 ${doc.STM32_ICU_USE_TIM22!"FALSE"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM2                  ${doc.STM32_PWM_USE_TIM2!"FALSE"}
#define STM32_PWM_USE_TIM21                 ${doc.STM32_PWM_USE_TIM21!"FALSE"}
#define STM32_PWM_USE_TIM22                 ${doc.STM32_PWM_USE_TIM22!"FALSE"}

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"TRUE"}
#define STM32_SERIAL_USE_LPUART1            ${doc.STM32_SERIAL_USE_LPUART1!"FALSE"}

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                ${doc.STM32_SIO_USE_USART1!"FALSE"}
#define STM32_SIO_USE_USART2                ${doc.STM32_SIO_USE_USART2!"FALSE"}
#define STM32_SIO_USE_LPUART1               ${doc.STM32_SIO_USE_LPUART1!"FALSE"}

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_USE_SPI2                  ${doc.STM32_SPI_USE_SPI2!"FALSE"}
#define STM32_SPI_SPI1_DMA_PRIORITY         ${doc.STM32_SPI_SPI1_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI2_DMA_PRIORITY         ${doc.STM32_SPI_SPI2_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI1_IRQ_PRIORITY         ${doc.STM32_SPI_SPI1_IRQ_PRIORITY!"1"}
#define STM32_SPI_SPI2_IRQ_PRIORITY         ${doc.STM32_SPI_SPI2_IRQ_PRIORITY!"1"}
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 2)"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 3)"}
#define STM32_SPI_SPI2_RX_DMA_STREAM        ${doc.STM32_SPI_SPI2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_SPI_SPI2_TX_DMA_STREAM        ${doc.STM32_SPI_SPI2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_SPI_DMA_ERROR_HOOK(spip)      ${doc.STM32_SPI_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               ${doc.STM32_ST_IRQ_PRIORITY!"2"}
#define STM32_ST_USE_TIMER                  ${doc.STM32_ST_USE_TIMER!"21"}

/*
 * TRNG driver system settings.
 */
#define STM32_TRNG_USE_RNG1                 ${doc.STM32_TRNG_USE_RNG1!"FALSE"}

/*
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               ${doc.STM32_UART_USE_USART1!"FALSE"}
#define STM32_UART_USE_USART2               ${doc.STM32_UART_USE_USART2!"FALSE"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_USART1_RX_DMA_STREAM     ${doc.STM32_UART_USART1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 5)"}
#define STM32_UART_USART1_TX_DMA_STREAM     ${doc.STM32_UART_USART1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 4)"}
#define STM32_UART_USART2_RX_DMA_STREAM     ${doc.STM32_UART_USART2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 6)"}
#define STM32_UART_USART2_TX_DMA_STREAM     ${doc.STM32_UART_USART2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID(1, 7)"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

#endif /* MCUCONF_H */
