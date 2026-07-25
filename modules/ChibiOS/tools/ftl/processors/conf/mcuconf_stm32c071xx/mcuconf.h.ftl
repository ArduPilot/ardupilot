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
 * STM32C071 drivers configuration.
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

#ifndef MCUCONF_H
#define MCUCONF_H

#define STM32C0xx_MCUCONF
#define STM32C071_MCUCONF

/*
 * HAL driver general settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}
#define STM32_CLOCK_DYNAMIC                 ${doc.STM32_CLOCK_DYNAMIC!"FALSE"}

/*
 * SYSCFG settings.
 */
#define STM32_SYSCFG_CFGR1                  ${doc.STM32_SYSCFG_CFGR1!"(0U)"}
#define STM32_SYSCFG_CFGR2                  ${doc.STM32_SYSCFG_CFGR2!"(0U)"}
#define STM32_SYSCFG_CFGR3                  ${doc.STM32_SYSCFG_CFGR3!"(0U)"}

/*
 * PWR settings.
 */
#define STM32_PWR_CR2                       ${doc.STM32_PWR_CR2!"(PWR_CR2_PVM_VDDIO2_ENABLED)"}
#define STM32_PWR_CR3                       ${doc.STM32_PWR_CR3!"(PWR_CR3_EIWUL)"}
#define STM32_PWR_CR4                       ${doc.STM32_PWR_CR4!"(0U)"}
#define STM32_PWR_PUCRA                     ${doc.STM32_PWR_PUCRA!"(0U)"}
#define STM32_PWR_PDCRA                     ${doc.STM32_PWR_PDCRA!"(0U)"}
#define STM32_PWR_PUCRB                     ${doc.STM32_PWR_PUCRB!"(0U)"}
#define STM32_PWR_PDCRB                     ${doc.STM32_PWR_PDCRB!"(0U)"}
#define STM32_PWR_PUCRC                     ${doc.STM32_PWR_PUCRC!"(0U)"}
#define STM32_PWR_PDCRC                     ${doc.STM32_PWR_PDCRC!"(0U)"}
#define STM32_PWR_PUCRD                     ${doc.STM32_PWR_PUCRD!"(0U)"}
#define STM32_PWR_PDCRD                     ${doc.STM32_PWR_PDCRD!"(0U)"}
#define STM32_PWR_PUCRF                     ${doc.STM32_PWR_PUCRF!"(0U)"}
#define STM32_PWR_PDCRF                     ${doc.STM32_PWR_PDCRF!"(0U)"}

/*
 * FLASH settings.
 */
#define STM32_FLASH_ACR                     ${doc.STM32_FLASH_ACR!"(FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN)"}

/*
 * Clock settings.
 */
#define STM32_HSIDIV_VALUE                  ${doc.STM32_HSIDIV_VALUE!"4"}
#define STM32_HSIKERDIV_VALUE               ${doc.STM32_HSIKERDIV_VALUE!"3"}
#define STM32_HSI48_ENABLED                 ${doc.STM32_HSI48_ENABLED!"TRUE"}
#define STM32_HSIUSB48_ENABLED              ${doc.STM32_HSIUSB48_ENABLED!"TRUE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"FALSE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"FALSE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"FALSE"}
#define STM32_SW                            ${doc.STM32_SW!"RCC_CFGR_SW_HSISYS"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"RCC_CFGR_HPRE_DIV1"}
#define STM32_PPRE                          ${doc.STM32_PPRE!"RCC_CFGR_PPRE_DIV1"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"RCC_CFGR_MCOSEL_NOCLOCK"}
#define STM32_MCOPRE                        ${doc.STM32_MCOPRE!"RCC_CFGR_MCOPRE_DIV1"}
#define STM32_MCO2SEL                       ${doc.STM32_MCO2SEL!"RCC_CFGR_MCO2SEL_NOCLOCK"}
#define STM32_MCO2PRE                       ${doc.STM32_MCO2PRE!"RCC_CFGR_MCO2PRE_DIV1"}
#define STM32_LSCOSEL                       ${doc.STM32_LSCOSEL!"RCC_CSR1_LSCOSEL_NOCLOCK"}

/*
 * Peripherals clock sources.
 */
#define STM32_ADCSEL                        ${doc.STM32_ADCSEL!"RCC_CCIPR_ADCSEL_SYSCLK"}
#define STM32_I2C1SEL                       ${doc.STM32_I2C1SEL!"RCC_CCIPR_I2C1SEL_PCLK"}
#define STM32_I2S1SEL                       ${doc.STM32_I2S1SEL!"RCC_CCIPR_I2S1SEL_SYSCLK"}
#define STM32_USART1SEL                     ${doc.STM32_USART1SEL!"RCC_CCIPR_USART1SEL_PCLK"}
#define STM32_USBSEL                        ${doc.STM32_USBSEL!"RCC_CCIPR2_USBSEL_HSIUSB48"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"RCC_CSR1_RTCSEL_NOCLOCK"}

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_1_PRIORITY          ${doc.STM32_IRQ_EXTI0_1_PRIORITY!"3"}
#define STM32_IRQ_EXTI2_3_PRIORITY          ${doc.STM32_IRQ_EXTI2_3_PRIORITY!"3"}
#define STM32_IRQ_EXTI4_15_PRIORITY         ${doc.STM32_IRQ_EXTI4_15_PRIORITY!"3"}

#define STM32_IRQ_I2C1_PRIORITY             ${doc.STM32_IRQ_I2C1_PRIORITY!"3"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"2"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"2"}

#define STM32_IRQ_TIM1_UP_PRIORITY          ${doc.STM32_IRQ_TIM1_UP_PRIORITY!"1"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"1"}
#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"1"}
#define STM32_IRQ_TIM3_PRIORITY             ${doc.STM32_IRQ_TIM3_PRIORITY!"1"}
#define STM32_IRQ_TIM14_PRIORITY            ${doc.STM32_IRQ_TIM14_PRIORITY!"1"}
#define STM32_IRQ_TIM16_PRIORITY            ${doc.STM32_IRQ_TIM16_PRIORITY!"1"}
#define STM32_IRQ_TIM17_PRIORITY            ${doc.STM32_IRQ_TIM17_PRIORITY!"1"}

#define STM32_IRQ_USB1_PRIORITY             ${doc.STM32_IRQ_USB1_PRIORITY!"3"}

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  ${doc.STM32_ADC_USE_ADC1!"FALSE"}
#define STM32_ADC_ADC1_CFGR2                ${doc.STM32_ADC_ADC1_CFGR2!"ADC_CFGR2_CKMODE_ADCCLK"}
#define STM32_ADC_ADC1_DMA_PRIORITY         ${doc.STM32_ADC_ADC1_DMA_PRIORITY!"2"}
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     ${doc.STM32_ADC_ADC1_DMA_IRQ_PRIORITY!"2"}
#define STM32_ADC_ADC1_DMA_STREAM           ${doc.STM32_ADC_ADC1_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_ADC_PRESCALER_VALUE           ${doc.STM32_ADC_PRESCALER_VALUE!"2"}

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  ${doc.STM32_GPT_USE_TIM1!"FALSE"}
#define STM32_GPT_USE_TIM3                  ${doc.STM32_GPT_USE_TIM3!"FALSE"}
#define STM32_GPT_USE_TIM14                 ${doc.STM32_GPT_USE_TIM14!"FALSE"}
#define STM32_GPT_USE_TIM16                 ${doc.STM32_GPT_USE_TIM16!"FALSE"}
#define STM32_GPT_USE_TIM17                 ${doc.STM32_GPT_USE_TIM17!"FALSE"}

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  ${doc.STM32_I2C_USE_I2C1!"FALSE"}
#define STM32_I2C_BUSY_TIMEOUT              ${doc.STM32_I2C_BUSY_TIMEOUT!"50"}
#define STM32_I2C_I2C1_DMA_CHANNEL          ${doc.STM32_I2C_I2C1_DMA_CHANNEL!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_I2C_I2C1_DMA_PRIORITY         ${doc.STM32_I2C_I2C1_DMA_PRIORITY!"3"}
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      ${doc.STM32_I2C_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  ${doc.STM32_ICU_USE_TIM1!"FALSE"}
#define STM32_ICU_USE_TIM3                  ${doc.STM32_ICU_USE_TIM3!"FALSE"}

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  ${doc.STM32_PWM_USE_TIM1!"FALSE"}
#define STM32_PWM_USE_TIM3                  ${doc.STM32_PWM_USE_TIM3!"FALSE"}
#define STM32_PWM_USE_TIM14                 ${doc.STM32_PWM_USE_TIM14!"FALSE"}
#define STM32_PWM_USE_TIM16                 ${doc.STM32_PWM_USE_TIM16!"FALSE"}
#define STM32_PWM_USE_TIM17                 ${doc.STM32_PWM_USE_TIM17!"FALSE"}

/*
 * RTC driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"FALSE"}

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                ${doc.STM32_SIO_USE_USART1!"FALSE"}
#define STM32_SIO_USE_USART2                ${doc.STM32_SIO_USE_USART2!"FALSE"}

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  ${doc.STM32_SPI_USE_SPI1!"FALSE"}
#define STM32_SPI_SPI1_RX_DMA_STREAM        ${doc.STM32_SPI_SPI1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI1_TX_DMA_STREAM        ${doc.STM32_SPI_SPI1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_SPI_SPI1_DMA_PRIORITY         ${doc.STM32_SPI_SPI1_DMA_PRIORITY!"1"}
#define STM32_SPI_SPI1_IRQ_PRIORITY         ${doc.STM32_SPI_SPI1_IRQ_PRIORITY!"2"}
#define STM32_SPI_DMA_ERROR_HOOK(spip)      ${doc.STM32_SPI_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               ${doc.STM32_ST_IRQ_PRIORITY!"2"}
#define STM32_ST_USE_TIMER                  ${doc.STM32_ST_USE_TIMER!"16"}

/*
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               ${doc.STM32_UART_USE_USART1!"FALSE"}
#define STM32_UART_USE_USART2               ${doc.STM32_UART_USE_USART2!"FALSE"}
#define STM32_UART_USART1_RX_DMA_STREAM     ${doc.STM32_UART_USART1_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART1_TX_DMA_STREAM     ${doc.STM32_UART_USART1_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART2_RX_DMA_STREAM     ${doc.STM32_UART_USART2_RX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART2_TX_DMA_STREAM     ${doc.STM32_UART_USART2_TX_DMA_STREAM!"STM32_DMA_STREAM_ID_ANY"}
#define STM32_UART_USART1_DMA_PRIORITY      ${doc.STM32_UART_USART1_DMA_PRIORITY!"0"}
#define STM32_UART_USART2_DMA_PRIORITY      ${doc.STM32_UART_USART2_DMA_PRIORITY!"0"}
#define STM32_UART_DMA_ERROR_HOOK(uartp)    ${doc.STM32_UART_DMA_ERROR_HOOK!"osalSysHalt(\"DMA failure\")"}

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

#endif /* MCUCONF_H */
