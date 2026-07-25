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

/*
 * STM32U0xx drivers configuration.
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

#define STM32U0xx_MCUCONF
#define STM32U073_MCUCONF
#define STM32U083_MCUCONF

/*
 * HAL driver general settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_CLOCK_DYNAMIC                 FALSE

/*
 * SYSCFG settings.
 */
#define STM32_SYSCFG_CFGR1                  (0U)

/*
 * PWR settings.
 */
#define STM32_PWR_CR1                       (PWR_CR1_VOS_RANGE1 | PWR_CR1_FPD_STOP)
#define STM32_PWR_CR2                       (PWR_CR2_USV)
#define STM32_PWR_CR3                       (0U)
#define STM32_PWR_CR4                       (0U)
#define STM32_PWR_PUCRA                     (0U)
#define STM32_PWR_PDCRA                     (0U)
#define STM32_PWR_PUCRB                     (0U)
#define STM32_PWR_PDCRB                     (0U)
#define STM32_PWR_PUCRC                     (0U)
#define STM32_PWR_PDCRC                     (0U)
#define STM32_PWR_PUCRD                     (0U)
#define STM32_PWR_PDCRD                     (0U)
#define STM32_PWR_PUCRE                     (0U)
#define STM32_PWR_PDCRE                     (0U)
#define STM32_PWR_PUCRF                     (0U)
#define STM32_PWR_PDCRF                     (0U)

/*
 * FLASH settings.
 */
#define STM32_FLASH_ACR                     (FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN)

/*
 * Clock settings.
 */
#define STM32_HSI16_ENABLED                 FALSE
#define STM32_HSI48_ENABLED                 FALSE
#define STM32_HSE_ENABLED                   FALSE
#define STM32_LSI_ENABLED                   FALSE
#define STM32_LSE_ENABLED                   TRUE
#define STM32_MSI_ENABLED                   TRUE
#define STM32_MSIPLL_ENABLED                TRUE
#define STM32_MSIRANGE                      RCC_CR_MSIRANGE_4M
#define STM32_MSISRANGE                     RCC_CSR_MSISRANGE_4M
#define STM32_SW                            RCC_CFGR_SW_PLLR
#define STM32_PLLSRC                        RCC_PLLCFGR_PLLSRC_MSI
#define STM32_PLLM_VALUE                    1
#define STM32_PLLN_VALUE                    28
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    2
#define STM32_PLLR_VALUE                    2
#define STM32_HPRE                          RCC_CFGR_HPRE_DIV1
#define STM32_PPRE                          RCC_CFGR_PPRE_DIV1
#define STM32_MCO1SEL                       RCC_CFGR_MCO1SEL_NOCLOCK
#define STM32_MCO1PRE                       RCC_CFGR_MCO1PRE_DIV1
#define STM32_MCO2SEL                       RCC_CFGR_MCO2SEL_NOCLOCK
#define STM32_MCO2PRE                       RCC_CFGR_MCO2PRE_DIV1
#define STM32_LSCOSEL                       RCC_BDCR_LSCOSEL_NOCLOCK

/*
 * Peripherals clock sources.
 */
#define STM32_USART1SEL                     RCC_CCIPR_USART1SEL_PCLK
#define STM32_USART2SEL                     RCC_CCIPR_USART2SEL_PCLK
#define STM32_LPUART1SEL                    RCC_CCIPR_LPUART1SEL_PCLK
#define STM32_LPUART2SEL                    RCC_CCIPR_LPUART2SEL_PCLK
#define STM32_LPUART3SEL                    RCC_CCIPR_LPUART3SEL_PCLK
#define STM32_I2C1SEL                       RCC_CCIPR_I2C1SEL_PCLK
#define STM32_I2C3SEL                       RCC_CCIPR_I2C3SEL_PCLK
#define STM32_LPTIM1SEL                     RCC_CCIPR_LPTIM1SEL_PCLK
#define STM32_LPTIM2SEL                     RCC_CCIPR_LPTIM2SEL_PCLK
#define STM32_LPTIM3SEL                     RCC_CCIPR_LPTIM3SEL_PCLK
#define STM32_TIM1SEL                       RCC_CCIPR_TIM1SEL_TIMPCLK
#define STM32_TIM15SEL                      RCC_CCIPR_TIM15SEL_TIMPCLK
#define STM32_CLK48SEL                      RCC_CCIPR_CLK48SEL_PLLQCLK
#define STM32_ADCSEL                        RCC_CCIPR_ADCSEL_SYSCLK
#define STM32_RTCSEL                        RCC_BDCR_RTCSEL_NOCLOCK

/*
 * IRQ system settings.
 */
#define STM32_IRQ_DAC1_PRIORITY             2

#define STM32_IRQ_EXTI0_1_PRIORITY          3
#define STM32_IRQ_EXTI2_3_PRIORITY          3
#define STM32_IRQ_EXTI4_15_PRIORITY         3

#define STM32_IRQ_I2C1_PRIORITY             3
#define STM32_IRQ_I2C2_3_4_PRIORITY         3

#define STM32_IRQ_SPI1_PRIORITY             2
#define STM32_IRQ_SPI2_PRIORITY             2
#define STM32_IRQ_SPI3_PRIORITY             2

#define STM32_IRQ_USART1_PRIORITY           2
#define STM32_IRQ_USART2_LP2_PRIORITY       2
#define STM32_IRQ_USART3_LP1_PRIORITY       2
#define STM32_IRQ_USART4_LP3_PRIORITY       2

#define STM32_IRQ_TIM1_UP_PRIORITY          1
#define STM32_IRQ_TIM1_CC_PRIORITY          1
#define STM32_IRQ_TIM2_PRIORITY             1
#define STM32_IRQ_TIM3_PRIORITY             1
#define STM32_IRQ_TIM6_PRIORITY             1
#define STM32_IRQ_TIM7_PRIORITY             1
#define STM32_IRQ_TIM15_PRIORITY            1
#define STM32_IRQ_TIM16_PRIORITY            1

#define STM32_IRQ_USB1_PRIORITY             3

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  FALSE
#define STM32_ADC_COMPACT_SAMPLES           FALSE
#define STM32_ADC_ADC1_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     5
#define STM32_ADC_ADC1_IRQ_PRIORITY         5

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 FALSE
#define STM32_DAC_USE_DAC1_CH1              FALSE
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH1_DMA3_CHANNEL     STM32_DMA3_MASK_FIFO2

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  FALSE
#define STM32_GPT_USE_TIM6                  FALSE
#define STM32_GPT_USE_TIM7                  FALSE
#define STM32_GPT_USE_TIM15                 FALSE
#define STM32_GPT_USE_TIM16                 FALSE

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  FALSE
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_USE_I2C3                  FALSE
#define STM32_I2C_USE_I2C4                  FALSE
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_DMA_CHANNEL          STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C2_DMA_CHANNEL          STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C3_DMA_CHANNEL          STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C4_DMA_CHANNEL          STM32_DMA_STREAM_ID_ANY
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_I2C3_DMA_PRIORITY         3
#define STM32_I2C_I2C4_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM15                 FALSE

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  FALSE
#define STM32_PWM_USE_TIM2                  FALSE
#define STM32_PWM_USE_TIM3                  FALSE
#define STM32_PWM_USE_TIM15                 FALSE
#define STM32_PWM_USE_TIM16                 FALSE

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               32
#define STM32_RTC_PRESS_VALUE               1024
#define STM32_RTC_CR_INIT                   0

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             FALSE
#define STM32_SERIAL_USE_USART2             FALSE
#define STM32_SERIAL_USE_USART3             FALSE
#define STM32_SERIAL_USE_UART4              FALSE
#define STM32_SERIAL_USE_LPUART1            TRUE
#define STM32_SERIAL_USE_LPUART2            FALSE
#define STM32_SERIAL_USE_LPUART3            FALSE

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                FALSE
#define STM32_SIO_USE_USART2                FALSE
#define STM32_SIO_USE_USART3                FALSE
#define STM32_SIO_USE_UART4                 FALSE
#define STM32_SIO_USE_LPUART1               FALSE
#define STM32_SIO_USE_LPUART2               FALSE
#define STM32_SIO_USE_LPUART3               FALSE

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  FALSE
#define STM32_SPI_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI2_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI2_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI3_RX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI3_TX_DMA_STREAM        STM32_DMA_STREAM_ID_ANY
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               2
#define STM32_ST_USE_TIMER                  2

/*
 * TRNG driver system settings.
 */
#define STM32_TRNG_USE_RNG1                 FALSE

/*
 * UART driver system settings.
 */

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_USB1                  FALSE
#define STM32_USB_USE_ISOCHRONOUS           FALSE
#define STM32_USB_USE_FAST_COPY             FALSE
#define STM32_USB_HOST_WAKEUP_DURATION      2
#define STM32_USB_48MHZ_DELTA               120000

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

/*
 * WSPI driver system settings.
 */

#endif /* MCUCONF_H */
