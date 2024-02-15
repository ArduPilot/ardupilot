/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
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

#pragma once
/*
 * STM32F303 drivers configuration.
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

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_LSE_ENABLED                   FALSE

#if STM32_HSECLK == 0U
// use HSI, disable HSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   FALSE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSI
#define STM32_PLLXTPRE                      STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE                  16
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_ADCPRE                        STM32_ADCPRE_DIV4
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
// can't do more than 64MHz with HSI
#undef HAL_EXPECTED_SYSCLOCK
#define HAL_EXPECTED_SYSCLOCK               64000000

#elif STM32_HSECLK == 8000000U
#define STM32_HSI_ENABLED                   FALSE
#define STM32_LSI_ENABLED                   FALSE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLXTPRE                      STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE                  9
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_ADCPRE                        STM32_ADCPRE_DIV4
#define STM32_HPRE                          STM32_HPRE_DIV1

#elif STM32_HSECLK == 24000000U
#define STM32_HSI_ENABLED                   FALSE
#define STM32_LSI_ENABLED                   FALSE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLXTPRE                      STM32_PLLXTPRE_DIV1
#define STM32_PLLMUL_VALUE                  3
#define STM32_PPRE1                         STM32_PPRE1_DIV2
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_ADCPRE                        STM32_ADCPRE_DIV4
#define STM32_HPRE                          STM32_HPRE_DIV1
#else
#error "Unsupported STM32F1xx clock frequency"
#endif

#define STM32_MCOSEL                        STM32_MCOSEL_NOCLOCK
#ifndef STM32_RTCSEL
#define STM32_RTCSEL                        STM32_RTCSEL_HSEDIV
#endif
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0

/*
 * ADC driver system settings.
 */
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_ADC1_IRQ_PRIORITY         6

/*
 * EXT driver system settings.
 */
#define STM32_EXT_EXTI0_IRQ_PRIORITY        6
#define STM32_EXT_EXTI1_IRQ_PRIORITY        6
#define STM32_EXT_EXTI2_IRQ_PRIORITY        6
#define STM32_EXT_EXTI3_IRQ_PRIORITY        6
#define STM32_EXT_EXTI4_IRQ_PRIORITY        6
#define STM32_EXT_EXTI5_9_IRQ_PRIORITY      6
#define STM32_EXT_EXTI10_15_IRQ_PRIORITY    6
#define STM32_EXT_EXTI16_IRQ_PRIORITY       6
#define STM32_EXT_EXTI17_IRQ_PRIORITY       6
#define STM32_EXT_EXTI18_IRQ_PRIORITY       6
#define STM32_EXT_EXTI19_IRQ_PRIORITY       6

/*
 * GPT driver system settings.
 */
#define STM32_GPT_TIM1_IRQ_PRIORITY         7
#define STM32_GPT_TIM2_IRQ_PRIORITY         7
#define STM32_GPT_TIM3_IRQ_PRIORITY         7
#define STM32_GPT_TIM4_IRQ_PRIORITY         7
#define STM32_GPT_TIM5_IRQ_PRIORITY         7
#define STM32_GPT_TIM8_IRQ_PRIORITY         7

/*
 * I2C driver system settings.
 */
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_IRQ_PRIORITY         5
#define STM32_I2C_I2C2_IRQ_PRIORITY         5
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      STM32_DMA_ERROR_HOOK(i2cp)

/*
 * ICU driver system settings.
 */
#define STM32_ICU_TIM1_IRQ_PRIORITY         7
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#define STM32_ICU_TIM5_IRQ_PRIORITY         7
#define STM32_ICU_TIM8_IRQ_PRIORITY         7

/*
 * PWM driver system settings.
 */
#define STM32_PWM_TIM1_IRQ_PRIORITY         7
#define STM32_PWM_TIM2_IRQ_PRIORITY         7
#define STM32_PWM_TIM3_IRQ_PRIORITY         7
#define STM32_PWM_TIM4_IRQ_PRIORITY         7
#define STM32_PWM_TIM5_IRQ_PRIORITY         7
#define STM32_PWM_TIM8_IRQ_PRIORITY         7

/*
 * RTC driver system settings.
 */
#define STM32_RTC_IRQ_PRIORITY              15

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USART1_PRIORITY        12
#define STM32_SERIAL_USART2_PRIORITY        12
#define STM32_SERIAL_USART3_PRIORITY        12

/*
 * SPI driver system settings.
 */
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      STM32_DMA_ERROR_HOOK(spip)

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#ifndef STM32_ST_USE_TIMER
#define STM32_ST_USE_TIMER                  2
#endif
/*
 * UART driver system settings.
 */
#define STM32_UART_USART1_IRQ_PRIORITY      12
#define STM32_UART_USART2_IRQ_PRIORITY      12
#define STM32_UART_USART3_IRQ_PRIORITY      12
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    STM32_DMA_ERROR_HOOK(uartp)

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

#ifndef STM32_ADC_USE_ADC1
#define STM32_ADC_USE_ADC1                  FALSE
#endif
