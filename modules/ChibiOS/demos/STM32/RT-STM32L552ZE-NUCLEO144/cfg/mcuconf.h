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
 * STM32L5xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 7...0        Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#ifndef MCUCONF_H
#define MCUCONF_H

#define STM32L5xx_MCUCONF
#define STM32L552_MCUCONF
#define STM32L562_MCUCONF

/*
 * Secure mode HAL settings.
 */
#define STM32_SECURE_MODE                   FALSE

/*
 * HAL driver global settings.
 */
#define STM32_NO_INIT                       FALSE

/*
 * ICache settings.
 */
#define STM32_ICACHE_CR                     (ICACHE_CR_WAYSEL | ICACHE_CR_EN)
#define STM32_ICACHE_CRR0                   0
#define STM32_ICACHE_CRR1                   0
#define STM32_ICACHE_CRR2                   0
#define STM32_ICACHE_CRR3                   0

/*
 * Power settings.
 */
#define STM32_VOS                           STM32_VOS_RANGE0
#define STM32_PWR_CR2                       (PWR_CR2_IOSV | PWR_CR2_PLS_LEV0)
#define STM32_PWR_CR3                       (0U)
#define STM32_PWR_CR4                       (0U)

/*
 * Clock settings.
 */
#define STM32_RCC_SECCFGR                   (RCC_SECCFGR_MSISEC | RCC_SECCFGR_PLLSEC | RCC_SECCFGR_SYSCLKSEC | RCC_SECCFGR_PRESCSEC)
#define STM32_HSI16_ENABLED                 TRUE
#define STM32_HSI48_ENABLED                 TRUE
#define STM32_LSI_ENABLED                   TRUE
#define STM32_HSE_ENABLED                   FALSE
#define STM32_LSE_ENABLED                   TRUE
#define STM32_MSIPLL_ENABLED                TRUE
#define STM32_MSIRANGE                      STM32_MSIRANGE_4M
#define STM32_MSISRANGE                     STM32_MSISRANGE_4M
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_MSI
#define STM32_PLLM_VALUE                    1
#define STM32_PLLN_VALUE                    55
#define STM32_PLLPDIV_VALUE                 0
#define STM32_PLLP_VALUE                    7
#define STM32_PLLQ_VALUE                    4
#define STM32_PLLR_VALUE                    2
#define STM32_PLLSAI1SRC                    STM32_PLLSAI1SRC_MSI
#define STM32_PLLSAI1M_VALUE                1
#define STM32_PLLSAI1N_VALUE                72
#define STM32_PLLSAI1PDIV_VALUE             6
#define STM32_PLLSAI1P_VALUE                7
#define STM32_PLLSAI1Q_VALUE                6
#define STM32_PLLSAI1R_VALUE                6
#define STM32_PLLSAI2SRC                    STM32_PLLSAI2SRC_MSI
#define STM32_PLLSAI2M_VALUE                1
#define STM32_PLLSAI2N_VALUE                72
#define STM32_PLLSAI2PDIV_VALUE             6
#define STM32_PLLSAI2P_VALUE                7
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV1
#define STM32_PPRE2                         STM32_PPRE2_DIV1
#define STM32_STOPWUCK                      STM32_STOPWUCK_MSI
#define STM32_MCOSEL                        STM32_MCOSEL_NOCLOCK
#define STM32_MCOPRE                        STM32_MCOPRE_DIV1
#define STM32_LSCOSEL                       STM32_LSCOSEL_NOCLOCK

/*
 * Peripherals clock sources.
 */
#define STM32_USART1SEL                     STM32_USART1SEL_SYSCLK
#define STM32_USART2SEL                     STM32_USART2SEL_SYSCLK
#define STM32_USART3SEL                     STM32_USART3SEL_SYSCLK
#define STM32_UART4SEL                      STM32_UART4SEL_SYSCLK
#define STM32_UART5SEL                      STM32_UART5SEL_SYSCLK
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_SYSCLK
#define STM32_I2C1SEL                       STM32_I2C1SEL_SYSCLK
#define STM32_I2C2SEL                       STM32_I2C2SEL_SYSCLK
#define STM32_I2C3SEL                       STM32_I2C3SEL_SYSCLK
#define STM32_I2C4SEL                       STM32_I2C4SEL_SYSCLK
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK1
#define STM32_LPTIM3SEL                     STM32_LPTIM3SEL_PCLK1
#define STM32_FDCANSEL                      STM32_FDCANSEL_PLL
#define STM32_CLK48SEL                      STM32_CLK48SEL_PLL
#define STM32_ADCSEL                        STM32_ADCSEL_SYSCLK
#define STM32_DFSDMSEL                      STM32_DFSDMSEL_PCLK2
#define STM32_ADFSDMSEL                     STM32_ADFSDMSEL_SAI1CLK
#define STM32_SAI1SEL                       STM32_SAI1SEL_OFF
#define STM32_SAI2SEL                       STM32_SAI2SEL_OFF
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_48CLK
#define STM32_OSPISEL                       STM32_OSPISEL_SYSCLK
#define STM32_RTCSEL                        STM32_RTCSEL_LSI

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            3
#define STM32_IRQ_EXTI1_PRIORITY            3
#define STM32_IRQ_EXTI2_PRIORITY            3
#define STM32_IRQ_EXTI3_PRIORITY            3
#define STM32_IRQ_EXTI4_PRIORITY            3
#define STM32_IRQ_EXTI5_PRIORITY            3
#define STM32_IRQ_EXTI6_PRIORITY            3
#define STM32_IRQ_EXTI7_PRIORITY            3
#define STM32_IRQ_EXTI8_PRIORITY            3
#define STM32_IRQ_EXTI9_PRIORITY            3
#define STM32_IRQ_EXTI10_PRIORITY           3
#define STM32_IRQ_EXTI11_PRIORITY           3
#define STM32_IRQ_EXTI12_PRIORITY           3
#define STM32_IRQ_EXTI13_PRIORITY           3
#define STM32_IRQ_EXTI14_PRIORITY           3
#define STM32_IRQ_EXTI15_PRIORITY           3
#define STM32_IRQ_EXTI1635_38_PRIORITY      3
#define STM32_IRQ_EXTI17_PRIORITY           3
#define STM32_IRQ_EXTI18_PRIORITY           3
#define STM32_IRQ_EXTI19_PRIORITY           3
#define STM32_IRQ_EXTI20_PRIORITY           3
#define STM32_IRQ_EXTI21_22_PRIORITY        3

#define STM32_IRQ_FDCAN1_PRIORITY           5

#define STM32_IRQ_TIM1_BRK_PRIORITY         4
#define STM32_IRQ_TIM1_UP_PRIORITY          4
#define STM32_IRQ_TIM1_TRGCO_PRIORITY       4
#define STM32_IRQ_TIM1_CC_PRIORITY          4
#define STM32_IRQ_TIM2_PRIORITY             4
#define STM32_IRQ_TIM3_PRIORITY             4
#define STM32_IRQ_TIM4_PRIORITY             4
#define STM32_IRQ_TIM5_PRIORITY             4
#define STM32_IRQ_TIM6_PRIORITY             4
#define STM32_IRQ_TIM7_PRIORITY             4
#define STM32_IRQ_TIM8_UP_PRIORITY          4
#define STM32_IRQ_TIM8_CC_PRIORITY          4
#define STM32_IRQ_TIM15_PRIORITY            4
#define STM32_IRQ_TIM16_PRIORITY            4
#define STM32_IRQ_TIM17_PRIORITY            4
#define STM32_IRQ_TIM20_UP_PRIORITY         4
#define STM32_IRQ_TIM20_CC_PRIORITY         4

#define STM32_IRQ_USART1_PRIORITY           5
#define STM32_IRQ_USART2_PRIORITY           5
#define STM32_IRQ_USART3_PRIORITY           5
#define STM32_IRQ_UART4_PRIORITY            5
#define STM32_IRQ_UART5_PRIORITY            5
#define STM32_IRQ_LPUART1_PRIORITY          5

/*
 * ADC driver system settings.
 */

/*
 * CAN driver system settings.
 */

/*
 * DAC driver system settings.
 */

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  FALSE
#define STM32_GPT_USE_TIM4                  FALSE
#define STM32_GPT_USE_TIM5                  FALSE
#define STM32_GPT_USE_TIM6                  FALSE
#define STM32_GPT_USE_TIM7                  FALSE
#define STM32_GPT_USE_TIM8                  FALSE
#define STM32_GPT_USE_TIM15                 FALSE
#define STM32_GPT_USE_TIM16                 FALSE
#define STM32_GPT_USE_TIM17                 FALSE

/*
 * I2C driver system settings.
 */

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_USE_TIM8                  FALSE
#define STM32_ICU_USE_TIM15                 FALSE
#define STM32_ICU_USE_TIM16                 FALSE
#define STM32_ICU_USE_TIM17                 FALSE

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  FALSE
#define STM32_PWM_USE_TIM2                  FALSE
#define STM32_PWM_USE_TIM3                  FALSE
#define STM32_PWM_USE_TIM4                  FALSE
#define STM32_PWM_USE_TIM5                  FALSE
#define STM32_PWM_USE_TIM8                  FALSE
#define STM32_PWM_USE_TIM15                 FALSE
#define STM32_PWM_USE_TIM16                 FALSE
#define STM32_PWM_USE_TIM17                 FALSE
#define STM32_PWM_USE_TIM20                 FALSE

/*
 * RTC driver system settings.
 */

/*
 * SDC driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             FALSE
#define STM32_SERIAL_USE_USART2             FALSE
#define STM32_SERIAL_USE_USART3             FALSE
#define STM32_SERIAL_USE_UART4              FALSE
#define STM32_SERIAL_USE_UART5              FALSE
#define STM32_SERIAL_USE_LPUART1            TRUE

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                FALSE
#define STM32_SIO_USE_USART2                FALSE
#define STM32_SIO_USE_USART3                FALSE
#define STM32_SIO_USE_UART4                 FALSE
#define STM32_SIO_USE_UART5                 FALSE
#define STM32_SIO_USE_LPUART1               FALSE

/*
 * SPI driver system settings.
 */

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               4
#define STM32_ST_USE_TIMER                  2

/*
 * TRNG driver system settings.
 */

/*
 * UART driver system settings.
 */

/*
 * USB driver system settings.
 */

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

/*
 * WSPI driver system settings.
 */

#endif /* MCUCONF_H */

