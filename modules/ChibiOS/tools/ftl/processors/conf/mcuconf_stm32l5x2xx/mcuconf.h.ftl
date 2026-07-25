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
#define STM32_SECURE_MODE                   ${doc.STM32_SECURE_MODE!"TRUE"}

/*
 * HAL driver global settings.
 */
#define STM32_NO_INIT                       ${doc.STM32_NO_INIT!"FALSE"}

/*
 * ICache settings.
 */
#define STM32_ICACHE_CR                     ${doc.STM32_ICACHE_CR!"(ICACHE_CR_WAYSEL | ICACHE_CR_EN)"}
#define STM32_ICACHE_CRR0                   ${doc.STM32_ICACHE_CRR0!"0"}
#define STM32_ICACHE_CRR1                   ${doc.STM32_ICACHE_CRR1!"0"}
#define STM32_ICACHE_CRR2                   ${doc.STM32_ICACHE_CRR2!"0"}
#define STM32_ICACHE_CRR3                   ${doc.STM32_ICACHE_CRR3!"0"}

/*
 * Power settings.
 */
#define STM32_VOS                           ${doc.STM32_VOS!"STM32_VOS_RANGE0"}
#define STM32_PWR_CR2                       ${doc.STM32_PWR_CR2!"(PWR_CR2_IOSV | PWR_CR2_PLS_LEV0)"}
#define STM32_PWR_CR3                       ${doc.STM32_PWR_CR3!"(0U)"}
#define STM32_PWR_CR4                       ${doc.STM32_PWR_CR4!"(0U)"}

/*
 * Clock settings.
 */
#define STM32_RCC_SECCFGR                   ${doc.STM32_RCC_SECCFGR!"(RCC_SECCFGR_MSISEC | RCC_SECCFGR_PLLSEC | RCC_SECCFGR_SYSCLKSEC | RCC_SECCFGR_PRESCSEC)"}
#define STM32_HSI16_ENABLED                 ${doc.STM32_HSI16_ENABLED!"TRUE"}
#define STM32_HSI48_ENABLED                 ${doc.STM32_HSI48_ENABLED!"TRUE"}
#define STM32_LSI_ENABLED                   ${doc.STM32_LSI_ENABLED!"TRUE"}
#define STM32_HSE_ENABLED                   ${doc.STM32_HSE_ENABLED!"FALSE"}
#define STM32_LSE_ENABLED                   ${doc.STM32_LSE_ENABLED!"TRUE"}
#define STM32_MSIPLL_ENABLED                ${doc.STM32_MSIPLL_ENABLED!"TRUE"}
#define STM32_MSIRANGE                      ${doc.STM32_MSIRANGE!"STM32_MSIRANGE_4M"}
#define STM32_MSISRANGE                     ${doc.STM32_MSISRANGE!"STM32_MSISRANGE_4M"}
#define STM32_SW                            ${doc.STM32_SW!"STM32_SW_PLL"}
#define STM32_PLLSRC                        ${doc.STM32_PLLSRC!"STM32_PLLSRC_MSI"}
#define STM32_PLLM_VALUE                    ${doc.STM32_PLLM_VALUE!"1"}
#define STM32_PLLN_VALUE                    ${doc.STM32_PLLN_VALUE!"55"}
#define STM32_PLLPDIV_VALUE                 ${doc.STM32_PLLPDIV_VALUE!"0"}
#define STM32_PLLP_VALUE                    ${doc.STM32_PLLP_VALUE!"7"}
#define STM32_PLLQ_VALUE                    ${doc.STM32_PLLQ_VALUE!"4"}
#define STM32_PLLR_VALUE                    ${doc.STM32_PLLR_VALUE!"2"}
#define STM32_PLLSAI1SRC                    ${doc.STM32_PLLSAI1SRC!"STM32_PLLSAI1SRC_MSI"}
#define STM32_PLLSAI1M_VALUE                ${doc.STM32_PLLSAI1M_VALUE!"1"}
#define STM32_PLLSAI1N_VALUE                ${doc.STM32_PLLSAI1N_VALUE!"72"}
#define STM32_PLLSAI1PDIV_VALUE             ${doc.STM32_PLLSAI1PDIV_VALUE!"6"}
#define STM32_PLLSAI1P_VALUE                ${doc.STM32_PLLSAI1P_VALUE!"7"}
#define STM32_PLLSAI1Q_VALUE                ${doc.STM32_PLLSAI1Q_VALUE!"6"}
#define STM32_PLLSAI1R_VALUE                ${doc.STM32_PLLSAI1R_VALUE!"6"}
#define STM32_PLLSAI2SRC                    ${doc.STM32_PLLSAI2SRC!"STM32_PLLSAI2SRC_MSI"}
#define STM32_PLLSAI2M_VALUE                ${doc.STM32_PLLSAI2M_VALUE!"1"}
#define STM32_PLLSAI2N_VALUE                ${doc.STM32_PLLSAI2N_VALUE!"72"}
#define STM32_PLLSAI2PDIV_VALUE             ${doc.STM32_PLLSAI2PDIV_VALUE!"6"}
#define STM32_PLLSAI2P_VALUE                ${doc.STM32_PLLSAI2P_VALUE!"7"}
#define STM32_HPRE                          ${doc.STM32_HPRE!"STM32_HPRE_DIV1"}
#define STM32_PPRE1                         ${doc.STM32_PPRE1!"STM32_PPRE1_DIV1"}
#define STM32_PPRE2                         ${doc.STM32_PPRE2!"STM32_PPRE2_DIV1"}
#define STM32_STOPWUCK                      ${doc.STM32_STOPWUCK!"STM32_STOPWUCK_MSI"}
#define STM32_MCOSEL                        ${doc.STM32_MCOSEL!"STM32_MCOSEL_NOCLOCK"}
#define STM32_MCOPRE                        ${doc.STM32_MCOPRE!"STM32_MCOPRE_DIV1"}
#define STM32_LSCOSEL                       ${doc.STM32_LSCOSEL!"STM32_LSCOSEL_NOCLOCK"}

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
#define STM32_LPTIM3SEL                     ${doc.STM32_LPTIM3SEL!"STM32_LPTIM3SEL_PCLK1"}
#define STM32_FDCANSEL                      ${doc.STM32_FDCANSEL!"STM32_FDCANSEL_PLL"}
#define STM32_CLK48SEL                      ${doc.STM32_CLK48SEL!"STM32_CLK48SEL_PLL"}
#define STM32_ADCSEL                        ${doc.STM32_ADCSEL!"STM32_ADCSEL_SYSCLK"}
#define STM32_DFSDMSEL                      ${doc.STM32_DFSDMSEL!"STM32_DFSDMSEL_PCLK2"}
#define STM32_ADFSDMSEL                     ${doc.STM32_ADFSDMSEL!"STM32_ADFSDMSEL_SAI1CLK"}
#define STM32_SAI1SEL                       ${doc.STM32_SAI1SEL!"STM32_SAI1SEL_OFF"}
#define STM32_SAI2SEL                       ${doc.STM32_SAI2SEL!"STM32_SAI2SEL_OFF"}
#define STM32_SDMMCSEL                      ${doc.STM32_SDMMCSEL!"STM32_SDMMCSEL_48CLK"}
#define STM32_OSPISEL                       ${doc.STM32_OSPISEL!"STM32_OSPISEL_SYSCLK"}
#define STM32_RTCSEL                        ${doc.STM32_RTCSEL!"STM32_RTCSEL_LSI"}

/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            ${doc.STM32_IRQ_EXTI0_PRIORITY!"2"}
#define STM32_IRQ_EXTI1_PRIORITY            ${doc.STM32_IRQ_EXTI1_PRIORITY!"2"}
#define STM32_IRQ_EXTI2_PRIORITY            ${doc.STM32_IRQ_EXTI2_PRIORITY!"2"}
#define STM32_IRQ_EXTI3_PRIORITY            ${doc.STM32_IRQ_EXTI3_PRIORITY!"2"}
#define STM32_IRQ_EXTI4_PRIORITY            ${doc.STM32_IRQ_EXTI4_PRIORITY!"2"}
#define STM32_IRQ_EXTI5_PRIORITY            ${doc.STM32_IRQ_EXTI5_PRIORITY!"2"}
#define STM32_IRQ_EXTI6_PRIORITY            ${doc.STM32_IRQ_EXTI6_PRIORITY!"2"}
#define STM32_IRQ_EXTI7_PRIORITY            ${doc.STM32_IRQ_EXTI7_PRIORITY!"2"}
#define STM32_IRQ_EXTI8_PRIORITY            ${doc.STM32_IRQ_EXTI8_PRIORITY!"2"}
#define STM32_IRQ_EXTI9_PRIORITY            ${doc.STM32_IRQ_EXTI9_PRIORITY!"2"}
#define STM32_IRQ_EXTI10_PRIORITY           ${doc.STM32_IRQ_EXTI10_PRIORITY!"2"}
#define STM32_IRQ_EXTI11_PRIORITY           ${doc.STM32_IRQ_EXTI11_PRIORITY!"2"}
#define STM32_IRQ_EXTI12_PRIORITY           ${doc.STM32_IRQ_EXTI12_PRIORITY!"2"}
#define STM32_IRQ_EXTI13_PRIORITY           ${doc.STM32_IRQ_EXTI13_PRIORITY!"2"}
#define STM32_IRQ_EXTI14_PRIORITY           ${doc.STM32_IRQ_EXTI14_PRIORITY!"2"}
#define STM32_IRQ_EXTI15_PRIORITY           ${doc.STM32_IRQ_EXTI15_PRIORITY!"2"}
#define STM32_IRQ_EXTI1635_38_PRIORITY      ${doc.STM32_IRQ_EXTI1635_38_PRIORITY!"2"}
#define STM32_IRQ_EXTI17_PRIORITY           ${doc.STM32_IRQ_EXTI17_PRIORITY!"2"}
#define STM32_IRQ_EXTI18_PRIORITY           ${doc.STM32_IRQ_EXTI18_PRIORITY!"2"}
#define STM32_IRQ_EXTI19_PRIORITY           ${doc.STM32_IRQ_EXTI19_PRIORITY!"2"}
#define STM32_IRQ_EXTI20_PRIORITY           ${doc.STM32_IRQ_EXTI20_PRIORITY!"2"}
#define STM32_IRQ_EXTI21_22_PRIORITY        ${doc.STM32_IRQ_EXTI21_22_PRIORITY!"2"}

#define STM32_IRQ_FDCAN1_PRIORITY           ${doc.STM32_IRQ_FDCAN1_PRIORITY!"1"}

#define STM32_IRQ_TIM1_BRK_PRIORITY         ${doc.STM32_IRQ_TIM1_BRK_PRIORITY!"1"}
#define STM32_IRQ_TIM1_UP_PRIORITY          ${doc.STM32_IRQ_TIM1_UP_PRIORITY!"1"}
#define STM32_IRQ_TIM1_TRGCO_PRIORITY       ${doc.STM32_IRQ_TIM1_TRGCO_PRIORITY!"1"}
#define STM32_IRQ_TIM1_CC_PRIORITY          ${doc.STM32_IRQ_TIM1_CC_PRIORITY!"1"}
#define STM32_IRQ_TIM2_PRIORITY             ${doc.STM32_IRQ_TIM2_PRIORITY!"1"}
#define STM32_IRQ_TIM3_PRIORITY             ${doc.STM32_IRQ_TIM3_PRIORITY!"1"}
#define STM32_IRQ_TIM4_PRIORITY             ${doc.STM32_IRQ_TIM4_PRIORITY!"1"}
#define STM32_IRQ_TIM5_PRIORITY             ${doc.STM32_IRQ_TIM5_PRIORITY!"1"}
#define STM32_IRQ_TIM6_PRIORITY             ${doc.STM32_IRQ_TIM6_PRIORITY!"1"}
#define STM32_IRQ_TIM7_PRIORITY             ${doc.STM32_IRQ_TIM7_PRIORITY!"1"}
#define STM32_IRQ_TIM8_UP_PRIORITY          ${doc.STM32_IRQ_TIM8_UP_PRIORITY!"1"}
#define STM32_IRQ_TIM8_CC_PRIORITY          ${doc.STM32_IRQ_TIM8_CC_PRIORITY!"1"}
#define STM32_IRQ_TIM15_PRIORITY            ${doc.STM32_IRQ_TIM15_PRIORITY!"1"}
#define STM32_IRQ_TIM16_PRIORITY            ${doc.STM32_IRQ_TIM16_PRIORITY!"1"}
#define STM32_IRQ_TIM17_PRIORITY            ${doc.STM32_IRQ_TIM17_PRIORITY!"1"}
#define STM32_IRQ_TIM20_UP_PRIORITY         ${doc.STM32_IRQ_TIM20_UP_PRIORITY!"1"}
#define STM32_IRQ_TIM20_CC_PRIORITY         ${doc.STM32_IRQ_TIM20_CC_PRIORITY!"1"}

#define STM32_IRQ_USART1_PRIORITY           ${doc.STM32_IRQ_USART1_PRIORITY!"1"}
#define STM32_IRQ_USART2_PRIORITY           ${doc.STM32_IRQ_USART2_PRIORITY!"1"}
#define STM32_IRQ_USART3_PRIORITY           ${doc.STM32_IRQ_USART3_PRIORITY!"1"}
#define STM32_IRQ_UART4_PRIORITY            ${doc.STM32_IRQ_UART4_PRIORITY!"1"}
#define STM32_IRQ_UART5_PRIORITY            ${doc.STM32_IRQ_UART5_PRIORITY!"1"}
#define STM32_IRQ_LPUART1_PRIORITY          ${doc.STM32_IRQ_LPUART1_PRIORITY!"1"}

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
#define STM32_PWM_USE_TIM20                 ${doc.STM32_PWM_USE_TIM20!"FALSE"}

/*
 * RTC driver system settings.
 */

/*
 * SDC driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             ${doc.STM32_SERIAL_USE_USART1!"FALSE"}
#define STM32_SERIAL_USE_USART2             ${doc.STM32_SERIAL_USE_USART2!"FALSE"}
#define STM32_SERIAL_USE_USART3             ${doc.STM32_SERIAL_USE_USART3!"FALSE"}
#define STM32_SERIAL_USE_UART4              ${doc.STM32_SERIAL_USE_UART4!"FALSE"}
#define STM32_SERIAL_USE_UART5              ${doc.STM32_SERIAL_USE_UART5!"FALSE"}
#define STM32_SERIAL_USE_LPUART1            ${doc.STM32_SERIAL_USE_LPUART1!"TRUE"}

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

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               ${doc.STM32_ST_IRQ_PRIORITY!"1"}
#define STM32_ST_USE_TIMER                  ${doc.STM32_ST_USE_TIMER!"2"}

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
#define STM32_WDG_USE_IWDG                  ${doc.STM32_WDG_USE_IWDG!"FALSE"}

/*
 * WSPI driver system settings.
 */

#endif /* MCUCONF_H */

