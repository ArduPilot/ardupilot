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
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */
/*
  this provides the default mcuconf.h for each board. Override values in hwdef.dat
 */
#pragma once


/*
 * STM32F4xx drivers configuration.
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

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE

#ifndef STM32_HSI_ENABLED
#define STM32_HSI_ENABLED                   TRUE
#endif

#ifndef STM32_LSI_ENABLED
#define STM32_LSI_ENABLED                   TRUE
#endif

#ifndef STM32_HSE_ENABLED
#define STM32_HSE_ENABLED                   TRUE
#endif

#ifndef STM32_LSE_ENABLED
#define STM32_LSE_ENABLED                   FALSE
#endif

#ifndef STM32_CLOCK48_REQUIRED
#define STM32_CLOCK48_REQUIRED              TRUE
#endif

#ifndef STM32_SW
#define STM32_SW                            STM32_SW_PLL
#endif

#ifndef STM32_PLLSRC
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#endif

#if !defined(HAL_CUSTOM_CLOCK_TREE)
#if defined(STM32F7xx_MCUCONF)
// F7 clock config
#if STM32_HSECLK == 0U
#undef STM32_HSE_ENABLED
#undef STM32_HSI_ENABLED
#undef STM32_PLLSRC
#define STM32_HSE_ENABLED                   FALSE
#define STM32_HSI_ENABLED                   TRUE
#define STM32_PLLSRC                        STM32_PLLSRC_HSI
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    216
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    9
#elif STM32_HSECLK == 8000000U
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    432
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    9
#elif STM32_HSECLK == 16000000U
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    216
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    9
#elif STM32_HSECLK == 24000000U
#define STM32_PLLM_VALUE                    24
#define STM32_PLLN_VALUE                    432
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    9
#else
#error "Unsupported F7 HSE clock"
#endif
#else // F4
// F4 clock config
#if STM32_HSECLK == 8000000U
#define STM32_PLLM_VALUE                    8
#define STM32_PLLN_VALUE                    336
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    7
#elif STM32_HSECLK == 16000000U
#define STM32_PLLM_VALUE                    16
#define STM32_PLLN_VALUE                    384
#define STM32_PLLP_VALUE                    4
#define STM32_PLLQ_VALUE                    8
#elif STM32_HSECLK == 24000000U
#define STM32_PLLM_VALUE                    24
#define STM32_PLLN_VALUE                    336
#define STM32_PLLP_VALUE                    2
#define STM32_PLLQ_VALUE                    7
#else
#error "Unsupported F4 HSE clock"
#endif
#endif // MCU
#endif // HAL_CUSTOM_CLOCK_TREE

// we don't use LSE, but we need the defines
#define STM32_LSECLK   32768U
#define STM32_LSEDRV   (3U << 3U)

#define STM32_VDD 330U

#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV4
#define STM32_PPRE2                         STM32_PPRE2_DIV2
#define STM32_RTCSEL                        STM32_RTCSEL_LSI
#define STM32_RTCPRE_VALUE                  8
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI
#define STM32_MCO1PRE                       STM32_MCO1PRE_DIV1
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYSCLK
#define STM32_MCO2PRE                       STM32_MCO2PRE_DIV5
#define STM32_I2SSRC                        STM32_I2SSRC_CKIN
#define STM32_PLLI2SN_VALUE                 192
#define STM32_PLLI2SR_VALUE                 5
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0
#define STM32_BKPRAM_ENABLE                 FALSE

/*
 * ADC driver system settings.
 */
#define STM32_ADC_ADCPRE                    ADC_CCR_ADCPRE_DIV4
#ifndef STM32_ADC_USE_ADC1
#define STM32_ADC_USE_ADC1                  TRUE
#endif
#ifndef STM32_ADC_USE_ADC2
#define STM32_ADC_USE_ADC2                  FALSE
#endif
#ifndef STM32_ADC_USE_ADC3
#define STM32_ADC_USE_ADC3                  FALSE
#endif
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_ADC2_DMA_PRIORITY         2
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#define STM32_ADC_IRQ_PRIORITY              6
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     6
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     6
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     6

/*
 * CAN driver system settings.
 */
#ifndef STM32_CAN_USE_CAN1
#define STM32_CAN_USE_CAN1                  FALSE
#endif
#ifndef STM32_CAN_USE_CAN2
#define STM32_CAN_USE_CAN2                  FALSE
#endif
#define STM32_CAN_CAN1_IRQ_PRIORITY         11
#define STM32_CAN_CAN2_IRQ_PRIORITY         11

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 FALSE
#define STM32_DAC_USE_DAC1_CH1              FALSE
#define STM32_DAC_USE_DAC1_CH2              FALSE
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     2

/*
 * EXT driver system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY        6
#define STM32_IRQ_EXTI1_PRIORITY        6
#define STM32_IRQ_EXTI2_PRIORITY        6
#define STM32_IRQ_EXTI3_PRIORITY        6
#define STM32_IRQ_EXTI4_PRIORITY        6
#define STM32_IRQ_EXTI5_9_PRIORITY      6
#define STM32_IRQ_EXTI10_15_PRIORITY    6
#define STM32_IRQ_EXTI16_PRIORITY       6
#define STM32_IRQ_EXTI17_PRIORITY       15
#define STM32_IRQ_EXTI18_PRIORITY       6
#define STM32_IRQ_EXTI19_PRIORITY       6
#define STM32_IRQ_EXTI20_PRIORITY       6
#define STM32_IRQ_EXTI21_PRIORITY       15
#define STM32_IRQ_EXTI22_PRIORITY       15
#define STM32_IRQ_EXTI23_PRIORITY       15

/*
 * GPT driver system settings.
 */
#ifndef STM32_GPT_USE_TIM1
#define STM32_GPT_USE_TIM1                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM2
#define STM32_GPT_USE_TIM2                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM3
#define STM32_GPT_USE_TIM3                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM4
#define STM32_GPT_USE_TIM4                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM5
#define STM32_GPT_USE_TIM5                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM6
#define STM32_GPT_USE_TIM6                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM7
#define STM32_GPT_USE_TIM7                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM8
#define STM32_GPT_USE_TIM8                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM9
#define STM32_GPT_USE_TIM9                  FALSE
#endif
#ifndef STM32_GPT_USE_TIM10
#define STM32_GPT_USE_TIM10                 FALSE
#endif
#ifndef STM32_GPT_USE_TIM11
#define STM32_GPT_USE_TIM11                 FALSE
#endif
#ifndef STM32_GPT_USE_TIM12
#define STM32_GPT_USE_TIM12                 FALSE
#endif
#ifndef STM32_GPT_USE_TIM13
#define STM32_GPT_USE_TIM13                 FALSE
#endif
#ifndef STM32_GPT_USE_TIM14
#define STM32_GPT_USE_TIM14                 FALSE
#endif
#define STM32_GPT_TIM1_IRQ_PRIORITY         7
#define STM32_GPT_TIM2_IRQ_PRIORITY         7
#define STM32_GPT_TIM3_IRQ_PRIORITY         7
#define STM32_GPT_TIM4_IRQ_PRIORITY         7
#define STM32_GPT_TIM5_IRQ_PRIORITY         7
#define STM32_GPT_TIM6_IRQ_PRIORITY         7
#define STM32_GPT_TIM7_IRQ_PRIORITY         7
#define STM32_GPT_TIM8_IRQ_PRIORITY         7
#define STM32_GPT_TIM9_IRQ_PRIORITY         7
#define STM32_GPT_TIM11_IRQ_PRIORITY        7
#define STM32_GPT_TIM12_IRQ_PRIORITY        7
#define STM32_GPT_TIM14_IRQ_PRIORITY        7

/*
 * I2C driver system settings.
 */
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_IRQ_PRIORITY         5
#define STM32_I2C_I2C2_IRQ_PRIORITY         5
#define STM32_I2C_I2C3_IRQ_PRIORITY         5
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_I2C3_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")

/*
 * I2S driver system settings.
 */
#define STM32_I2S_SPI2_IRQ_PRIORITY         10
#define STM32_I2S_SPI3_IRQ_PRIORITY         10
#define STM32_I2S_SPI2_DMA_PRIORITY         1
#define STM32_I2S_SPI3_DMA_PRIORITY         1
#define STM32_I2S_DMA_ERROR_HOOK(i2sp)      osalSysHalt("DMA failure")

/*
 * ICU driver system settings.
 */
#define STM32_ICU_TIM1_IRQ_PRIORITY         7
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#define STM32_ICU_TIM5_IRQ_PRIORITY         7
#define STM32_ICU_TIM8_IRQ_PRIORITY         7
#define STM32_ICU_TIM9_IRQ_PRIORITY         7

/*
 * EICU driver system settings.
 */
#define STM32_EICU_TIM1_IRQ_PRIORITY         6
#define STM32_EICU_TIM2_IRQ_PRIORITY         6
#define STM32_EICU_TIM3_IRQ_PRIORITY         6
#define STM32_EICU_TIM4_IRQ_PRIORITY         6
#define STM32_EICU_TIM5_IRQ_PRIORITY         6
#define STM32_EICU_TIM8_IRQ_PRIORITY         6
#define STM32_EICU_TIM9_IRQ_PRIORITY         6
#define STM32_EICU_TIM10_IRQ_PRIORITY        6
#define STM32_EICU_TIM11_IRQ_PRIORITY        6
#define STM32_EICU_TIM12_IRQ_PRIORITY        6
#define STM32_EICU_TIM13_IRQ_PRIORITY        6
#define STM32_EICU_TIM14_IRQ_PRIORITY        6

/*
 * MAC driver system settings.
 */
#define STM32_MAC_TRANSMIT_BUFFERS          2
#define STM32_MAC_RECEIVE_BUFFERS           4
#define STM32_MAC_BUFFERS_SIZE              1522
#define STM32_MAC_PHY_TIMEOUT               100
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     TRUE
#define STM32_MAC_ETH1_IRQ_PRIORITY         13
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       0

/*
 * PWM driver system settings.
 */
#ifndef STM32_PWM_USE_ADVANCED
#define STM32_PWM_USE_ADVANCED              FALSE
#endif
#define STM32_PWM_TIM1_IRQ_PRIORITY         7
#define STM32_PWM_TIM2_IRQ_PRIORITY         7
#define STM32_PWM_TIM3_IRQ_PRIORITY         7
#define STM32_PWM_TIM4_IRQ_PRIORITY         7
#define STM32_PWM_TIM5_IRQ_PRIORITY         7
#define STM32_PWM_TIM8_IRQ_PRIORITY         7
#define STM32_PWM_TIM9_IRQ_PRIORITY         7

/*
 * SDC driver system settings.
 */
#define STM32_SDC_SDIO_DMA_PRIORITY         3
#define STM32_SDC_SDIO_IRQ_PRIORITY         9
#define STM32_SDC_WRITE_TIMEOUT_MS          1000
#define STM32_SDC_READ_TIMEOUT_MS           1000
#define STM32_SDC_CLOCK_ACTIVATION_DELAY    10
#define STM32_SDC_SDIO_UNALIGNED_SUPPORT    TRUE

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USART1_PRIORITY        11
#define STM32_SERIAL_USART2_PRIORITY        11
#define STM32_SERIAL_USART3_PRIORITY        11
#define STM32_SERIAL_UART4_PRIORITY         11
#define STM32_SERIAL_UART5_PRIORITY         11
#define STM32_SERIAL_USART6_PRIORITY        11
#define STM32_SERIAL_UART7_PRIORITY         11
#define STM32_SERIAL_UART8_PRIORITY         11

/*
 * SPI driver system settings.
 */
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_SPI4_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#ifndef STM32_ST_USE_TIMER
#define STM32_ST_USE_TIMER                  2
#endif

#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_UP_TIM10_PRIORITY    7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   7
#define STM32_IRQ_TIM1_BRK_TIM9_PRIORITY    7
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    7
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 7
#define STM32_IRQ_TIM1_TRGCO_TIM11_PRIORITY 7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM15_PRIORITY            7
#define STM32_IRQ_TIM16_PRIORITY            7
#define STM32_IRQ_TIM17_PRIORITY            7

/*
 * UART driver system settings.
 */
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_UART4_DMA_PRIORITY       0
#define STM32_UART_UART5_DMA_PRIORITY       0
#define STM32_UART_USART6_DMA_PRIORITY      0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    osalSysHalt("DMA failure")

#define STM32_IRQ_UART1_PRIORITY            12
#define STM32_IRQ_UART2_PRIORITY            12
#define STM32_IRQ_UART3_PRIORITY            12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_UART6_PRIORITY            12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12
#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_USART4_PRIORITY           12
#define STM32_IRQ_USART5_PRIORITY           12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_USART7_PRIORITY           12
#define STM32_IRQ_USART8_PRIORITY           12

/*
 * USB driver system settings.
 */
#ifndef STM32_USB_OTG1_IRQ_PRIORITY
#define STM32_USB_OTG1_IRQ_PRIORITY         14
#endif
#ifndef STM32_USB_OTG2_IRQ_PRIORITY
#define STM32_USB_OTG2_IRQ_PRIORITY         14
#endif
#ifndef STM32_USB_OTG1_RX_FIFO_SIZE
#define STM32_USB_OTG1_RX_FIFO_SIZE         512
#endif
#ifndef STM32_USB_OTG2_RX_FIFO_SIZE
#define STM32_USB_OTG2_RX_FIFO_SIZE         1024
#endif
#ifndef STM32_USB_OTG_THREAD_PRIO
#define STM32_USB_OTG_THREAD_PRIO           LOWPRIO
#endif
#ifndef STM32_USB_OTG_THREAD_STACK_SIZE
#define STM32_USB_OTG_THREAD_STACK_SIZE     128
#endif
#ifndef STM32_USB_OTGFIFO_FILL_BASEPRI
#define STM32_USB_OTGFIFO_FILL_BASEPRI      0
#endif

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

// limit ISR count per byte
#define STM32_I2C_ISR_LIMIT                 6


