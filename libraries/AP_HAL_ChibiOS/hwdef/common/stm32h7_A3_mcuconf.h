/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
  this header is modelled on the one for the Nucleo-144 H743 board from ChibiOS
 */
#pragma once

// this is a newer H7 varient
#define STM32_ENFORCE_H7_REV_XY

// MPU region for ethernet
#define STM32_NOCACHE_MPU_REGION_ETH            MPU_REGION_2

#ifndef STM32_LSECLK
#define STM32_LSECLK 32768U
#endif
#ifndef STM32_LSEDRV
#define STM32_LSEDRV                (3U << 3U)
#endif

/*
 * STM32H7xx drivers configuration.
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

#define STM32H7xx_MCUCONF
#define STM32H7A3_MCUCONF

/*
 * General settings.
 */
#ifndef STM32_NO_INIT
#define STM32_NO_INIT                       FALSE
#endif
#define STM32_TARGET_CORE                   1

/*
 * Memory attributes settings.
 */
//  Disable ChibiOS memory protection which is fixed to SRAM1-3
#define STM32_NOCACHE_ENABLE                FALSE

// enable memory protection on SRAM4, used for bdshot
#define STM32_NOCACHE_MPU_REGION_1          MPU_REGION_5
#define STM32_NOCACHE_MPU_REGION_1_BASE     0x38000000U
#define STM32_NOCACHE_MPU_REGION_1_SIZE     MPU_RASR_SIZE_16K

// enable memory protection on part of AXI used for SDMMC
#define STM32_NOCACHE_MPU_REGION_2          MPU_REGION_4
#define STM32_NOCACHE_MPU_REGION_2_BASE     0x24044000
#define STM32_NOCACHE_MPU_REGION_2_SIZE     MPU_RASR_SIZE_16K

/*
 * PWR system settings.
 * Reading STM32 Reference Manual is required, settings in PWR_CR3 are
 * very critical.
 * Register constants are taken from the ST header.
 */
#define STM32_PWR_CR1                       (PWR_CR1_SVOS_1 | PWR_CR1_SVOS_0)
#define STM32_PWR_CR2                       (PWR_CR2_BREN)
#ifdef SMPS_PWR
#define STM32_PWR_CR3                       (PWR_CR3_SMPSEN | PWR_CR3_USB33DEN)
#else
#define STM32_PWR_CR3                       (PWR_CR3_LDOEN | PWR_CR3_USB33DEN)
#endif
#define STM32_PWR_CPUCR                     0

/*
 * Clock tree static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_LSI_ENABLED                   FALSE
#define STM32_CSI_ENABLED                   FALSE
#define STM32_HSI48_ENABLED                 TRUE
#define STM32_LSE_ENABLED                   FALSE
#define STM32_HSIDIV                        STM32_HSIDIV_DIV1

/*
 * Setup clocks for the STM32H7A3
 */
#define STM32_VOS                           STM32_VOS_SCALE0
/*
  setup PLLs based on HSE clock
 */
#if STM32_HSECLK == 0U
// no crystal, this gives 280MHz system clock
#define STM32_HSE_ENABLED                   FALSE
#define STM32_HSI_ENABLED                   TRUE // HSI is 64MHzs
#define STM32_PLL1_DIVM_VALUE               32
#define STM32_PLL2_DIVM_VALUE               32
#define STM32_PLL3_DIVM_VALUE               32
#define STM32_PLLSRC                        STM32_PLLSRC_HSI_CK
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSI_CK
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSI_CK

#elif STM32_HSECLK == 8000000U
#define STM32_HSE_ENABLED                   TRUE
#define STM32_HSI_ENABLED                   FALSE
#define STM32_PLL1_DIVM_VALUE               4
#define STM32_PLL2_DIVM_VALUE               4
#define STM32_PLL3_DIVM_VALUE               4
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSE_CK

#elif STM32_HSECLK == 16000000U
#define STM32_HSE_ENABLED                   TRUE
#define STM32_HSI_ENABLED                   FALSE
#define STM32_PLL1_DIVM_VALUE               8
#define STM32_PLL2_DIVM_VALUE               8
#define STM32_PLL3_DIVM_VALUE               8
#define STM32_PLLSRC                        STM32_PLLSRC_HSE_CK
#define STM32_CKPERSEL                      STM32_CKPERSEL_HSE_CK

#else
#error "Unsupported HSE clock"
#endif

#if (STM32_HSECLK == 0U) || (STM32_HSECLK == 8000000U) || (STM32_HSECLK == 16000000U)
// common clock tree for multiples of 8MHz crystals
#define STM32_PLL1_DIVN_VALUE               280
#define STM32_PLL1_DIVP_VALUE               2
#define STM32_PLL1_DIVQ_VALUE               2
#define STM32_PLL1_DIVR_VALUE               2

#define STM32_PLL2_DIVN_VALUE               160
#define STM32_PLL2_DIVP_VALUE               2
#define STM32_PLL2_DIVQ_VALUE               4 // 80MHz for FDCAN
#define STM32_PLL2_DIVR_VALUE               2

#define STM32_PLL3_DIVN_VALUE               100
#define STM32_PLL3_DIVP_VALUE               2
#define STM32_PLL3_DIVQ_VALUE               2
#define STM32_PLL3_DIVR_VALUE               4
#endif // clock selection

/*
 * PLLs static settings.
 * Reading STM32 Reference Manual is required.
 */
#define STM32_PLLCFGR_MASK                  ~0

#define STM32_PLL1_ENABLED                  TRUE
#define STM32_PLL1_P_ENABLED                TRUE
#define STM32_PLL1_Q_ENABLED                TRUE
#define STM32_PLL1_R_ENABLED                TRUE
#define STM32_PLL1_FRACN_VALUE              0

#define STM32_PLL2_ENABLED                  TRUE
#define STM32_PLL2_P_ENABLED                TRUE
#define STM32_PLL2_Q_ENABLED                TRUE
#define STM32_PLL2_R_ENABLED                TRUE
#define STM32_PLL2_FRACN_VALUE              0

#define STM32_PLL3_ENABLED                  TRUE
#define STM32_PLL3_P_ENABLED                TRUE
#define STM32_PLL3_Q_ENABLED                TRUE
#define STM32_PLL3_R_ENABLED                TRUE
#define STM32_PLL3_FRACN_VALUE              0

/*
 * Core clocks dynamic settings (can be changed at runtime).
 * Reading STM32 Reference Manual is required.
 */
#define STM32_SW                            STM32_SW_PLL1_P_CK
#define STM32_RTCSEL                        STM32_RTCSEL_LSI_CK
#define STM32_D1CPRE                        STM32_D1CPRE_DIV1
#define STM32_D1HPRE                        STM32_D1HPRE_DIV2
#define STM32_D1PPRE3                       STM32_D1PPRE3_DIV2
#define STM32_D2PPRE1                       STM32_D2PPRE1_DIV2
#define STM32_D2PPRE2                       STM32_D2PPRE2_DIV2
#define STM32_D3PPRE4                       STM32_D3PPRE4_DIV2

/*
 * Peripherals clocks static settings.
 * Reading STM32 Reference Manual is required.
 */
#ifndef STM32_MCO1SEL
#define STM32_MCO1SEL                       STM32_MCO1SEL_HSE_CK
#endif
#ifndef STM32_MCO1PRE_VALUE
#define STM32_MCO1PRE_VALUE                 1
#endif
#ifndef STM32_MCO2SEL
#define STM32_MCO2SEL                       STM32_MCO2SEL_SYS_CK
#endif
#ifndef STM32_MCO2PRE_VALUE
#define STM32_MCO2PRE_VALUE                 1
#endif
#define STM32_TIMPRE_ENABLE                 TRUE
#define STM32_HRTIMSEL                      0
#define STM32_STOPKERWUCK                   0
#define STM32_STOPWUCK                      0
#define STM32_RTCPRE_VALUE                  2
#define STM32_SDMMCSEL                      STM32_SDMMCSEL_PLL2_R_CK
#define STM32_OCTOSPISEL                    STM32_OCTOSPISEL_HCLK
#define STM32_FMCSEL                        STM32_OCTOSPISEL_HCLK

#define STM32_SWPSEL                        STM32_SWPSEL_PCLK1
#define STM32_FDCANSEL                      STM32_FDCANSEL_PLL2_Q_CK
#define STM32_DFSDM1SEL                     STM32_DFSDM1SEL_PCLK2
#define STM32_SPDIFSEL                      STM32_SPDIFSEL_PLL1_Q_CK
#define STM32_SPI45SEL                      STM32_SPI45SEL_PCLK2
#define STM32_SPI123SEL                     STM32_SPI123SEL_PLL3_P_CK
#define STM32_SAI1SEL                       STM32_SAI1SEL_PLL1_Q_CK
#define STM32_LPTIM1SEL                     STM32_LPTIM1SEL_PCLK1
#define STM32_LPTIM3SEL                     STM32_LPTIM3SEL_PLL3_P_CK
#define STM32_CECSEL                        STM32_CECSEL_DISABLE
#define STM32_USBSEL                        STM32_USBSEL_HSI48_CK
#define STM32_I2C123SEL                     STM32_I2C123SEL_PLL3_R_CK
#define STM32_RNGSEL                        STM32_RNGSEL_HSI48_CK
#define STM32_USART16SEL                    STM32_USART16SEL_PLL3_Q_CK
#define STM32_USART234578SEL                STM32_USART234578SEL_PLL3_Q_CK
#define STM32_SPI6SEL                       STM32_SPI6SEL_PLL3_Q_CK
#define STM32_SAI4BSEL                      STM32_SAI4BSEL_PLL1_Q_CK
#define STM32_SAI4ASEL                      STM32_SAI4ASEL_PLL1_Q_CK
#define STM32_ADCSEL                        STM32_ADCSEL_PLL3_R_CK
#define STM32_LPTIM345SEL                   STM32_LPTIM345SEL_PCLK4
#define STM32_LPTIM2SEL                     STM32_LPTIM2SEL_PCLK4
#define STM32_I2C4SEL                       STM32_I2C4SEL_PLL3_R_CK
#define STM32_LPUART1SEL                    STM32_LPUART1SEL_PCLK4 // really PCLK3


/*
 * IRQ system settings.
 */
#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#define STM32_IRQ_EXTI16_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           15
#define STM32_IRQ_EXTI18_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6
#define STM32_IRQ_EXTI20_21_PRIORITY        6

#define STM32_IRQ_FDCAN1_PRIORITY           10
#define STM32_IRQ_FDCAN2_PRIORITY           10

#define STM32_IRQ_MDMA_PRIORITY             9
#define STM32_IRQ_OCTOSPI1_PRIORITY         10
#define STM32_IRQ_OCTOSPI2_PRIORITY         10

#define STM32_IRQ_SDMMC1_PRIORITY           9
#define STM32_IRQ_SDMMC2_PRIORITY           9

#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_TIM12_PRIORITY   7
#define STM32_IRQ_TIM8_UP_TIM13_PRIORITY    7
#define STM32_IRQ_TIM8_TRGCO_TIM14_PRIORITY 7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM15_PRIORITY            7
#define STM32_IRQ_TIM16_PRIORITY            7
#define STM32_IRQ_TIM17_PRIORITY            7

#define STM32_IRQ_USART1_PRIORITY           12
#define STM32_IRQ_USART2_PRIORITY           12
#define STM32_IRQ_USART3_PRIORITY           12
#define STM32_IRQ_UART4_PRIORITY            12
#define STM32_IRQ_UART5_PRIORITY            12
#define STM32_IRQ_USART6_PRIORITY           12
#define STM32_IRQ_UART7_PRIORITY            12
#define STM32_IRQ_UART8_PRIORITY            12
#define STM32_IRQ_UART9_PRIORITY            12
#define STM32_IRQ_USART10_PRIORITY          12

/*
 * ADC driver system settings.
 */
#ifndef STM32_ADC_DUAL_MODE
#define STM32_ADC_DUAL_MODE                 FALSE
#endif
#ifndef STM32_ADC_SAMPLES_SIZE
#define STM32_ADC_SAMPLES_SIZE              16
#endif
#define STM32_ADC_COMPACT_SAMPLES           FALSE
#define STM32_ADC_USE_ADC12                 TRUE
#define STM32_ADC_ADC12_DMA_PRIORITY        2
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#define STM32_ADC_ADC12_IRQ_PRIORITY        5
#define STM32_ADC_ADC3_IRQ_PRIORITY         5
#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_ADCCK
#define STM32_ADC_ADC3_CLOCK_MODE           ADC_CCR_CKMODE_ADCCK

// we call it ADC1 in hwdef.dat, but driver uses ADC12 for DMA stream
#ifdef STM32_ADC_ADC1_DMA_STREAM
#define STM32_ADC_ADC12_DMA_STREAM STM32_ADC_ADC1_DMA_STREAM
#elif defined(STM32_ADC_ADC2_DMA_STREAM)
#define STM32_ADC_ADC12_DMA_STREAM STM32_ADC_ADC2_DMA_STREAM
#endif

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_FDCAN1                FALSE
#define STM32_CAN_USE_FDCAN2                FALSE

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
#define STM32_GPT_USE_TIM12                 FALSE
#define STM32_GPT_USE_TIM13                 FALSE
#define STM32_GPT_USE_TIM14                 FALSE
#define STM32_GPT_USE_TIM15                 FALSE
#define STM32_GPT_USE_TIM16                 FALSE
#define STM32_GPT_USE_TIM17                 FALSE

/*
 * I2C driver system settings.
 */
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_IRQ_PRIORITY         5
#define STM32_I2C_I2C2_IRQ_PRIORITY         5
#define STM32_I2C_I2C3_IRQ_PRIORITY         5
#define STM32_I2C_I2C4_IRQ_PRIORITY         5
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_I2C3_DMA_PRIORITY         3
#define STM32_I2C_I2C4_DMA_PRIORITY         3
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      STM32_DMA_ERROR_HOOK(i2cp)

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_USE_TIM8                  FALSE
#define STM32_ICU_USE_TIM9                  FALSE
#define STM32_ICU_TIM1_IRQ_PRIORITY         7
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#define STM32_ICU_TIM5_IRQ_PRIORITY         7
#define STM32_ICU_TIM8_IRQ_PRIORITY         7
#define STM32_ICU_TIM9_IRQ_PRIORITY         7

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

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               32
#define STM32_RTC_PRESS_VALUE               1024
#define STM32_RTC_CR_INIT                   0
#define STM32_RTC_TAMPCR_INIT               0

/*
 * SDC driver system settings.
 */
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   TRUE
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       1000
#define STM32_SDC_SDMMC_READ_TIMEOUT        1000
#define STM32_SDC_SDMMC_CLOCK_DELAY         10
#define STM32_SDC_SDMMC1_DMA_PRIORITY       3
#define STM32_SDC_SDMMC1_IRQ_PRIORITY       9
#define STM32_SDC_SDMMC_PWRSAV              TRUE

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USART1_PRIORITY        12
#define STM32_SERIAL_USART2_PRIORITY        12
#define STM32_SERIAL_USART3_PRIORITY        12
#define STM32_SERIAL_UART4_PRIORITY         12
#define STM32_SERIAL_UART5_PRIORITY         12
#define STM32_SERIAL_USART6_PRIORITY        12
#define STM32_SERIAL_UART7_PRIORITY         12
#define STM32_SERIAL_UART8_PRIORITY         12

/*
 * SIO driver system settings.
 */
#define STM32_SIO_USE_USART1                FALSE
#define STM32_SIO_USE_USART2                FALSE
#define STM32_SIO_USE_USART3                FALSE
#define STM32_SIO_USE_UART4                 FALSE
#define STM32_SIO_USE_UART5                 FALSE
#define STM32_SIO_USE_USART6                FALSE
#define STM32_SIO_USE_UART7                 FALSE
#define STM32_SIO_USE_UART8                 FALSE
#define STM32_SIO_USE_LPUART1               FALSE

/*
 * SPI driver system settings.
 */
#ifndef STM32_SPI_USE_SPI1
#define STM32_SPI_USE_SPI1                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI2
#define STM32_SPI_USE_SPI2                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI3
#define STM32_SPI_USE_SPI3                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI4
#define STM32_SPI_USE_SPI4                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI5
#define STM32_SPI_USE_SPI5                  FALSE
#endif
#ifndef STM32_SPI_USE_SPI6
#define STM32_SPI_USE_SPI6                  FALSE
#endif
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#define STM32_SPI_SPI5_DMA_PRIORITY         1
#define STM32_SPI_SPI6_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         10
#define STM32_SPI_SPI2_IRQ_PRIORITY         10
#define STM32_SPI_SPI3_IRQ_PRIORITY         10
#define STM32_SPI_SPI4_IRQ_PRIORITY         10
#define STM32_SPI_SPI5_IRQ_PRIORITY         10
#define STM32_SPI_SPI6_IRQ_PRIORITY         10
#define STM32_SPI_DMA_ERROR_HOOK(spip)      STM32_DMA_ERROR_HOOK(spip)

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
#ifndef STM32_ST_USE_TIMER
#define STM32_ST_USE_TIMER                  5
#endif

/*
 * TRNG driver system settings.
 */
#define STM32_TRNG_USE_RNG1                 FALSE

/*
 * UART driver system settings.
 */
#define STM32_UART_USART1_IRQ_PRIORITY      12
#define STM32_UART_USART2_IRQ_PRIORITY      12
#define STM32_UART_USART3_IRQ_PRIORITY      12
#define STM32_UART_UART4_IRQ_PRIORITY       12
#define STM32_UART_UART5_IRQ_PRIORITY       12
#define STM32_UART_USART6_IRQ_PRIORITY      12
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_UART4_DMA_PRIORITY       0
#define STM32_UART_UART5_DMA_PRIORITY       0
#define STM32_UART_USART6_DMA_PRIORITY      0
#define STM32_UART_UART7_DMA_PRIORITY       0
#define STM32_UART_UART8_DMA_PRIORITY       0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    STM32_DMA_ERROR_HOOK(uartp)

#define STM32_IRQ_LPUART1_PRIORITY          12

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_OTG2                  TRUE
#define STM32_USB_USE_OTG1                  FALSE
#define STM32_USB_OTG1_IRQ_PRIORITY         14
#define STM32_USB_OTG2_IRQ_PRIORITY         14
#define STM32_USB_OTG1_RX_FIFO_SIZE         512
#define STM32_USB_OTG2_RX_FIFO_SIZE         1024
#define STM32_USB_HOST_WAKEUP_DURATION      2
#define STM32_OTG2_IS_OTG1                  TRUE

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

#define STM32_EXTI_ENHANCED

// limit ISR count per byte
#define STM32_I2C_ISR_LIMIT                 6

// limit SDMMC clock to 12.5MHz by default. This increases
// reliability
#ifndef STM32_SDC_MAX_CLOCK
#define STM32_SDC_MAX_CLOCK                 12500000
#endif

#ifndef STM32_WSPI_USE_OCTOSPI1
#define STM32_WSPI_USE_OCTOSPI1                  FALSE
#endif

#if HAL_XIP_ENABLED
#define STM32_OSPI1_NO_RESET TRUE
#define STM32_OSPI2_NO_RESET TRUE
#endif

#if STM32_WSPI_USE_OCTOSPI1
#define STM32_WSPI_OCTOSPI1_MDMA_CHANNEL    STM32_MDMA_CHANNEL_ID_ANY
#define STM32_WSPI_OCTOSPI1_MDMA_PRIORITY   1
#define STM32_WSPI_OCTOSPI1_PRESCALER_VALUE (STM32_OCTOSPICLK / HAL_OSPI1_CLK)
#endif

/*
  we use a fixed allocation of BDMA streams. We previously dynamically
  allocated these, but bugs in the chip make that unreliable. This is
  a tested set of allocations that is known to work on boards that are
  using all 3 of ADC3, I2C4 and SPI6. They are the only peripherals
  that can use BDMA, so fixed allocation is possible as we have 8
  streams and a maximum of 6 needed.

  The layout is chosen to:

   - avoid stream 0, as this doesn't work on ADC3 or SPI6_RX for no known reason
   - leave a gap between the peripheral types, as we have previously found that we sometimes
     lost SPI6 BDMA completion interrupts if SPI6 and I2c4 are neighbours
 */
#define STM32_I2C_I2C4_RX_BDMA_STREAM 1
#define STM32_I2C_I2C4_TX_BDMA_STREAM 2
#define STM32_SPI_SPI6_RX_BDMA_STREAM 4
#define STM32_SPI_SPI6_TX_BDMA_STREAM 5
#define STM32_ADC_ADC3_BDMA_STREAM 7

// disable DMA on I2C by default on H7
#ifndef STM32_I2C_USE_DMA
#define STM32_I2C_USE_DMA FALSE
#endif
