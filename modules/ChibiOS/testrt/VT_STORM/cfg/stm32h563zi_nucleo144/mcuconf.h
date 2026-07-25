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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * STM32H5xx drivers configuration.
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

#define STM32H5xx_MCUCONF
#define STM32H562_MCUCONF
#define STM32H563_MCUCONF
#define STM32H573_MCUCONF

/*
 * HAL driver general settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_CLOCK_DYNAMIC                 FALSE

/*
 * ICache settings.
 */
#define STM32_ICACHE_CR                     (ICACHE_CR_WAYSEL | ICACHE_CR_EN)
#define STM32_ICACHE_CRR0                   (0U)
#define STM32_ICACHE_CRR1                   (0U)
#define STM32_ICACHE_CRR2                   (0U)
#define STM32_ICACHE_CRR3                   (0U)

/*
 * PWR settings.
 */
#define STM32_PWR_VOSCR                     PWR_VOSCR_VOS_RANGE0
#define STM32_PWR_BDCR                      (0U)
#define STM32_PWR_UCPDR                     (0U)
#define STM32_PWR_SCCR                      (0U)
#define STM32_PWR_VMCR                      (0U)
#define STM32_PWR_USBSCR                    (0U)
#define STM32_PWR_WUCR                      (0U)
#define STM32_PWR_IORETR                    (0U)
#define STM32_PWR_SECCFGR                   (0U)
#define STM32_PWR_PRIVCFGR                  (0U)

/*
 * Clock settings.
 */
#define STM32_HSI_ENABLED                   TRUE
#define STM32_HSIDIV_VALUE                  2
#define STM32_HSI48_ENABLED                 FALSE
#define STM32_CSI_ENABLED                   FALSE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_LSI_ENABLED                   FALSE
#define STM32_LSE_ENABLED                   TRUE
#define STM32_SW                            RCC_CFGR1_SW_PLL1P
#define STM32_PLL1SRC                       RCC_PLL1CFGR_PLL1SRC_HSE
#define STM32_PLL1M_VALUE                   4
#define STM32_PLL1N_VALUE                   250
#define STM32_PLL1P_VALUE                   2
#define STM32_PLL1Q_VALUE                   4
#define STM32_PLL1R_VALUE                   2
#define STM32_PLL2SRC                       RCC_PLL2CFGR_PLL2SRC_HSE
#define STM32_PLL2M_VALUE                   4
#define STM32_PLL2N_VALUE                   200
#define STM32_PLL2P_VALUE                   2
#define STM32_PLL2Q_VALUE                   2
#define STM32_PLL2R_VALUE                   2
#define STM32_PLL3SRC                       RCC_PLL3CFGR_PLL3SRC_HSE
#define STM32_PLL3M_VALUE                   4
#define STM32_PLL3N_VALUE                   240
#define STM32_PLL3P_VALUE                   2
#define STM32_PLL3Q_VALUE                   10
#define STM32_PLL3R_VALUE                   2
#define STM32_HPRE                          RCC_CFGR2_HPRE_DIV1
#define STM32_PPRE1                         RCC_CFGR2_PPRE1_DIV1
#define STM32_PPRE2                         RCC_CFGR2_PPRE2_DIV1
#define STM32_PPRE3                         RCC_CFGR2_PPRE3_DIV1
#define STM32_STOPWUCK                      RCC_CFGR1_STOPWUCK_HSI
#define STM32_STOPKERWUCK                   RCC_CFGR1_STOPKERWUCK_HSI
#define STM32_TIMPRE                        RCC_CFGR1_TIMPRE_LOW
#define STM32_RTCPRE_VALUE                  RCC_CFGR1_RTCPRE_NOCLOCK
#define STM32_MCO1SEL                       RCC_CFGR1_MCO1SEL_HSI
#define STM32_MCO1PRE_VALUE                 RCC_CFGR1_MCO1PRE_NOCLOCK
#define STM32_MCO2SEL                       RCC_CFGR1_MCO2SEL_SYSCLK
#define STM32_MCO2PRE_VALUE                 RCC_CFGR1_MCO2PRE_NOCLOCK
#define STM32_LSCOSEL                       RCC_BDCR_LSCOSEL_NOCLOCK

/*
 * Peripherals clock sources.
 */
#define STM32_USART1SEL                     RCC_CCIPR1_USART1SEL_PCLK2
#define STM32_USART2SEL                     RCC_CCIPR1_USART2SEL_PCLK1
#define STM32_USART3SEL                     RCC_CCIPR1_USART3SEL_PCLK1
#define STM32_UART4SEL                      RCC_CCIPR1_UART4SEL_PCLK1
#define STM32_UART5SEL                      RCC_CCIPR1_UART5SEL_PCLK1
#define STM32_USART6SEL                     RCC_CCIPR1_USART6SEL_PCLK1
#define STM32_UART7SEL                      RCC_CCIPR1_UART7SEL_PCLK1
#define STM32_UART8SEL                      RCC_CCIPR1_UART8SEL_PCLK1
#define STM32_UART9SEL                      RCC_CCIPR1_UART9SEL_PCLK1
#define STM32_USART10SEL                    RCC_CCIPR1_USART10SEL_PCLK1
#define STM32_USART11SEL                    RCC_CCIPR2_USART11SEL_PCLK1
#define STM32_UART12SEL                     RCC_CCIPR2_UART12SEL_PCLK1
#define STM32_LPUART1SEL                    RCC_CCIPR3_LPUART1SEL_PCLK3
#define STM32_TIMICSEL                      RCC_CCIPR1_TIMICSEL_NOCLK
#define STM32_LPTIM1SEL                     RCC_CCIPR2_LPTIM1SEL_PCLK3
#define STM32_LPTIM2SEL                     RCC_CCIPR2_LPTIM2SEL_PCLK1
#define STM32_LPTIM3SEL                     RCC_CCIPR2_LPTIM3SEL_PCLK3
#define STM32_LPTIM4SEL                     RCC_CCIPR2_LPTIM4SEL_PCLK3
#define STM32_LPTIM5SEL                     RCC_CCIPR2_LPTIM5SEL_PCLK3
#define STM32_LPTIM6SEL                     RCC_CCIPR2_LPTIM6SEL_PCLK3
#define STM32_SPI1SEL                       RCC_CCIPR3_SPI1SEL_PLL1Q
#define STM32_SPI2SEL                       RCC_CCIPR3_SPI2SEL_PLL1Q
#define STM32_SPI3SEL                       RCC_CCIPR3_SPI3SEL_PLL1Q
#define STM32_SPI4SEL                       RCC_CCIPR3_SPI4SEL_PCLK2
#define STM32_SPI5SEL                       RCC_CCIPR3_SPI5SEL_PCLK3
#define STM32_SPI6SEL                       RCC_CCIPR3_SPI6SEL_PCLK2
#define STM32_OSPISEL                       RCC_CCIPR4_OSPISEL_HCLK4
#define STM32_SYSTICKSEL                    RCC_CCIPR4_SYSTICKSEL_HCLKDIV8
#define STM32_USBSEL                        RCC_CCIPR4_USBSEL_NOCLOCK
#define STM32_SDMMC1SEL                     RCC_CCIPR4_SDMMC1SEL_PLL1Q
#define STM32_SDMMC2SEL                     RCC_CCIPR4_SDMMC2SEL_PLL1Q
#define STM32_I2C1SEL                       RCC_CCIPR4_I2C1SEL_PCLK1
#define STM32_I2C2SEL                       RCC_CCIPR4_I2C2SEL_PCLK1
#define STM32_I2C3SEL                       RCC_CCIPR4_I2C3SEL_PCLK3
#define STM32_I2C4SEL                       RCC_CCIPR4_I2C4SEL_PCLK3
#define STM32_I3C1SEL                       RCC_CCIPR4_I3C1SEL_PCLK1
#define STM32_ADCDACSEL                     RCC_CCIPR5_ADCDACSEL_HCLK
#define STM32_DACSEL                        RCC_CCIPR5_DACSEL_IGNORE
#define STM32_RNGSEL                        RCC_CCIPR5_RNGSEL_IGNORE
#define STM32_CECSEL                        RCC_CCIPR5_CECSEL_IGNORE
#define STM32_FDCANSEL                      RCC_CCIPR5_FDCANSEL_IGNORE
#define STM32_SAI1SEL                       RCC_CCIPR5_SAI1SEL_PLL1Q
#define STM32_SAI2SEL                       RCC_CCIPR5_SAI2SEL_PLL1Q
#define STM32_CKPERSEL                      RCC_CCIPR5_CKPERSEL_HSI
#define STM32_RTCSEL                        RCC_BDCR_RTCSEL_LSE

/*
 * IRQ system settings.
 */
#define STM32_IRQ_DAC1_PRIORITY             9

#define STM32_IRQ_EXTI0_PRIORITY            6
#define STM32_IRQ_EXTI1_PRIORITY            6
#define STM32_IRQ_EXTI2_PRIORITY            6
#define STM32_IRQ_EXTI3_PRIORITY            6
#define STM32_IRQ_EXTI4_PRIORITY            6
#define STM32_IRQ_EXTI5_PRIORITY            6
#define STM32_IRQ_EXTI6_PRIORITY            6
#define STM32_IRQ_EXTI7_PRIORITY            6
#define STM32_IRQ_EXTI8_PRIORITY            6
#define STM32_IRQ_EXTI9_PRIORITY            6
#define STM32_IRQ_EXTI10_PRIORITY           6
#define STM32_IRQ_EXTI11_PRIORITY           6
#define STM32_IRQ_EXTI12_PRIORITY           6
#define STM32_IRQ_EXTI13_PRIORITY           6
#define STM32_IRQ_EXTI14_PRIORITY           6
#define STM32_IRQ_EXTI15_PRIORITY           6
#define STM32_IRQ_EXTI17_PRIORITY           6
#define STM32_IRQ_EXTI19_PRIORITY           6

#define STM32_IRQ_FDCAN1_PRIORITY           10
#define STM32_IRQ_FDCAN2_PRIORITY           10

#define STM32_IRQ_I2C1_PRIORITY             5
#define STM32_IRQ_I2C2_PRIORITY             5
#define STM32_IRQ_I2C3_PRIORITY             5
#define STM32_IRQ_I2C4_PRIORITY             5

#define STM32_IRQ_SPI1_PRIORITY             10
#define STM32_IRQ_SPI2_PRIORITY             10
#define STM32_IRQ_SPI3_PRIORITY             10
#define STM32_IRQ_SPI4_PRIORITY             10
#define STM32_IRQ_SPI5_PRIORITY             10
#define STM32_IRQ_SPI6_PRIORITY             10

#define STM32_IRQ_TIM1_BRK_PRIORITY         7
#define STM32_IRQ_TIM1_UP_PRIORITY          7
#define STM32_IRQ_TIM1_TRGCO_PRIORITY       7
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#define STM32_IRQ_TIM2_PRIORITY             7
#define STM32_IRQ_TIM3_PRIORITY             7
#define STM32_IRQ_TIM4_PRIORITY             7
#define STM32_IRQ_TIM5_PRIORITY             7
#define STM32_IRQ_TIM6_PRIORITY             7
#define STM32_IRQ_TIM7_PRIORITY             7
#define STM32_IRQ_TIM8_BRK_PRIORITY         7
#define STM32_IRQ_TIM8_UP_PRIORITY          7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM8_CC_PRIORITY          7
#define STM32_IRQ_TIM12_PRIORITY            7
#define STM32_IRQ_TIM13_PRIORITY            7
#define STM32_IRQ_TIM14_PRIORITY            7
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
#define STM32_IRQ_USART11_PRIORITY          12
#define STM32_IRQ_UART12_PRIORITY           12
#define STM32_IRQ_LPUART1_PRIORITY          12

#define STM32_IRQ_USB1_PRIORITY             13

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  FALSE
#define STM32_ADC_USE_ADC2                  FALSE
#define STM32_ADC_DUAL_MODE                 FALSE
#define STM32_ADC_COMPACT_SAMPLES           FALSE
#define STM32_ADC_ADC1_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_ADC_ADC2_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#define STM32_ADC_ADC2_DMA_PRIORITY         2
#define STM32_ADC_ADC1_IRQ_PRIORITY         5
#define STM32_ADC_ADC2_IRQ_PRIORITY         5
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     5
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     5
#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC_ADC12_PRESC               ADC_CCR_PRESC_DIV2

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
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH1_DMA3_CHANNEL     STM32_DMA3_MASK_FIFO2
#define STM32_DAC_DAC1_CH2_DMA3_CHANNEL     STM32_DMA3_MASK_FIFO2

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  TRUE
#define STM32_GPT_USE_TIM4                  TRUE
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
#define STM32_I2C_USE_I2C1                  FALSE
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_USE_I2C3                  FALSE
#define STM32_I2C_USE_I2C4                  FALSE
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_DMA_PRIORITY         3
#define STM32_I2C_I2C2_DMA_PRIORITY         3
#define STM32_I2C_I2C3_DMA_PRIORITY         3
#define STM32_I2C_I2C4_DMA_PRIORITY         3
#define STM32_I2C_I2C1_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_I2C_I2C2_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_I2C_I2C3_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_I2C_I2C4_DMA3_CHANNEL         STM32_DMA3_MASK_FIFO2
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_USE_TIM4                  FALSE
#define STM32_ICU_USE_TIM5                  FALSE
#define STM32_ICU_USE_TIM8                  FALSE
#define STM32_ICU_USE_TIM12                 FALSE
#define STM32_ICU_USE_TIM15                 FALSE
#define STM32_ICU_USE_TIM16                 FALSE
#define STM32_ICU_USE_TIM17                 FALSE

/*
 * MAC driver system settings.
 */
#define STM32_MAC_TRANSMIT_BUFFERS          2
#define STM32_MAC_RECEIVE_BUFFERS           4
#define STM32_MAC_BUFFERS_SIZE              1522
#define STM32_MAC_PHY_TIMEOUT               100
#define STM32_MAC_ETH1_CHANGE_PHY_STATE     TRUE
#define STM32_MAC_ETH1_IRQ_PRIORITY         13
#define STM32_MAC_IP_CHECKSUM_OFFLOAD       FALSE

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_TIM1                  FALSE
#define STM32_PWM_USE_TIM2                  FALSE
#define STM32_PWM_USE_TIM3                  FALSE
#define STM32_PWM_USE_TIM4                  FALSE
#define STM32_PWM_USE_TIM5                  FALSE
#define STM32_PWM_USE_TIM8                  FALSE
#define STM32_PWM_USE_TIM12                 FALSE
#define STM32_PWM_USE_TIM13                 FALSE
#define STM32_PWM_USE_TIM14                 FALSE
#define STM32_PWM_USE_TIM15                 FALSE
#define STM32_PWM_USE_TIM16                 FALSE
#define STM32_PWM_USE_TIM17                 FALSE

/*
 * RTC driver system settings.
 */
#define STM32_RTC_PRESA_VALUE               32
#define STM32_RTC_PRESS_VALUE               1024
#define STM32_RTC_CR_INIT                   0
#define STM32_TAMP_CR1_INIT                 0
#define STM32_TAMP_CR2_INIT                 0
#define STM32_TAMP_FLTCR_INIT               0
#define STM32_TAMP_IER_INIT                 0

/*
 * SDC driver system settings.
 */
#define STM32_SDC_USE_SDMMC1                FALSE
#define STM32_SDC_USE_SDMMC2                FALSE
#define STM32_SDC_SDMMC_UNALIGNED_SUPPORT   TRUE
#define STM32_SDC_SDMMC_WRITE_TIMEOUT       10000
#define STM32_SDC_SDMMC_READ_TIMEOUT        10000
#define STM32_SDC_SDMMC_CLOCK_DELAY         10
#define STM32_SDC_SDMMC_PWRSAV              TRUE

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             FALSE
#define STM32_SERIAL_USE_USART2             FALSE
#define STM32_SERIAL_USE_USART3             TRUE
#define STM32_SERIAL_USE_UART4              FALSE
#define STM32_SERIAL_USE_UART5              FALSE
#define STM32_SERIAL_USE_USART6             FALSE
#define STM32_SERIAL_USE_UART7              FALSE
#define STM32_SERIAL_USE_UART8              FALSE
#define STM32_SERIAL_USE_UART9              FALSE
#define STM32_SERIAL_USE_USART10            FALSE
#define STM32_SERIAL_USE_USART11            FALSE
#define STM32_SERIAL_USE_UART12             FALSE
#define STM32_SERIAL_USE_LPUART1            FALSE

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
#define STM32_SIO_USE_UART9                 FALSE
#define STM32_SIO_USE_USART10               FALSE
#define STM32_SIO_USE_USART11               FALSE
#define STM32_SIO_USE_UART12                FALSE
#define STM32_SIO_USE_LPUART1               FALSE

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  FALSE
#define STM32_SPI_USE_SPI4                  FALSE
#define STM32_SPI_USE_SPI5                  FALSE
#define STM32_SPI_USE_SPI6                  FALSE
#define STM32_SPI_SPI1_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI1_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI2_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI2_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI3_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI3_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI4_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI4_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI5_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI5_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI6_RX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI6_TX_DMA3_CHANNEL      STM32_DMA3_MASK_FIFO2
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#define STM32_SPI_SPI5_DMA_PRIORITY         1
#define STM32_SPI_SPI6_DMA_PRIORITY         1
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               8
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
