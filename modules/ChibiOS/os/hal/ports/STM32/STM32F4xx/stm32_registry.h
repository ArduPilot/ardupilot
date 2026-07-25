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

/**
 * @file    STM32F4xx/stm32_registry.h
 * @brief   STM32F4xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

#if defined(STM32F469xx) || defined(STM32F479xx)
#define STM32F469_479xx
#define STM32F4XX

#elif defined(STM32F446xx)
#define STM32F4XX

#elif defined(STM32F439xx) || defined(STM32F429xx)
#define STM32F429_439xx
#define STM32F4XX

#elif defined(STM32F437xx) || defined(STM32F427xx)
#define STM32F427_437xx
#define STM32F4XX

#elif defined(STM32F413xx)
#define STM32F413xx
#define STM32F4XX

#elif defined(STM32F412Cx) || defined(STM32F412Rx) ||                       \
      defined(STM32F412Vx) || defined(STM32F412Zx)
#define STM32F412xx
#define STM32F4XX

#elif defined(STM32F411xE)
#define STM32F411xx
#define STM32F4XX

#elif defined(STM32F410Cx) || defined(STM32F410Rx) ||                       \
      defined(STM32F410Tx)
#define STM32F410xx
#define STM32F4XX

#elif defined(STM32F405xx) || defined(STM32F415xx) ||                       \
      defined(STM32F407xx) || defined(STM32F417xx)
#define STM32F40_41xxx
#define STM32F4XX

#elif defined(STM32F401xC) || defined(STM32F401xE)
#define STM32F401xx
#define STM32F4XX

#elif defined(STM32F205xx) || defined(STM32F215xx) ||                       \
      defined(STM32F207xx) || defined(STM32F217xx)
#define STM32F2XX

#else
#error "STM32F2xx/F4xx device not specified"
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    STM32F4xx/STM32F2xx capabilities
 * @{
 */

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM4_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP
#define STM32_DBGMCU_TIM5_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM8_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM8_STOP
#define STM32_DBGMCU_TIM9_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM9_STOP
#define STM32_DBGMCU_TIM10_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM10_STOP
#define STM32_DBGMCU_TIM11_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM11_STOP
#define STM32_DBGMCU_TIM12_STOP()           DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM12_STOP
#define STM32_DBGMCU_TIM13_STOP()           DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM13_STOP
#define STM32_DBGMCU_TIM14_STOP()           DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM14_STOP

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   FALSE

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#if !defined(STM32F2XX)
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#else
#define STM32_RTC_HAS_SUBSECONDS            FALSE
#endif
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              80
#define STM32_RTC_TAMP_STAMP_HANDLER        Vector48
#define STM32_RTC_WKUP_HANDLER              Vector4C
#define STM32_RTC_ALARM_HANDLER             VectorE4
#define STM32_RTC_TAMP_STAMP_NUMBER         2
#define STM32_RTC_WKUP_NUMBER               3
#define STM32_RTC_ALARM_NUMBER              41
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           21
#define STM32_RTC_WKUP_EXTI                 22
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI21_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI22_PRIORITY);       \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI17_PRIORITY);      \
} while (false)

/*===========================================================================*/
/* STM32F469xx, STM32F479xx.                                                 */
/*===========================================================================*/

#if defined(STM32F469_479xx) || defined(__DOXYGEN__)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                TRUE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              TRUE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLSAI_P_CLKOUT
#define STM32_TIMPRE_PRESCALE4              TRUE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC2_DMA_CHN                  0x00001100

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_ADC3_DMA_CHN                  0x00000022

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE
#define STM32_ETH_HANDLER                   Vector134
#define STM32_ETH_NUMBER                    61

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOI                     TRUE
#define STM32_HAS_GPIOJ                     TRUE
#define STM32_HAS_GPIOK                     TRUE

#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN |          \
                                             RCC_AHB1ENR_GPIOIEN |          \
                                             RCC_AHB1ENR_GPIOJEN |          \
                                             RCC_AHB1ENR_GPIOKEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_HANDLER              Vector1AC
#define STM32_QUADSPI1_NUMBER               91
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE
#define STM32_SPI6_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(2, 6)
#define STM32_SPI6_RX_DMA_CHN               0x01000000
#define STM32_SPI6_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(2, 5)
#define STM32_SPI6_TX_DMA_CHN               0x00100000

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_UART4_RX_DMA_CHN              0x00000400
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_UART4_TX_DMA_CHN              0x00040000

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART5_RX_DMA_CHN              0x00000004
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART5_TX_DMA_CHN              0x40000000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     TRUE
#define STM32_UART7_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_UART7_RX_DMA_CHN              0x00005000
#define STM32_UART7_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_UART7_TX_DMA_CHN              0x00000050

#define STM32_HAS_UART8                     TRUE
#define STM32_UART8_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_UART8_RX_DMA_CHN              0x05000000
#define STM32_UART8_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART8_TX_DMA_CHN              0x00000005

#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5
#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                7

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE
#define STM32_FSMC_HANDLER                  Vector100
#define STM32_FSMC_NUMBER                   48

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F469_479xx) */

/*===========================================================================*/
/* STM32F446xx.                                                              */
/*===========================================================================*/

#if defined(STM32F446xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                TRUE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              TRUE
#define STM32_HAS_RCC_I2SSRC                FALSE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              TRUE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLSAI_P_CLKOUT
#define STM32_TIMPRE_PRESCALE4              TRUE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC2_DMA_CHN                  0x00001100

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_ADC3_DMA_CHN                  0x00000022

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE

#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_HANDLER              Vector1B0
#define STM32_QUADSPI1_NUMBER               92
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           FALSE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_UART4_RX_DMA_CHN              0x00000400
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_UART4_TX_DMA_CHN              0x00040000

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART5_RX_DMA_CHN              0x00000004
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART5_TX_DMA_CHN              0x40000000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5
#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                7

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE
#define STM32_FSMC_HANDLER                  Vector100
#define STM32_FSMC_NUMBER                   48

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F446xx) */

/*===========================================================================*/
/* STM32F439xx, STM32F429xx, STM32F437xx, STM32F427xx.                       */
/*===========================================================================*/

#if defined(STM32F429_439xx) || defined(STM32F427_437xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                TRUE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_TIMPRE_PRESCALE4              TRUE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC2_DMA_CHN                  0x00001100

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_ADC3_DMA_CHN                  0x00000022

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE
#define STM32_ETH_HANDLER                   Vector134
#define STM32_ETH_NUMBER                    61

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOI                     TRUE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN |          \
                                             RCC_AHB1ENR_GPIOIEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE
#define STM32_SPI6_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI6_RX_DMA_CHN               0x01000000
#define STM32_SPI6_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI6_TX_DMA_CHN               0x00100000

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_UART4_RX_DMA_CHN              0x00000400
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_UART4_TX_DMA_CHN              0x00040000

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART5_RX_DMA_CHN              0x00000004
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART5_TX_DMA_CHN              0x40000000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     TRUE
#define STM32_UART7_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_UART7_RX_DMA_CHN              0x00005000
#define STM32_UART7_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_UART7_TX_DMA_CHN              0x00000050

#define STM32_HAS_UART8                     TRUE
#define STM32_UART8_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_UART8_RX_DMA_CHN              0x05000000
#define STM32_UART8_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART8_TX_DMA_CHN              0x00000005

#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  1
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                3
#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                5

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE
#define STM32_FSMC_HANDLER                  Vector100
#define STM32_FSMC_NUMBER                   48

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F429_439xx) || defined(STM32F427_437xx) */

/*===========================================================================*/
/* STM32F413xx.                                                              */
/*===========================================================================*/

#if defined(STM32F413xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              TRUE
#define STM32_HAS_RCC_I2SSRC                FALSE
#define STM32_HAS_RCC_I2SPLLSRC             TRUE
#define STM32_HAS_RCC_CK48MSEL              TRUE
#define STM32_RCC_CK48MSEL_USES_I2S         TRUE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLI2S_Q_CLKOUT
#define STM32_TIMPRE_PRESCALE4              FALSE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      TRUE
#define STM32_CAN_MAX_FILTERS               28
#define STM32_CAN3_MAX_FILTERS              14

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C3_TX_DMA_CHN               0x00630000

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_SUPPORTS_FMP             TRUE
#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C4_RX_DMA_CHN               0x00001000
#define STM32_I2C4_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_I2C4_TX_DMA_CHN               0x00000020

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_HANDLER              Vector1B0
#define STM32_QUADSPI1_NUMBER               92
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303200

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             TRUE
#define STM32_SPI4_I2S_FULLDUPLEX           TRUE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_RX_DMA_CHN               0x00045004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             TRUE
#define STM32_SPI5_I2S_FULLDUPLEX           TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07520000

#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

#define STM32_HAS_LPTIM1                    TRUE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_UART4_RX_DMA_CHN              0x00000400
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_UART4_TX_DMA_CHN              0x00040000

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART5_RX_DMA_CHN              0x00000004
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART5_TX_DMA_CHN              0x80000000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     TRUE
#define STM32_UART7_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_UART7_RX_DMA_CHN              0x00005000
#define STM32_UART7_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_UART7_TX_DMA_CHN              0x00000050

#define STM32_HAS_UART8                     TRUE
#define STM32_UART8_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_UART8_RX_DMA_CHN              0x05000000
#define STM32_UART8_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART8_TX_DMA_CHN              0x00000005

#define STM32_HAS_UART9                     TRUE
#define STM32_UART9_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_UART9_RX_DMA_CHN              0x00000000
#define STM32_UART9_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 0)
#define STM32_UART9_TX_DMA_CHN              0x00000001

#define STM32_HAS_UART10                    TRUE
#define STM32_UART10_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 0)
#define STM32_UART10_RX_DMA_CHN             0x00000005
#define STM32_UART10_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_UART10_TX_DMA_CHN             0x60000000

#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      FALSE
#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F413xx) */

/*===========================================================================*/
/* STM32F412xx.                                                              */
/*===========================================================================*/

#if defined(STM32F412xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              TRUE
#define STM32_HAS_RCC_I2SSRC                FALSE
#define STM32_HAS_RCC_I2SPLLSRC             TRUE
#define STM32_HAS_RCC_CK48MSEL              TRUE
#define STM32_RCC_CK48MSEL_USES_I2S         TRUE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLI2S_Q_CLKOUT
#define STM32_TIMPRE_PRESCALE4              FALSE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C3_TX_DMA_CHN               0x00630000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_HANDLER              Vector1B0
#define STM32_QUADSPI1_NUMBER               92
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             TRUE
#define STM32_SPI4_I2S_FULLDUPLEX           TRUE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_RX_DMA_CHN               0x00045004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             TRUE
#define STM32_SPI5_I2S_FULLDUPLEX           TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07520000

#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      FALSE
#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F412xx) */

/*===========================================================================*/
/* STM32F411xx.                                                              */
/*===========================================================================*/

#if defined(STM32F411xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLSAI_Q_CLKOUT
#define STM32_TIMPRE_PRESCALE4              FALSE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 1))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
+                                            STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
+                                            STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C3_TX_DMA_CHN               0x00630000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             TRUE
#define STM32_SPI4_I2S_FULLDUPLEX           TRUE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             TRUE
#define STM32_SPI5_I2S_FULLDUPLEX           TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM6                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  1
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                3

#define STM32_HAS_OTG2                      FALSE
#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F411xx) */

/*===========================================================================*/
/* STM32F410xx.                                                              */
/*===========================================================================*/

#if defined(STM32F410xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                FALSE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              TRUE
#define STM32_HAS_RCC_I2SSRC                FALSE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_PLL48CLK_ALTSRC               STM32_PLLSAI_Q_CLKOUT
#define STM32_TIMPRE_PRESCALE4              FALSE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      FALSE

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0)) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C4_RX_DMA_CHN               0x00002007
#define STM32_I2C4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7)) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 1))
#define STM32_I2C4_TX_DMA_CHN               0x00040020

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00003200

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             TRUE
#define STM32_SPI5_I2S_FULLDUPLEX           TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM2                      FALSE
#define STM32_HAS_TIM3                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_RX_DMA_CHN             0x60400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       FALSE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F410xx) */

/*===========================================================================*/
/* STM32F405xx, STM32F415xx, STM32F407xx, STM32F417xx, STM32F205xx,          */
/* STM32F215xx, STM32F207xx, STM32F217xx.                                    */
/*===========================================================================*/

#if defined(STM32F40_41xxx) || defined(STM32F2XX)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               FALSE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_TIMPRE_PRESCALE4              TRUE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC2_DMA_CHN                  0x00001100

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_ADC3_DMA_CHN                  0x00000022

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F205xx) || \
    defined(STM32F215xx)
#define STM32_HAS_ETH                       FALSE
#else
#define STM32_HAS_ETH                       TRUE
#define STM32_ETH_HANDLER                   Vector134
#define STM32_ETH_NUMBER                    61
#endif

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOI                     TRUE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN |          \
                                             RCC_AHB1ENR_GPIOIEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 6

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 1)
#define STM32_USART3_RX_DMA_CHN             0x00000040
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x00074000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_UART4_RX_DMA_CHN              0x00000400
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_UART4_TX_DMA_CHN              0x00040000

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 0)
#define STM32_UART5_RX_DMA_CHN              0x00000004
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART5_TX_DMA_CHN              0x40000000

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                3
#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                5

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F40_41xxx) || defined(STM32F2XX) */

/*===========================================================================*/
/* STM32F401xx.                                                              */
/*===========================================================================*/

#if defined(STM32F401xx)

/* Clock tree attributes.*/
#define STM32_HAS_RCC_PLLSAI                FALSE
#define STM32_HAS_RCC_PLLI2S                TRUE
#define STM32_HAS_RCC_DCKCFGR               TRUE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_TIMPRE_PRESCALE4              FALSE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC2_DMA_CHN                  0x00001100

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_ADC3_DMA_CHN                  0x00000022

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C1_RX_DMA_CHN               0x00100001
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x11000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C2_RX_DMA_CHN               0x00007700
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_I2C2_TX_DMA_CHN               0x70000000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE
#define STM32_SDC_SDIO_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDIO_DMA_CHN              0x04004000

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000303
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI1_TX_DMA_CHN               0x00303000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI3_RX_DMA_CHN               0x00000000
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI3_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM6                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_RX_DMA_CHN             0x00400400
#define STM32_USART1_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_USART1_TX_DMA_CHN             0x40000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00400000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_USART2_TX_DMA_CHN             0x04000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_USART6_RX_DMA_CHN             0x00000550
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(2, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART6_TX_DMA_CHN             0x55000000

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  1
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                3
#define STM32_HAS_OTG2                      FALSE

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F401xx) */
/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
