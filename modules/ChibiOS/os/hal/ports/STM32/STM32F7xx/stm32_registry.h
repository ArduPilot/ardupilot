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
 * @file    STM32F7xx/stm32_registry.h
 * @brief   STM32F7xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    STM32F7xx capabilities
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

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              128
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

#if defined(STM32F732xx) || defined(STM32F733xx) || defined(STM32F756xx) || \
    defined(STM32F777xx) || defined(STM32F779xx) || defined(__DOXYGEN__)
#define STM32_HAS_HASH1                     TRUE
#define STM32_HAS_CRYP1                     TRUE
#define STM32_HASH1_DMA_MSK                 STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_HASH1_DMA_CHN                 0x20000000
#define STM32_CRYP1_IN_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 6)
#define STM32_CRYP1_IN_DMA_CHN              0x02000000
#define STM32_CRYP1_OUT_DMA_MSK             STM32_DMA_STREAM_ID_MSK(2, 5)
#define STM32_CRYP1_OUT_DMA_CHN             0x00200000

#else /* Devices without cryp nor hash.*/
#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     FALSE
#endif

/*===========================================================================*/
/* STM32F722xx, STM32F723xx, STM32F732xx, STM32F733xx.                       */
/*===========================================================================*/
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F732xx) || \
    defined(STM32F733xx) || defined(__DOXYGEN__)
/* ADC attributes.*/
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
#define STM32_CAN_MAX_FILTERS               28

#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                24
#define STM32_EXTI_IMR1_MASK                0xFF000000

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
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_HAS_INTERRUPTS            FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDMMC1_DMA_CHN            0x04004000

#define STM32_HAS_SDMMC2                    TRUE
#define STM32_SDC_SDMMC2_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC2_DMA_CHN            0x00B0000B

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
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

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

#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                8

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32F722xx) || defined(STM32F723xx) ||
          defined(STM32F732xx) || defined(STM32F733xx) */

/*===========================================================================*/
/* STM32F745xx, STM32F746xx, STM32F756xx.                                    */
/*===========================================================================*/
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F756xx) || \
    defined(__DOXYGEN__)
/* ADC attributes.*/
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
#define STM32_CAN_MAX_FILTERS               28
#define STM32_CAN3_MAX_FILTERS              14

#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      TRUE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                24
#define STM32_EXTI_IMR1_MASK                0xFF000000

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
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C3_TX_DMA_CHN               0x00030000

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C4_RX_DMA_CHN               0x00000200
#define STM32_I2C4_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C4_TX_DMA_CHN               0x00200000

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 7)
#define STM32_QUADSPI1_DMA_CHN              0x30000000

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_HAS_INTERRUPTS            FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDMMC1_DMA_CHN            0x04004000

#define STM32_HAS_SDMMC2                    FALSE

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
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050040

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00702000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4) |            \
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI6_RX_DMA_CHN               0x01000000
#define STM32_SPI6_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI6_TX_DMA_CHN               0x00100000

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

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
#define STM32_TIM8_UP_HANDLER               VectorF0
#define STM32_TIM8_CC_HANDLER               VectorF8
#define STM32_TIM8_UP_NUMBER                44
#define STM32_TIM8_CC_NUMBER                46

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

#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                8

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F756xx) */

/*===========================================================================*/
/* STM32F765xx, STM32F767xx, STM32F769xx, STM32F777xx, STM32F779xx.          */
/*===========================================================================*/
#if defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F777xx) || defined(STM32F779xx) ||                         \
    defined(__DOXYGEN__)
/* ADC attributes.*/
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
#define STM32_CAN_MAX_FILTERS               28
#define STM32_CAN3_MAX_FILTERS              14

#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      TRUE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_DAC1_CH1_DMA_CHN              0x00700000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_DAC1_CH2_DMA_CHN              0x07000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_CACHE_HANDLING            TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                24
#define STM32_EXTI_IMR1_MASK                0xFF000000

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
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C2_TX_DMA_CHN               0x70080000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_RX_DMA_CHN               0x00000310
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C3_TX_DMA_CHN               0x00030008

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_EVENT_HANDLER            Vector1BC
#define STM32_I2C4_ERROR_HANDLER            Vector1C0
#define STM32_I2C4_EVENT_NUMBER             95
#define STM32_I2C4_ERROR_NUMBER             96
#define STM32_I2C4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 1))
#define STM32_I2C4_RX_DMA_CHN               0x00000280
#define STM32_I2C4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C4_TX_DMA_CHN               0x08200000

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x30000B00

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_HAS_INTERRUPTS            FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SDC_SDMMC1_DMA_CHN            0x04004000

#define STM32_HAS_SDMMC2                    TRUE
#define STM32_SDC_SDMMC2_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC2_DMA_CHN            0x00B0000B

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
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI2_RX_DMA_CHN               0x00000090
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_TX_DMA_CHN               0x09000000

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
#define STM32_SPI4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 0) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI4_RX_DMA_CHN               0x00005004
#define STM32_SPI4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI4_TX_DMA_CHN               0x00050940

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI5_RX_DMA_CHN               0x00902000
#define STM32_SPI5_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 4)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI5_TX_DMA_CHN               0x07020000

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_SPI6_RX_DMA_CHN               0x01000000
#define STM32_SPI6_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SPI6_TX_DMA_CHN               0x00100000

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

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

#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                8

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) ||
          defined(STM32F777xx) || defined(STM32F779xx) */
/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
