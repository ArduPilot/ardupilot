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
 * @file    STM32F0xx/stm32_registry.h
 * @brief   STM32F0xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

#if !defined(STM32F0XX) || defined(__DOXYGEN__)
#define STM32F0XX
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    STM32F0xx capabilities
 * @{
 */

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM14_STOP()           DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM14_STOP
#define STM32_DBGMCU_TIM15_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM15_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM17_STOP

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   FALSE

/* RNG attributes.*/
#define STM32_HAS_RNG1                      FALSE

/*===========================================================================*/
/* STM32F030x4, STM32F030x6, STM32F030x8, STM32F030xC.                       */
/*===========================================================================*/
#if defined(STM32F030x4) || defined(STM32F030x6) ||                         \
    defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)

/* Common identifier of all STM32F030 devices.*/
#define STM32F030

/* RCC attributes. */
#define STM32_HAS_HSI48                     FALSE
#if defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_HSI_PREDIV                TRUE
#else
#define STM32_HAS_HSI_PREDIV                FALSE
#endif
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000011

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

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
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#if defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#else
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#endif
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                20
#define STM32_EXTI_IMR1_MASK                0xFFF50000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             1024U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        32 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#if !defined(STM32F030x4) && !defined(STM32F030x6)
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#else
#define STM32_HAS_GPIOC                     FALSE
#define STM32_HAS_GPIOD                     FALSE
#endif
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000200
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000020

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00020000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00002000
#else
#define STM32_HAS_I2C2                      FALSE
#endif

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#if defined (STM32F030xC)
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#else
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#endif
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000030
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000300

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_RX_DMA_CHN               0x00003000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_SPI2_TX_DMA_CHN               0x00030000
#else
#define STM32_HAS_SPI2                      FALSE
#endif

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0
#else
#define STM32_HAS_TIM6                      FALSE
#endif

#if defined(STM32F030xC)
#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0
#else
#define STM32_HAS_TIM7                      FALSE
#endif

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2
#else
#define STM32_HAS_TIM15                     FALSE
#endif

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM2                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00080808
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00008080

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART2_RX_DMA_CHN             0x00090909
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART2_TX_DMA_CHN             0x00009090
#else
#define STM32_HAS_USART2                    FALSE
#endif

#if defined(STM32F030xC) || defined(__DOXYGEN__)
#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART3_RX_DMA_CHN             0x000A0A0A
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART3_TX_DMA_CHN             0x0000A0A0

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_UART4_RX_DMA_CHN              0x000B0B0B
#define STM32_UART4_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_UART4_TX_DMA_CHN              0x0000B0B0

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_UART5_RX_DMA_CHN              0x000C0C0C
#define STM32_UART5_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_UART5_TX_DMA_CHN              0x0000C0C0

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART6_RX_DMA_CHN             0x000D0D0D
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART6_TX_DMA_CHN             0x0000D0D0

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#else
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#endif

/* USB attributes.*/
#define STM32_HAS_USB                       FALSE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

/*===========================================================================*/
/* STM32F031x6, STM32F038xx.                                                 */
/*===========================================================================*/
#elif defined(STM32F031x6) || defined(STM32F038xx)

/* RCC attributes. */
#define STM32_HAS_HSI48                     FALSE
#define STM32_HAS_HSI_PREDIV                FALSE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

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
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x0FF40000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             1024U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        32 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM6                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    FALSE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       FALSE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32F042x6.                                                              */
/*===========================================================================*/
#elif defined(STM32F042x6)

/* RCC attributes. */
#define STM32_HAS_HSI48                     TRUE
#define STM32_HAS_HSI_PREDIV                TRUE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               14

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x7FF40000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             1024U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        32 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM6                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00000000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_USART2_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  768
#define STM32_USB_HAS_BCDR                  TRUE

#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32F048xx.                                                              */
/*===========================================================================*/
#elif defined(STM32F048xx)

/* RCC attributes. */
#define STM32_HAS_HSI48                     TRUE
#define STM32_HAS_HSI_PREDIV                TRUE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

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
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x7FF40000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             1024U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        32 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM6                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00000000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_USART2_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  768
#define STM32_USB_HAS_BCDR                  TRUE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32F051x8, STM32F058xx.                                                 */
/*===========================================================================*/
#elif defined(STM32F051x8) || defined(STM32F058xx)

/* RCC attributes. */
#define STM32_HAS_HSI48                     FALSE
#define STM32_HAS_HSI_PREDIV                FALSE
#define STM32_HAS_MCO_PREDIV                FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_DAC1_CH1_DMA_CHN              0x00000000

#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0

#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x0F940000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             1024U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        64 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00000000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00000000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_USART2_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       FALSE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32F070x6, STM32F070xB.                                                 */
/*===========================================================================*/
#elif defined(STM32F070x6) || defined(STM32F070xB)

/* Common identifier of all STM32F070 devices.*/
#define STM32F070

/* RCC attributes. */
#define STM32_HAS_HSI48                     FALSE
#define STM32_HAS_HSI_PREDIV                TRUE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

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
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x7F840000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             2048U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        128 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#if defined(STM32F070x6)
#define STM32_HAS_GPIOD                     FALSE
#else
#define STM32_HAS_GPIOD                     TRUE
#endif
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00000000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#if defined (STM32F070xB)
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#else
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#endif
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM2                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_USART2_RX_DMA_CHN             0x00000000
#define STM32_USART2_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_USART2_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_USART3_RX_DMA_CHN             0x00000000
#define STM32_USART3_TX_DMA_MSK             STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_USART3_TX_DMA_CHN             0x00000000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              0
#define STM32_UART4_RX_DMA_CHN              0x00000000
#define STM32_UART4_TX_DMA_MSK              0
#define STM32_UART4_TX_DMA_CHN              0x00000000

#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  768
#define STM32_USB_HAS_BCDR                  TRUE

#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

/*===========================================================================*/
/* STM32F071xB, STM32F072xB, STM32F078xx.                                    */
/*===========================================================================*/
#elif defined(STM32F071xB) || defined(STM32F072xB) ||                       \
      defined(STM32F078xx)

/* RCC attributes. */
#define STM32_HAS_HSI48                     TRUE
#define STM32_HAS_HSI_PREDIV                TRUE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#if defined(STM32F072xB)
#define STM32_HAS_CAN1                      TRUE
#define STM32_CAN_MAX_FILTERS               14
#else
#define STM32_HAS_CAN1                      FALSE
#endif
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_DAC1_CH1_DMA_CHN              0x00000000

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_DAC1_CH2_DMA_CHN              0x00000000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH4567_NUMBER            11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x7F840000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             2048U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        128 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOEEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x00000000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00000000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00000000

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 2)
#define STM32_SPI1_RX_DMA_CHN               0x00000000
#define STM32_SPI1_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 3)
#define STM32_SPI1_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00000000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x00000000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00000000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00000000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x00000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000000
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 6)
#define STM32_UART4_RX_DMA_CHN              0x00000000
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(1, 7)
#define STM32_UART4_TX_DMA_CHN              0x00000000

#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#if defined(STM32F072xB) || defined(STM32F078xx)
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  768
#define STM32_USB_HAS_BCDR                  TRUE
#else
#define STM32_HAS_USB                       FALSE
#endif
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32F091xC, STM32F098xx.                                                 */
/*===========================================================================*/
#elif defined(STM32F091xC) || defined(STM32F098xx)

/* RCC attributes. */
#define STM32_HAS_HSI48                     TRUE
#define STM32_HAS_HSI_PREDIV                TRUE
#define STM32_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_ADC1_DMA_CHN                  0x00100011

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               14

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_DAC1_CH1_DMA_CHN              0x00000100

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH2_DMA_CHN              0x00001000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE

#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             5

#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA12_CH23_CH12_HANDLER       Vector68
#define STM32_DMA12_CH4567_CH345_HANDLER    Vector6C
#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA12_CH23_CH12_NUMBER        10
#define STM32_DMA12_CH4567_CH345_NUMBER     11

#define STM32_DMA1_CH2_NUMBER               STM32_DMA12_CH23_CH12_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA12_CH23_CH12_NUMBER
#define STM32_DMA2_CH1_NUMBER               STM32_DMA12_CH23_CH12_NUMBER
#define STM32_DMA2_CH2_NUMBER               STM32_DMA12_CH23_CH12_NUMBER
#define STM32_DMA1_CH2_CMASK                0x00000186U
#define STM32_DMA1_CH3_CMASK                0x00000186U
#define STM32_DMA2_CH1_CMASK                0x00000186U
#define STM32_DMA2_CH2_CMASK                0x00000186U

#define STM32_DMA1_CH4_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA2_CH3_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA2_CH4_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA2_CH5_NUMBER               STM32_DMA12_CH4567_CH345_NUMBER
#define STM32_DMA1_CH4_CMASK                0x00000E78U
#define STM32_DMA1_CH5_CMASK                0x00000E78U
#define STM32_DMA1_CH6_CMASK                0x00000E78U
#define STM32_DMA1_CH7_CMASK                0x00000E78U
#define STM32_DMA2_CH3_CMASK                0x00000E78U
#define STM32_DMA2_CH4_CMASK                0x00000E78U
#define STM32_DMA2_CH5_CMASK                0x00000E78U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                32
#define STM32_EXTI_IMR1_MASK                0x7F840000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#define STM32_FLASH_SECTOR_SIZE             2048U
#define STM32_FLASH_LINE_SIZE               2U
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        128 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x02000200
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00200020

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_I2C2_RX_DMA_CHN               0x00020020
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_I2C2_TX_DMA_CHN               0x00002002

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000330
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00003300

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00303000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x03030000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_USART1_RX_DMA_CHN             0x00880888
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART1_TX_DMA_CHN             0x08088088

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_USART2_RX_DMA_CHN             0x00990999
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART2_TX_DMA_CHN             0x09099099

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_USART3_RX_DMA_CHN             0x00AA0AAA
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART3_TX_DMA_CHN             0x0A0AA0AA

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_UART4_RX_DMA_CHN              0x00BB0BBB
#define STM32_UART4_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_UART4_TX_DMA_CHN              0x0B0BB0BB

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_UART5_RX_DMA_CHN              0x00CC0CCC
#define STM32_UART5_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_UART5_TX_DMA_CHN              0x0C0CC0CC

#define STM32_HAS_USART6                    TRUE
#define STM32_USART6_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_USART6_RX_DMA_CHN             0x00DD0DDD
#define STM32_USART6_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_USART6_TX_DMA_CHN             0x0D0DD0DD

#define STM32_HAS_UART7                     TRUE
#define STM32_UART7_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_UART7_RX_DMA_CHN              0x00EE0EEE
#define STM32_UART7_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_UART7_TX_DMA_CHN              0x0E0EE0EE

#define STM32_HAS_UART8                     TRUE
#define STM32_UART8_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_UART8_RX_DMA_CHN              0x00FF0FFF
#define STM32_UART8_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_UART8_TX_DMA_CHN              0x0F0FF0FF

#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       FALSE
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#else
#error "STM32F0xx device not specified"
#endif

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
