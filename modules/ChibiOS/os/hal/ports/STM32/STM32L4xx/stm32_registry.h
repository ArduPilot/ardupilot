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
 * @file    STM32L4xx/stm32_registry.h
 * @brief   STM32L4xx capabilities registry.
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
 * @name    STM32L4xx capabilities
 * @{
 */

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM4_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM4_STOP
#define STM32_DBGMCU_TIM5_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM5_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM8_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM8_STOP_Msk
#define STM32_DBGMCU_TIM15_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM15_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM17_STOP

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   TRUE

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
#define STM32_RTC_ALARM_EXTI                18
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI19_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI20_PRIORITY);       \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI18_PRIORITY);      \
} while (false)

#if defined(STM32L486xx) || defined(STM32L4A6xx) ||                         \
    defined(__DOXYGEN__)
#define STM32_HAS_HASH1                     TRUE
#define STM32_HAS_CRYP1                     TRUE
#else
#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     FALSE
#endif

/*===========================================================================*/
/* STM32L431xx, STM32L432xx.                                                 */
/*===========================================================================*/

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(__DOXYGEN__)

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               TRUE
#define STM32_RCC_PLLSAI1_HAS_P             TRUE
#define STM32_RCC_PLLSAI1_HAS_Q             TRUE
#define STM32_RCC_PLLSAI1_HAS_R             TRUE

#define STM32_RCC_HAS_PLLSAI2               FALSE
#define STM32_RCC_PLLSAI2_HAS_P             FALSE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

/* CAN attributes.*/
#define STM32_CAN_MAX_FILTERS               14
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH1_DMA_CHN              0x00003600

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_DAC1_CH2_DMA_CHN              0x00035000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#if defined(STM32L431xx) || defined(__DOXYGEN__)
#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C2_RX_DMA_CHN               0x00030000
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C2_TX_DMA_CHN               0x00003000
#else
#define STM32_HAS_I2C2                      FALSE
#endif /* defined(STM32L431xx) */

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#if defined(STM32L431xx) || defined(__DOXYGEN__)
#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_SPI2_RX_DMA_CHN               0x00001000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_SPI2_TX_DMA_CHN               0x00010000
#else
#define STM32_HAS_SPI2                      FALSE
#endif /* defined(STM32L431xx) */

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_SPI3_RX_DMA_CHN               0x00000003
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI3_TX_DMA_CHN               0x00000030

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM3                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#if defined(STM32L431xx) || defined(__DOXYGEN__)
#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000200
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000020
#else
#define STM32_HAS_USART3                    FALSE
#endif /* defined(STM32L431xx) */

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#if defined(STM32L431xx) || defined(__DOXYGEN__)
#define STM32_HAS_USB                       FALSE
#else
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  1024
#define STM32_USB_HAS_BCDR                  TRUE
#endif /* defined(STM32L431xx) */

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
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L432xx) */

/*===========================================================================*/
/* STM32L422xx                                                               */
/*===========================================================================*/

#if defined(STM32L422xx) || defined(__DOXYGEN__)

 /* Enabling RTC-related EXTI lines.*/
#define STM32_RTC_ENABLE_ALL_EXTI() do {                                    \
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI) |                       \
                   EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI) |                  \
                   EXTI_MASK1(STM32_RTC_WKUP_EXTI),                         \
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);     \
} while (false)

/* Clearing EXTI interrupts. */
#define STM32_RTC_CLEAR_ALL_EXTI() do {                                     \
  extiClearGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI) |                        \
                  EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI) |                   \
                  EXTI_MASK1(STM32_RTC_WKUP_EXTI));                         \
} while (false)

/* Masks used to preserve state of RTC and TAMP register reserved bits. */
#define STM32_RTC_CR_MASK                   0xE7FFFF7F
#define STM32_RTC_PRER_MASK                 0x007F7FFF
#define STM32_TAMP_CR1_MASK                 0x003C0003
#define STM32_TAMP_CR2_MASK                 0x030300FF
#define STM32_TAMP_FLTCR_MASK               0x000000FF
#define STM32_TAMP_IER_MASK                 0x003C0003

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 FALSE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               FALSE
#define STM32_RCC_PLLSAI1_HAS_P             FALSE
#define STM32_RCC_PLLSAI1_HAS_Q             FALSE
#define STM32_RCC_PLLSAI1_HAS_R             FALSE

#define STM32_RCC_HAS_PLLSAI2               FALSE
#define STM32_RCC_PLLSAI2_HAS_P             FALSE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC2_DMA_CHN                  0x00000000

#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

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
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
/* TODO à verifier */
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIODEN |          \
                                             RCC_AHB2ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM3                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  1024
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
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L422xx) */

/*===========================================================================*/
/* STM32L433xx, STM32L443xx.                                                 */
/*===========================================================================*/

#if defined(STM32L433xx) || defined(STM32L443xx) || defined(__DOXYGEN__)

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               TRUE
#define STM32_RCC_PLLSAI1_HAS_P             TRUE
#define STM32_RCC_PLLSAI1_HAS_Q             TRUE
#define STM32_RCC_PLLSAI1_HAS_R             TRUE

#define STM32_RCC_HAS_PLLSAI2               FALSE
#define STM32_RCC_PLLSAI2_HAS_P             FALSE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

/* CAN attributes.*/
#define STM32_CAN_MAX_FILTERS               14
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH1_DMA_CHN              0x00003600

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_DAC1_CH2_DMA_CHN              0x00035000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIODEN |          \
                                             RCC_AHB2ENR_GPIOEEN |          \
                                             RCC_AHB2ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C2_RX_DMA_CHN               0x00030000
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C2_TX_DMA_CHN               0x00003000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC1_DMA_CHN            0x00077000

#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_SPI2_RX_DMA_CHN               0x00001000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_SPI2_TX_DMA_CHN               0x00010000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_SPI3_RX_DMA_CHN               0x00000003
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI3_TX_DMA_CHN               0x00000030

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM3                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000200
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000020

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  1024
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
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L443xx) */

/*===========================================================================*/
/* STM32L452xx.                                                              */
/*===========================================================================*/

#if defined(STM32L452xx) || defined(__DOXYGEN__)

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               TRUE
#define STM32_RCC_PLLSAI1_HAS_P             TRUE
#define STM32_RCC_PLLSAI1_HAS_Q             TRUE
#define STM32_RCC_PLLSAI1_HAS_R             TRUE

#define STM32_RCC_HAS_PLLSAI2               FALSE
#define STM32_RCC_PLLSAI2_HAS_P             FALSE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

/* CAN attributes.*/
#define STM32_CAN_MAX_FILTERS               14
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH1_DMA_CHN              0x00003600

#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIODEN |          \
                                             RCC_AHB2ENR_GPIOEEN |          \
                                             RCC_AHB2ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C2_RX_DMA_CHN               0x00030000
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C2_TX_DMA_CHN               0x00003000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_I2C4_RX_DMA_CHN               0x00000000
#define STM32_I2C4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_I2C4_TX_DMA_CHN               0x00000000

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC1_DMA_CHN            0x00077000

#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_SPI2_RX_DMA_CHN               0x00001000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_SPI2_TX_DMA_CHN               0x00010000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_SPI3_RX_DMA_CHN               0x00000003
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI3_TX_DMA_CHN               0x00000030

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM7                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000200
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000020

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_UART4_RX_DMA_CHN              0x00020000
#define STM32_UART4_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_UART4_TX_DMA_CHN              0x00000200

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  1024
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
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L452xx) */

/*===========================================================================*/
/* STM32L475xx, STM32L476xx, STM32L486xx.                                    */
/*===========================================================================*/

#if defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L486xx)

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               TRUE
#define STM32_RCC_PLLSAI1_HAS_P             TRUE
#define STM32_RCC_PLLSAI1_HAS_Q             TRUE
#define STM32_RCC_PLLSAI1_HAS_R             TRUE

#define STM32_RCC_HAS_PLLSAI2               TRUE
#define STM32_RCC_PLLSAI2_HAS_P             TRUE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC2_DMA_CHN                  0x00000000

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_ADC3_DMA_CHN                  0x00000000

#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

/* CAN attributes.*/
#define STM32_CAN_MAX_FILTERS               14
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH1_DMA_CHN              0x00003600

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_DAC1_CH2_DMA_CHN              0x00035000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         2
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

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
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIODEN |          \
                                             RCC_AHB2ENR_GPIOEEN |          \
                                             RCC_AHB2ENR_GPIOFEN |          \
                                             RCC_AHB2ENR_GPIOGEN |          \
                                             RCC_AHB2ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C2_RX_DMA_CHN               0x00030000
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C2_TX_DMA_CHN               0x00003000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC1_DMA_CHN            0x00077000

#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_SPI2_RX_DMA_CHN               0x00001000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_SPI2_TX_DMA_CHN               0x00010000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_SPI3_RX_DMA_CHN               0x00000003
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI3_TX_DMA_CHN               0x00000030

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
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

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE
#define STM32_TIM15_CHANNELS                2

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000200
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000020

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 5)
#define STM32_UART4_RX_DMA_CHN              0x00020000
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 3)
#define STM32_UART4_TX_DMA_CHN              0x00000200

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 2)
#define STM32_UART5_RX_DMA_CHN              0x00000020
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 1)
#define STM32_UART5_TX_DMA_CHN              0x00000002

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_OTG_SEQUENCE_WORKAROUND
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      FALSE
#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L476xx) */

/*===========================================================================*/
/* STM32L496xx, STM32L4A6xx.                                                 */
/*===========================================================================*/

#if defined(STM32L496xx) || defined(STM32L4A6xx)

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               TRUE
#define STM32_RCC_PLLSAI1_HAS_P             TRUE
#define STM32_RCC_PLLSAI1_HAS_Q             TRUE
#define STM32_RCC_PLLSAI1_HAS_R             TRUE

#define STM32_RCC_HAS_PLLSAI2               TRUE
#define STM32_RCC_PLLSAI2_HAS_P             TRUE
#define STM32_RCC_PLLSAI2_HAS_Q             FALSE
#define STM32_RCC_PLLSAI2_HAS_R             TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC1_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 1) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_ADC1_DMA_CHN                  0x00000000

#define STM32_HAS_ADC2                      TRUE
#define STM32_ADC2_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_ADC2_DMA_CHN                  0x00000000

#define STM32_HAS_ADC3                      TRUE
#define STM32_ADC3_DMA_MSK                  (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_ADC3_DMA_CHN                  0x00000000

#define STM32_HAS_ADC4                      FALSE
#define STM32_HAS_ADC5                      FALSE

/* CAN attributes.*/
#define STM32_CAN_MAX_FILTERS               28
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_DAC1_CH1_DMA_CHN              0x00003600

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4)|\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_DAC1_CH2_DMA_CHN              0x00035000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                40
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         2
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOI                     TRUE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN |          \
                                             RCC_AHB2ENR_GPIODEN |          \
                                             RCC_AHB2ENR_GPIOEEN |          \
                                             RCC_AHB2ENR_GPIOFEN |          \
                                             RCC_AHB2ENR_GPIOGEN |          \
                                             RCC_AHB2ENR_GPIOHEN |          \
                                             RCC_AHB2ENR_GPIOIEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 7) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_I2C1_RX_DMA_CHN               0x03500000
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 6) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_I2C1_TX_DMA_CHN               0x05300000

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C2_RX_DMA_CHN               0x00030000
#define STM32_I2C2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C2_TX_DMA_CHN               0x00003000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_I2C3_RX_DMA_CHN               0x00000300
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_I2C3_TX_DMA_CHN               0x00000030

#define STM32_HAS_I2C4                      TRUE
#define STM32_I2C4_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_I2C4_RX_DMA_CHN               0x00000000
#define STM32_I2C4_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_I2C4_TX_DMA_CHN               0x00000000

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_QUADSPI1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_QUADSPI1_DMA_CHN              0x03050000

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_SDC_SDMMC1_DMA_MSK            (STM32_DMA_STREAM_ID_MSK(2, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 5))
#define STM32_SDC_SDMMC1_DMA_CHN            0x00077000

#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 3))
#define STM32_SPI1_RX_DMA_CHN               0x00000410
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 4))
#define STM32_SPI1_TX_DMA_CHN               0x00004100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_SPI2_RX_DMA_CHN               0x00001000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_SPI2_TX_DMA_CHN               0x00010000

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE
#define STM32_SPI3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 1))
#define STM32_SPI3_RX_DMA_CHN               0x00000003
#define STM32_SPI3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(2, 2))
#define STM32_SPI3_TX_DMA_CHN               0x00000030

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
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

#define STM32_HAS_TIM15                     TRUE
#define STM32_TIM15_IS_32BITS               FALSE

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 7))
#define STM32_USART1_RX_DMA_CHN             0x02020000
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(2, 6))
#define STM32_USART1_TX_DMA_CHN             0x00202000

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00200000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x02000000

#define STM32_HAS_USART3                    TRUE
#define STM32_USART3_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_USART3_RX_DMA_CHN             0x00000200
#define STM32_USART3_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_USART3_TX_DMA_CHN             0x00000020

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 5)
#define STM32_UART4_RX_DMA_CHN              0x00020000
#define STM32_UART4_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 3)
#define STM32_UART4_TX_DMA_CHN              0x00000200

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 2)
#define STM32_UART5_RX_DMA_CHN              0x00000020
#define STM32_UART5_TX_DMA_MSK              STM32_DMA_STREAM_ID_MSK(2, 1)
#define STM32_UART5_TX_DMA_CHN              0x00000002

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                5

#define STM32_HAS_OTG2                      FALSE
#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#endif /* defined(STM32L496xx) */

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
