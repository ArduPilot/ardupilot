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
 * @file    STM32L1xx/stm32_registry.h
 * @brief   STM32L1xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

#if defined(STM32L100xB) || defined(STM32L151xB) || defined(STM32L152xB)
#define STM32L1XX_PROD_CAT      1

#elif defined(STM32L100xBA) || defined(STM32L151xBA) || defined(STM32L152xBA)
#define STM32L1XX_PROD_CAT      2

#elif defined(STM32L100xC)  || defined(STM32L151xC)   ||                    \
      defined(STM32L151xCA) || defined(STM32L152xC)   ||                    \
      defined(STM32L152xCA) || defined(STM32L162xC)   ||                    \
      defined(STM32L162xCA)
#define STM32L1XX_PROD_CAT      3

#elif defined(STM32L151xD)  || defined(STM32L152xD)   ||                    \
      defined(STM32L162xD)
#define STM32L1XX_PROD_CAT      4

#elif defined(STM32L151xE)  || defined (STM32L152xE)  ||                    \
      defined(STM32L162xE)
#define STM32L1XX_PROD_CAT      5

#elif defined(STM32L151xDX) || defined (STM32L152xDX) ||                    \
      defined(STM32L162xDX)
#define STM32L1XX_PROD_CAT      6

#else
#error "STM32L1xx device not specified"
#endif

#if defined(STM32L100xB) || defined(STM32L100xBA) ||  defined(STM32L100xC)
#define STM32L1XX_VALUE_LINE                TRUE
#else
#define STM32L1XX_VALUE_LINE                FALSE
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM4_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP
#define STM32_DBGMCU_TIM5_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM9_STOP()            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM9_STOP
#define STM32_DBGMCU_TIM10_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM10_STOP
#define STM32_DBGMCU_TIM11_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM11_STOP

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   FALSE

/**
 * @name    STM32L1xx capabilities
 * @{
 */
/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_DAC_DAC1_CH1_DMA_STREAM       STM32_DMA_STREAM_ID(1, 2)

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC_DAC1_CH2_DMA_STREAM       STM32_DMA_STREAM_ID(1, 3)

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  FALSE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            FALSE

#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA1_CH1_HANDLER              Vector6C
#define STM32_DMA1_CH2_HANDLER              Vector70
#define STM32_DMA1_CH3_HANDLER              Vector74
#define STM32_DMA1_CH4_HANDLER              Vector78
#define STM32_DMA1_CH5_HANDLER              Vector7C
#define STM32_DMA1_CH6_HANDLER              Vector80
#define STM32_DMA1_CH7_HANDLER              Vector84
#define STM32_DMA1_CH1_NUMBER               11
#define STM32_DMA1_CH2_NUMBER               12
#define STM32_DMA1_CH3_NUMBER               13
#define STM32_DMA1_CH4_NUMBER               14
#define STM32_DMA1_CH5_NUMBER               15
#define STM32_DMA1_CH6_NUMBER               16
#define STM32_DMA1_CH7_NUMBER               17

#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    defined(__DOXYGEN__)
#define STM32_DMA2_NUM_CHANNELS             0
#else
#define STM32_DMA2_NUM_CHANNELS             5
#define STM32_DMA2_CH1_HANDLER              Vector108
#define STM32_DMA2_CH2_HANDLER              Vector10C
#define STM32_DMA2_CH3_HANDLER              Vector110
#define STM32_DMA2_CH4_HANDLER              Vector114
#define STM32_DMA2_CH5_HANDLER              Vector118
#define STM32_DMA2_CH1_NUMBER               50
#define STM32_DMA2_CH2_NUMBER               51
#define STM32_DMA2_CH3_NUMBER               52
#define STM32_DMA2_CH4_NUMBER               53
#define STM32_DMA2_CH5_NUMBER               54
#endif

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    defined(__DOXYGEN__)
#define STM32_EXTI_NUM_LINES                23
#else
#define STM32_EXTI_NUM_LINES                24
#endif
#define STM32_EXTI_IMR1_MASK                0x00000000U

#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    (STM32L1XX_PROD_CAT == 3) || defined(__DOXYGEN__)
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
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOEEN |           \
                                             RCC_AHBENR_GPIOHEN)
#else
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
#define STM32_GPIO_EN_MASK                  (RCC_AHBENR_GPIOAEN |           \
                                             RCC_AHBENR_GPIOBEN |           \
                                             RCC_AHBENR_GPIOCEN |           \
                                             RCC_AHBENR_GPIODEN |           \
                                             RCC_AHBENR_GPIOEEN |           \
                                             RCC_AHBENR_GPIOFEN |           \
                                             RCC_AHBENR_GPIOGEN |           \
                                             RCC_AHBENR_GPIOHEN)
#endif

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C_I2C1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 7)
#define STM32_I2C_I2C1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 6)

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C_I2C2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 5)
#define STM32_I2C_I2C2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#if (STM32L1XX_PROD_CAT == 1) || defined(__DOXYGEN__)
#define STM32_RTC_HAS_SUBSECONDS            FALSE
#else
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#endif
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#if STM32L1XX_VALUE_LINE || defined(__DOXYGEN__)
#define STM32_RTC_STORAGE_SIZE              20
#elif (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2)
#define STM32_RTC_STORAGE_SIZE              80
#else
#define STM32_RTC_STORAGE_SIZE              128
#endif
#define STM32_RTC_TAMP_STAMP_HANDLER        Vector48
#define STM32_RTC_WKUP_HANDLER              Vector4C
#define STM32_RTC_ALARM_HANDLER             VectorE4
#define STM32_RTC_TAMP_STAMP_NUMBER         3
#define STM32_RTC_WKUP_NUMBER               1
#define STM32_RTC_ALARM_NUMBER              2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI19_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI20_PRIORITY);       \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI18_PRIORITY);      \
} while (false)

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 2)
#define STM32_SPI_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 3)

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           FALSE
#define STM32_SPI_SPI2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)
#define STM32_SPI_SPI2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 5)

#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    defined(__DOXYGEN__)
#define STM32_HAS_SPI3                      FALSE
#else
#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           FALSE
#define STM32_SPI_SPI3_RX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 1)
#define STM32_SPI_SPI3_TX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 2)
#endif

#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                FALSE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    defined(__DOXYGEN__)
#define STM32_HAS_TIM5                      FALSE
#else
#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4
#endif

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
#define STM32_TIM10_CHANNELS                2

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
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
#define STM32_UART_USART1_RX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 5)
#define STM32_UART_USART1_TX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 4)

#define STM32_HAS_USART2                    TRUE
#define STM32_UART_USART2_RX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 6)
#define STM32_UART_USART2_TX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 7)

#define STM32_HAS_USART3                    TRUE
#define STM32_UART_USART3_RX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 3)
#define STM32_UART_USART3_TX_DMA_STREAM     STM32_DMA_STREAM_ID(1, 2)

#if (STM32L1XX_PROD_CAT == 1) || (STM32L1XX_PROD_CAT == 2) ||               \
    (STM32L1XX_PROD_CAT == 3) || defined(__DOXYGEN__)
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#else
#define STM32_HAS_UART4                     TRUE
#define STM32_UART_UART4_RX_DMA_STREAM      STM32_DMA_STREAM_ID(2, 3)
#define STM32_UART_UART4_TX_DMA_STREAM      STM32_DMA_STREAM_ID(2, 5)

#define STM32_HAS_UART5                     TRUE
#define STM32_UART_UART5_RX_DMA_STREAM      STM32_DMA_STREAM_ID(2, 2)
#define STM32_UART_UART5_TX_DMA_STREAM      STM32_DMA_STREAM_ID(2, 1)
#endif

#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        FALSE
#define STM32_USB_PMA_SIZE                  512
#define STM32_USB_HAS_BCDR                  FALSE
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

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
