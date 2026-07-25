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
 * @file    STM32L0xx/stm32_registry.h
 * @brief   STM32L0xx capabilities registry.
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
 * @name    STM32L0xx capabilities
 * @{
 */

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM21_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM21_STOP
#define STM32_DBGMCU_TIM22_STOP()           DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM22_STOP

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
#define STM32_RTC_STORAGE_SIZE              20
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_PRIORITY)

/*===========================================================================*/
/* STM32L011xx.                                                              */
/*===========================================================================*/
#if defined(STM32L011xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000018U
#define STM32_DMA1_CH5_CMASK                0x00000018U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     FALSE
#define STM32_HAS_GPIOE                     FALSE
#define STM32_HAS_GPIOF                     FALSE
#define STM32_HAS_GPIOG                     FALSE
#define STM32_HAS_GPIOH                     FALSE
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                FALSE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM3                      FALSE
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
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART1                    FALSE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

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
/* STM32L031xx.                                                              */
/*===========================================================================*/
#elif defined(STM32L031xx)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                FALSE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM22                     TRUE
#define STM32_TIM22_IS_32BITS               FALSE
#define STM32_TIM22_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM3                      FALSE
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
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART1                    FALSE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

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
/* STM32L051xx, STM32L061xx.                                                 */
/*===========================================================================*/
#elif defined(STM32L051xx) || defined(STM32L061xx)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00070000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00007000

#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           FALSE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00202000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x02020000

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                FALSE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM22                     TRUE
#define STM32_TIM22_IS_32BITS               FALSE
#define STM32_TIM22_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM3                      FALSE
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
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00030300
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00003030

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

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
/* STM32L052xx, STM32L062xx, STM32L053xx, STM32L063xx.                       */
/*===========================================================================*/
#elif defined(STM32L052xx) || defined(STM32L062xx) ||                       \
      defined(STM32L053xx) || defined(STM32L063xx)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_DAC1_CH1_DMA_CHN              0x00000090

#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  TRUE
#define STM32_HAS_DAC2_CH2                  TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00070000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00007000

#define STM32_HAS_I2C3                      FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00202000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x02020000

#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                FALSE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM22                     TRUE
#define STM32_TIM22_IS_32BITS               FALSE
#define STM32_TIM22_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM3                      FALSE
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
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00030300
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00003030

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

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
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/*===========================================================================*/
/* STM32L071xx.                                                              */
/*===========================================================================*/
#elif defined(STM32L071xx)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_DAC1_CH1_DMA_CHN              0x00000090
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_DAC1_CH2_DMA_CHN              0x0000F000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOEEN |           \
                                             RCC_IOPENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00070000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00007000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C3_RX_DMA_CHN               0x000E0E00
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C3_TX_DMA_CHN               0x0000E0E0

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00202000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x02020000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM22                     TRUE
#define STM32_TIM22_IS_32BITS               FALSE
#define STM32_TIM22_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00030300
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00003030

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_UART4_RX_DMA_CHN              0x00C000C0
#define STM32_UART4_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_UART4_TX_DMA_CHN              0x0C000C00

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_UART5_RX_DMA_CHN              0x00D000D0
#define STM32_UART5_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_UART5_TX_DMA_CHN              0x0D000D00

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

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
/* STM32L072xx, STM32L073xx.                                                 */
/*===========================================================================*/
#elif defined(STM32L072xx) || defined(STM32L073xx)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
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
#define STM32_DAC1_CH1_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_DAC1_CH1_DMA_CHN              0x00000090
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_DAC1_CH2_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_DAC1_CH2_DMA_CHN              0x0000F000

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           FALSE
#define STM32_DMA_SUPPORTS_CSELR            TRUE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000078U
#define STM32_DMA1_CH5_CMASK                0x00000078U
#define STM32_DMA1_CH6_CMASK                0x00000078U
#define STM32_DMA1_CH7_CMASK                0x00000078U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0xFF840000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOEEN |           \
                                             RCC_IOPENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_I2C1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_I2C1_RX_DMA_CHN               0x06000600
#define STM32_I2C1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_I2C1_TX_DMA_CHN               0x00600060

#define STM32_HAS_I2C2                      TRUE
#define STM32_I2C2_RX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 5)
#define STM32_I2C2_RX_DMA_CHN               0x00070000
#define STM32_I2C2_TX_DMA_MSK               STM32_DMA_STREAM_ID_MSK(1, 4)
#define STM32_I2C2_TX_DMA_CHN               0x00007000

#define STM32_HAS_I2C3                      TRUE
#define STM32_I2C3_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_I2C3_RX_DMA_CHN               0x000E0E00
#define STM32_I2C3_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_I2C3_TX_DMA_CHN               0x0000E0E0

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 2))
#define STM32_SPI1_RX_DMA_CHN               0x00000010
#define STM32_SPI1_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 3))
#define STM32_SPI1_TX_DMA_CHN               0x00000100

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_RX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_SPI2_RX_DMA_CHN               0x00202000
#define STM32_SPI2_TX_DMA_MSK               (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_SPI2_TX_DMA_CHN               0x02020000

#define STM32_HAS_SPI3                      FALSE
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

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM21                     TRUE
#define STM32_TIM21_IS_32BITS               FALSE
#define STM32_TIM21_CHANNELS                2

#define STM32_HAS_TIM22                     TRUE
#define STM32_TIM22_IS_32BITS               FALSE
#define STM32_TIM22_CHANNELS                2

#define STM32_HAS_TIM1                      FALSE
#define STM32_HAS_TIM4                      FALSE
#define STM32_HAS_TIM5                      FALSE
#define STM32_HAS_TIM8                      FALSE
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
#define STM32_HAS_TIM12                     FALSE
#define STM32_HAS_TIM13                     FALSE
#define STM32_HAS_TIM14                     FALSE
#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_USART1_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 5))
#define STM32_USART1_RX_DMA_CHN             0x00030300
#define STM32_USART1_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 4))
#define STM32_USART1_TX_DMA_CHN             0x00003030

#define STM32_HAS_USART2                    TRUE
#define STM32_USART2_RX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 5) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_USART2_RX_DMA_CHN             0x00440000
#define STM32_USART2_TX_DMA_MSK             (STM32_DMA_STREAM_ID_MSK(1, 4) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_USART2_TX_DMA_CHN             0x04004000

#define STM32_HAS_UART4                     TRUE
#define STM32_UART4_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_UART4_RX_DMA_CHN              0x00C000C0
#define STM32_UART4_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_UART4_TX_DMA_CHN              0x0C000C00

#define STM32_HAS_UART5                     TRUE
#define STM32_UART5_RX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 2) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 6))
#define STM32_UART5_RX_DMA_CHN              0x00D000D0
#define STM32_UART5_TX_DMA_MSK              (STM32_DMA_STREAM_ID_MSK(1, 3) |\
                                             STM32_DMA_STREAM_ID_MSK(1, 7))
#define STM32_UART5_TX_DMA_CHN              0x0D000D00

#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USART3                    FALSE
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
#define STM32_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

#else
#error "STM32L0xx device not specified"
#endif

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
