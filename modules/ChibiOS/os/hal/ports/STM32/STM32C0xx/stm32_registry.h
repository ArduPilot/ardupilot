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
 * @file    STM32C0xx/stm32_registry.h
 * @brief   STM32C0xx capabilities registry.
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
 * @name    STM32C0xx capabilities
 * @{
 */

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define STM32_RTC_NUM_ALARMS                1
#define STM32_RTC_STORAGE_SIZE              0
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_EVENT_RTC_EXTI            19
#if !defined(STM32_RTC_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_RTC_IRQ_PRIORITY              STM32_IRQ_EXTI4_15_PRIORITY
#endif
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER,                                 \
                   STM32_RTC_IRQ_PRIORITY);                                 \
} while (false)

 /* Enabling RTC-related EXTI lines.*/
#define STM32_RTC_ENABLE_ALL_EXTI() do {                                    \
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_EVENT_RTC_EXTI),                    \
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);     \
} while (false)

/* Clearing EXTI interrupts. */
#define STM32_RTC_CLEAR_ALL_EXTI() do {                                     \
  extiClearGroup1(EXTI_MASK1(STM32_RTC_EVENT_RTC_EXTI));                    \
} while (false)

/* Masks used to preserve state of RTC and TAMP register reserved bits. */
#define STM32_RTC_CR_MASK                   0xE7FFFF7F
#define STM32_RTC_PRER_MASK                 0x007F7FFF

/* RCC attributes (common).*/
#define STM32_RCC_HAS_HSI16                 FALSE
#define STM32_RCC_HAS_MSI                   FALSE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE
#define STM32_RCC_HAS_HSI48                 TRUE

#define STM32_RCC_HAS_PLL                   FALSE
#define STM32_RCC_HAS_PLLSAI1               FALSE
#define STM32_RCC_HAS_PLLSAI2               FALSE

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM14_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM14_STOP
#define STM32_DBGMCU_TIM15_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM15_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM17_STOP

/*===========================================================================*/
/* STM32C011xx, STM32C031xx.                                                 */
/*===========================================================================*/

#if defined(STM32C011xx) || defined(STM32C031xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_HAS_FDCAN1                    FALSE
#define STM32_HAS_FDCAN2                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             3
#define STM32_DMA2_NUM_CHANNELS             0

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                16
#define STM32_EXTI_IMR1_MASK                0xFF800000U

/* GPIO attributes.*/
#if defined(STM32C031xx) || defined(__DOXYGEN__)
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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOFEN)
#else
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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIOFEN)
#endif

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* OTG/USB attributes.*/
#define STM32_HAS_USB1                       FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSIUSB48              FALSE
#define STM32_RCC_HAS_SYSDIV                FALSE
#define STM32_RCC_HAS_MCODIVEXT             FALSE
#define STM32_RCC_HAS_FDCAN1SEL             FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      FALSE
#define STM32_HAS_SPI3                      FALSE
#define STM32_HAS_SPI4                      FALSE
#define STM32_HAS_SPI5                      FALSE
#define STM32_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              6

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 6

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

#define STM32_HAS_TIM2                      FALSE
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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#define STM32_HAS_LPUART2                   FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32C011xx) || defined(STM32C031xx) */

/*===========================================================================*/
/* STM32C051xx.                                                              */
/*===========================================================================*/

#if defined(STM32C051xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_HAS_FDCAN1                    FALSE
#define STM32_HAS_FDCAN2                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                16
#define STM32_EXTI_IMR1_MASK                0xFF800000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* OTG/USB attributes.*/
//#define STM32_HAS_OTG1                      FALSE
//#define STM32_HAS_OTG2                      FALSE
//#define STM32_HAS_USB                       FALSE
#define STM32_HAS_USB1                       FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSIUSB48              FALSE
#define STM32_RCC_HAS_SYSDIV                TRUE
#define STM32_RCC_HAS_MCODIVEXT             FALSE
#define STM32_RCC_HAS_FDCAN1SEL             FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_HAS_SPI2                      TRUE

#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI3                      FALSE
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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#define STM32_HAS_LPUART2                   FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32C051xx) */

/*===========================================================================*/
/* STM32C071xx.                                                              */
/*===========================================================================*/

#if defined(STM32C071xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_HAS_FDCAN1                    FALSE
#define STM32_HAS_FDCAN2                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             5
#define STM32_DMA2_NUM_CHANNELS             0

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                16
#define STM32_EXTI_IMR1_MASK                0xFF800000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* OTG/USB attributes.*/
//#define STM32_HAS_OTG1                      FALSE
//#define STM32_HAS_OTG2                      FALSE

#define STM32_HAS_USB1                      TRUE
//#define STM32_HAS_USB                       TRUE
#define STM32_USB_ACCESS_SCHEME_2x16        TRUE
#define STM32_USB_PMA_SIZE                  2048
#define STM32_USB_HAS_BCDR                  TRUE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSIUSB48              TRUE
#define STM32_RCC_HAS_SYSDIV                TRUE
#define STM32_RCC_HAS_MCODIVEXT             FALSE
#define STM32_RCC_HAS_FDCAN1SEL             FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_HAS_SPI2                      TRUE

#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI3                      FALSE
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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    FALSE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#define STM32_HAS_LPUART2                   FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32C071xx) */

/*===========================================================================*/
/* STM32C091xx, STM32C092xx.                                                 */
/*===========================================================================*/

#if defined(STM32C091xx) || defined(STM32C092xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#if defined(STM32C091xx)
#define STM32_HAS_FDCAN1                    FALSE
#else
#define STM32_HAS_FDCAN1                    TRUE
#endif
#define STM32_HAS_FDCAN2                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  FALSE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             0

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                16
#define STM32_EXTI_IMR1_MASK                0xFF800000U

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* OTG/USB attributes.*/
//#define STM32_HAS_OTG1                      FALSE
//#define STM32_HAS_OTG2                      FALSE
//#define STM32_HAS_USB                       FALSE
#define STM32_HAS_USB1                       FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSIUSB48              FALSE
#define STM32_RCC_HAS_SYSDIV                TRUE
#define STM32_RCC_HAS_MCODIVEXT             FALSE
#if defined(STM32C091xx)
#define STM32_RCC_HAS_FDCAN1SEL             FALSE
#else
#define STM32_RCC_HAS_FDCAN1SEL             TRUE
#endif

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_HAS_SPI2                      TRUE

#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI3                      FALSE
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
#define STM32_HAS_TIM6                      FALSE
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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               FALSE
#define STM32_USART3_HAS_FIFO               FALSE
#define STM32_UART4_HAS_FIFO                FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART1                   FALSE
#define STM32_HAS_LPUART2                   FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32C091xx) || defined(STM32C092xx) */

#endif /* STM32_REGISTRY_H */

/** @} */
