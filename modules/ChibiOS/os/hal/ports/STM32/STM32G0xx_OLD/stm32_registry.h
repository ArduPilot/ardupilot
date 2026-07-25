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
 * @file    STM32G0xx/stm32_registry.h
 * @brief   STM32G0xx capabilities registry.
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
 * @name    STM32G0xx capabilities
 * @{
 */

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* RTC and TAMP attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              20
#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_EVENT_RTC_EXTI            19
#define STM32_RTC_EVENT_TAMP_EXTI           21
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER,                                 \
                   STM32_IRQ_EXTI1921_PRIORITY);                            \
} while (false)

 /* Enabling RTC-related EXTI lines.*/
#define STM32_RTC_ENABLE_ALL_EXTI() do {                                    \
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_EVENT_RTC_EXTI) |                   \
                   EXTI_MASK1(STM32_RTC_EVENT_TAMP_EXTI),                   \
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);     \
} while (false)

/* Clearing EXTI interrupts. */
#define STM32_RTC_CLEAR_ALL_EXTI() do {                                     \
  extiClearGroup1(EXTI_MASK1(STM32_RTC_EVENT_RTC_EXTI) |                    \
                  EXTI_MASK1(STM32_RTC_EVENT_TAMP_EXTI));                   \
} while (false)

/* Masks used to preserve state of RTC and TAMP register reserved bits. */
#define STM32_RTC_CR_MASK                   0xE7FFFF7F
#define STM32_RTC_PRER_MASK                 0x007F7FFF
#define STM32_TAMP_CR1_MASK                 0x003C0003
#define STM32_TAMP_CR2_MASK                 0x030300FF
#define STM32_TAMP_FLTCR_MASK               0x000000FF
#define STM32_TAMP_IER_MASK                 0x003C0003

#if defined(STM32G041xx) || defined(STM32G081xx) ||                         \
    defined(STM32G0C1xx) || defined(__DOXYGEN__)
#define STM32_HAS_RNG1                      TRUE
#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     TRUE
#else
#define STM32_HAS_RNG1                      FALSE
#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     FALSE
#endif

/* RCC attributes (common).*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_MSI                   FALSE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

#define STM32_RCC_HAS_PLLSAI1               FALSE
#define STM32_RCC_HAS_PLLSAI2               FALSE

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM4_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM4_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBG->APBFZ1 |= DBG_APB_FZ1_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM14_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM14_STOP
#define STM32_DBGMCU_TIM15_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM15_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBG->APBFZ2 |= DBG_APB_FZ2_DBG_TIM17_STOP

/*===========================================================================*/
/* STM32G070xx.                                                              */
/*===========================================================================*/

#if defined(STM32G070xx) || defined(__DOXYGEN__)

/* Errata attributes.*/
#define STM32_HAS_TIM1617_ERRATA            TRUE

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
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                16
#define STM32_EXTI_IMR1_MASK                0xFFD00000U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1

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

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* PWR attributes.*/
#define STM32_PWR_HAS_CR2                   FALSE
#define STM32_PWR_HAS_VDDIO2                FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_PLL_HAS_Q                 FALSE
#define STM32_RCC_HAS_CCIPR2                FALSE
#define STM32_RCC_HAS_MCO2                  FALSE
#define STM32_RCC_HAS_MCOSEL_EXT            FALSE
#define STM32_RCC_HAS_MCOPRE_EXT            FALSE
#define STM32_RCC_HAS_MCOSEL_PLLP           FALSE
#define STM32_RCC_HAS_MCOSEL_PLLQ           FALSE
#define STM32_RCC_HAS_MCOSEL_RTCCLK         FALSE
#define STM32_RCC_HAS_MCOSEL_RTCWKP         FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      TRUE
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

#define STM32_HAS_LPTIM1                    TRUE
#define STM32_HAS_LPTIM2                    TRUE

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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               TRUE
#define STM32_USART3_HAS_FIFO               FALSE
#define STM32_UART4_HAS_FIFO                FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART2                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB1                      FALSE
#define STM32_HAS_UCPD1                     FALSE
#define STM32_HAS_UCPD2                     FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      FALSE

#endif /* defined(STM32G070xx) */

/*===========================================================================*/
/* STM32G031xx, STM32G041xx.                                                 */
/*===========================================================================*/

#if defined(STM32G031xx) || defined(STM32G041xx)

/* Errata attributes.*/
#define STM32_HAS_TIM1617_ERRATA            TRUE

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
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000018U
#define STM32_DMA1_CH5_CMASK                0x00000018U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                36
#define STM32_EXTI_IMR1_MASK                0xFFD00000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFFU

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1

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

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* PWR attributes.*/
#define STM32_PWR_HAS_CR2                   TRUE
#define STM32_PWR_HAS_VDDIO2                FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_HAS_CCIPR2                FALSE
#define STM32_RCC_HAS_MCO2                  FALSE
#define STM32_RCC_HAS_MCOSEL_EXT            FALSE
#define STM32_RCC_HAS_MCOPRE_EXT            FALSE
#define STM32_RCC_HAS_MCOSEL_PLLP           FALSE
#define STM32_RCC_HAS_MCOSEL_PLLQ           FALSE
#define STM32_RCC_HAS_MCOSEL_RTCCLK         FALSE
#define STM32_RCC_HAS_MCOSEL_RTCWKP         FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      TRUE
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

#define STM32_HAS_LPTIM1                    TRUE
#define STM32_HAS_LPTIM2                    TRUE

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
#define STM32_HAS_LPUART1                   TRUE
#define STM32_HAS_LPUART2                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB1                      FALSE
#define STM32_HAS_UCPD1                     FALSE
#define STM32_HAS_UCPD2                     FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      FALSE

#endif /* defined(STM32G031xx) || defined(STM32G041xx) */

/*===========================================================================*/
/* STM32G071xx, STM32G081xx.                                                 */
/*===========================================================================*/

#if defined(STM32G071xx) || defined(STM32G081xx)

/* Errata attributes.*/
#define STM32_HAS_TIM1617_ERRATA            TRUE

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
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE
#define STM32_DAC_HAS_MCR                   TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
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
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                36
#define STM32_EXTI_IMR1_MASK                0xFFD00000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFFU

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1

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

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* PWR attributes.*/
#define STM32_PWR_HAS_CR2                   TRUE
#define STM32_PWR_HAS_VDDIO2                FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_HAS_CCIPR2                FALSE
#define STM32_RCC_HAS_MCO2                  FALSE
#define STM32_RCC_HAS_MCOSEL_EXT            FALSE
#define STM32_RCC_HAS_MCOPRE_EXT            FALSE
#define STM32_RCC_HAS_MCOSEL_PLLP           FALSE
#define STM32_RCC_HAS_MCOSEL_PLLQ           FALSE
#define STM32_RCC_HAS_MCOSEL_RTCCLK         FALSE
#define STM32_RCC_HAS_MCOSEL_RTCWKP         FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      TRUE
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

#define STM32_HAS_LPTIM1                    TRUE
#define STM32_HAS_LPTIM2                    TRUE

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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               TRUE
#define STM32_USART3_HAS_FIFO               FALSE
#define STM32_UART4_HAS_FIFO                FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_LPUART2                   FALSE

/* USB attributes.*/
#define STM32_HAS_USB1                      FALSE
#define STM32_HAS_UCPD1                     FALSE
#define STM32_HAS_UCPD2                     FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      FALSE

#endif /* defined(STM32G071xx) || defined(STM32G081xx) */

/*===========================================================================*/
/* STM32G0B1xx, STM32G0C1xx.                                                 */
/*===========================================================================*/

#if defined(STM32G0B1xx) || defined(STM32G0C1xx)

/* Errata attributes.*/
#define STM32_HAS_TIM1617_ERRATA            FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             5
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000FF8U
#define STM32_DMA1_CH5_CMASK                0x00000FF8U
#define STM32_DMA1_CH6_CMASK                0x00000FF8U
#define STM32_DMA1_CH7_CMASK                0x00000FF8U
#define STM32_DMA2_CH1_CMASK                0x00000FF8U
#define STM32_DMA2_CH2_CMASK                0x00000FF8U
#define STM32_DMA2_CH3_CMASK                0x00000FF8U
#define STM32_DMA2_CH4_CMASK                0x00000FF8U
#define STM32_DMA2_CH5_CMASK                0x00000FF8U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                36
#define STM32_EXTI_IMR1_MASK                0xFFD00000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFFU

/* Flash attributes.*/
#if defined(STM32G0B1xx)
#define STM32_FLASH_NUMBER_OF_BANKS         1
#else
#define STM32_FLASH_NUMBER_OF_BANKS         2
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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOEEN |           \
                                             RCC_IOPENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      FALSE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* PWR attributes.*/
#define STM32_PWR_HAS_CR2                   TRUE
#define STM32_PWR_HAS_VDDIO2                TRUE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_HAS_CCIPR2                TRUE
#define STM32_RCC_HAS_MCO2                  TRUE
#define STM32_RCC_HAS_MCOSEL_EXT            TRUE
#define STM32_RCC_HAS_MCOPRE_EXT            TRUE
#define STM32_RCC_HAS_MCOSEL_PLLP           TRUE
#define STM32_RCC_HAS_MCOSEL_PLLQ           TRUE
#define STM32_RCC_HAS_MCOSEL_RTCCLK         TRUE
#define STM32_RCC_HAS_MCOSEL_RTCWKP         TRUE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_LPTIM1                    TRUE
#define STM32_HAS_LPTIM2                    TRUE

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
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               TRUE
#define STM32_USART3_HAS_FIFO               TRUE
#define STM32_UART4_HAS_FIFO                FALSE
#define STM32_UART5_HAS_FIFO                FALSE
#define STM32_USART6_HAS_FIFO               FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     TRUE
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_LPUART1                   TRUE
#define STM32_HAS_LPUART2                   TRUE

#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_HAS_USB1                      TRUE
#define STM32_USB_PMA_SIZE                  2048

#define STM32_HAS_UCPD1                     TRUE
#define STM32_HAS_UCPD2                     TRUE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      FALSE

#endif /* defined(STM32G0B1xx) || defined(STM32G0C1xx) */

/*===========================================================================*/
/* STM32G0B0xx.                                                              */
/*===========================================================================*/

#if defined(STM32G0B0xx)

/* Errata attributes.*/
#define STM32_HAS_TIM1617_ERRATA            FALSE

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
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             5
#define STM32_DMA1_CH2_CMASK                0x00000006U
#define STM32_DMA1_CH3_CMASK                0x00000006U
#define STM32_DMA1_CH4_CMASK                0x00000FF8U
#define STM32_DMA1_CH5_CMASK                0x00000FF8U
#define STM32_DMA1_CH6_CMASK                0x00000FF8U
#define STM32_DMA1_CH7_CMASK                0x00000FF8U
#define STM32_DMA2_CH1_CMASK                0x00000FF8U
#define STM32_DMA2_CH2_CMASK                0x00000FF8U
#define STM32_DMA2_CH3_CMASK                0x00000FF8U
#define STM32_DMA2_CH4_CMASK                0x00000FF8U
#define STM32_DMA2_CH5_CMASK                0x00000FF8U

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_HAS_GROUP2               FALSE
#define STM32_EXTI_NUM_LINES                34
#define STM32_EXTI_IMR1_MASK                0xFFFF0000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFFU

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         2

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
#define STM32_GPIO_EN_MASK                  (RCC_IOPENR_GPIOAEN |           \
                                             RCC_IOPENR_GPIOBEN |           \
                                             RCC_IOPENR_GPIOCEN |           \
                                             RCC_IOPENR_GPIODEN |           \
                                             RCC_IOPENR_GPIOEEN |           \
                                             RCC_IOPENR_GPIOFEN)

/* I2C attributes.*/
#define STM32_I2C_SINGLE_IRQ                TRUE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      FALSE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* PWR attributes.*/
#define STM32_PWR_HAS_CR2                   FALSE /* Handled as special case.*/
#define STM32_PWR_HAS_VDDIO2                FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_HAS_CCIPR2                TRUE
#define STM32_RCC_HAS_MCO2                  TRUE
#define STM32_RCC_HAS_MCOSEL_EXT            TRUE
#define STM32_RCC_HAS_MCOPRE_EXT            TRUE
#define STM32_RCC_HAS_MCOSEL_PLLP           TRUE
#define STM32_RCC_HAS_MCOSEL_PLLQ           TRUE
#define STM32_RCC_HAS_MCOSEL_RTCCLK         TRUE
#define STM32_RCC_HAS_MCOSEL_RTCWKP         TRUE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

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
#define STM32_HAS_LPTIM1                    FALSE
#define STM32_HAS_LPTIM2                    FALSE

/* USART attributes.*/
#define STM32_USART_MIXED                   TRUE
#define STM32_USART1_HAS_FIFO               TRUE
#define STM32_USART2_HAS_FIFO               TRUE
#define STM32_USART3_HAS_FIFO               TRUE
#define STM32_UART4_HAS_FIFO                FALSE
#define STM32_UART5_HAS_FIFO                FALSE
#define STM32_USART6_HAS_FIFO               FALSE

#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     TRUE
#define STM32_HAS_USART6                    TRUE

#define STM32_HAS_LPUART1                   FALSE
#define STM32_HAS_LPUART2                   FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE

/* USB attributes.*/
#define STM32_HAS_USB1                      TRUE
#define STM32_USB_PMA_SIZE                  2048

#define STM32_HAS_UCPD1                     FALSE
#define STM32_HAS_UCPD2                     FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      FALSE

#endif /* defined(STM32G0B0xx) */

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
