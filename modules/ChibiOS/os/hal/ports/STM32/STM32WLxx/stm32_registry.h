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
 * @file    STM32WLxx/stm32_registry.h
 * @brief   STM32WLxx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM17_STOP

/* Cores.*/
#if defined(STM32WL55xx) || defined(STM32WL54xx)
#define STM32_HAS_M4                        TRUE
#define STM32_HAS_M0                        TRUE
#else
#define STM32_HAS_M4                        TRUE
#define STM32_HAS_M0                        FALSE
#endif

/**
 * @name    STM32WLxx capabilities
 * @{
 */

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   TRUE

/* RCC attributes.*/
#define STM32_RCC_HAS_HSI16                 TRUE
#define STM32_RCC_HAS_HSI48                 FALSE
#define STM32_RCC_HAS_MSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         TRUE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   FALSE
#define STM32_RCC_HAS_HSE32                 TRUE

#define STM32_RCC_HAS_PLL                   TRUE
#define STM32_RCC_PLL_HAS_P                 TRUE
#define STM32_RCC_PLL_HAS_Q                 TRUE
#define STM32_RCC_PLL_HAS_R                 TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE
#define STM32_ADC_SUPPORTS_OVERSAMPLING     TRUE
#define STM32_ADC_SUPPORTS_PRESCALER        TRUE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      FALSE
#define STM32_HAS_CAN2                      FALSE
#define STM32_HAS_CAN3                      FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  FALSE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     FALSE

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                47
#define STM32_EXTI_IMR1_MASK                0xFF9E0000U
#define STM32_EXTI_IMR2_MASK                0xFFFFDCFBU

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         1
#if !defined(STM32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define STM32_FLASH_SECTORS_PER_BANK        256 /* Maximum, can be redefined.*/
#endif

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      FALSE

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
#define STM32_GPIO_EN_MASK                  (RCC_AHB2ENR_GPIOAEN |          \
                                             RCC_AHB2ENR_GPIOBEN |          \
                                             RCC_AHB2ENR_GPIOCEN)

/* HSEM attributes.*/
#if defined(STM32WL55xx) || defined(STM32WL54xx) || defined(__DOXYGEN__)
#define STM32_HAS_HSEM                      TRUE
#define STM32_HSEM_SEMAPHORES               16
#else
#define STM32_HAS_HSEM                      FALSE
#endif /* defined(STM32WL55xx) || defined(STM32WL54xx) */

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C2                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* IPCC attributes.*/
#if defined(STM32WL55xx) || defined(STM32WL54xx) || defined(__DOXYGEN__)
#define STM32_HAS_IPCC                      TRUE
#define STM32_IPCC_RX_CHANNELS              6
#define STM32_IPCC_TX_CHANNELS              6
#else
#define STM32_HAS_IPCC                      FALSE
#endif /* defined(STM32WL55xx) || defined(STM32WL54xx) */

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      FALSE

/* PKA attributes.*/
#define STM32_HAS_PKA                       TRUE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_HAS_BINARY_MODE           TRUE
#define STM32_RTC_HAS_MIXED_MODE            TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              32
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           19
#define STM32_RTC_WKUP_EXTI                 20

#if !defined(CORE_CM0PLUS)

#define STM32_RTC_TAMP_STAMP_HANDLER        Vector48
#define STM32_RTC_WKUP_HANDLER              Vector4C
#define STM32_RTC_ALARM_HANDLER             VectorE8
#define STM32_RTC_TAMP_STAMP_NUMBER         2
#define STM32_RTC_WKUP_NUMBER               3
#define STM32_RTC_ALARM_NUMBER              42
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI19_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI20_PRIORITY);       \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI17_PRIORITY);      \
} while (false)

#else

#define STM32_RTC_COMMON_HANDLER            Vector48
#define STM32_RTC_COMMON_NUMBER             2
#define STM32_RTC_IRQ_ENABLE()                                              \
  nvicEnableVector(STM32_RTC_COMMON_NUMBER, STM32_IRQ_EXTI17_20_IRQ_PRIORITY)

#endif /* !defined(CORE_CM0PLUS) */

 /* Enabling RTC-related EXTI lines.*/
#define STM32_RTC_ENABLE_ALL_EXTI() do {                                    \
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_ALARM_EXTI) |                       \
                   EXTI_MASK1(STM32_RTC_TAMP_STAMP_EXTI) |                  \
                   EXTI_MASK1(STM32_RTC_WKUP_EXTI),                         \
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);     \
} while (false)

/* Clearing RTC-related EXTI interrupts. */
#define STM32_RTC_CLEAR_ALL_EXTI() do {                                     \
} while (false)

/* Masks used to preserve state of RTC and TAMP register reserved bits. */
#define STM32_RTC_CR_MASK                   0xE7FFFF7F
#define STM32_RTC_PRER_MASK                 0x007F7FFF
#define STM32_TAMP_CR1_MASK                 0xFFFF0007
#define STM32_TAMP_CR2_MASK                 0x07070007
#define STM32_TAMP_FLTCR_MASK               0x000000FF
#define STM32_TAMP_IER_MASK                 0x003C0003

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_TIM16                     TRUE
#define STM32_TIM16_IS_32BITS               FALSE
#define STM32_TIM16_CHANNELS                1

#define STM32_HAS_TIM17                     TRUE
#define STM32_TIM17_IS_32BITS               FALSE
#define STM32_TIM17_CHANNELS                1

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
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* LPTIM attributes.*/
#define STM32_HAS_LPTIM1                    TRUE
#define STM32_HAS_LPTIM2                    TRUE
#define STM32_HAS_LPTIM3                    TRUE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
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

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
