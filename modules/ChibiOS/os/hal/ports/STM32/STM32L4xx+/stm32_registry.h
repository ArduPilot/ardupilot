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
 * @file    STM32L4xx+/stm32_registry.h
 * @brief   STM32L4xx+ capabilities registry.
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
 * @name    STM32L4xx+ capabilities
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
#define STM32_RCC_PLLSAI2_HAS_Q             TRUE
#define STM32_RCC_PLLSAI2_HAS_R             TRUE

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

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)

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
#define STM32_TAMP_CR1_MASK                 0x003C0007
#define STM32_TAMP_CR2_MASK                 0x07070007
#define STM32_TAMP_FLTCR_MASK               0x000000FF
#define STM32_TAMP_IER_MASK                 0x003C0007
#endif/* !(defined(STM32L4P5xx) || defined(STM32L4Q5xx)) */

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4S5xx) || \
    defined(STM32L4S7xx) || defined(STM32L4S9xx) || defined(__DOXYGEN__)
#define STM32_HAS_HASH1                     TRUE
#if defined(STM32L4P5xx)
#define STM32_HAS_CRYP1                     FALSE
#else
#define STM32_HAS_CRYP1                     TRUE
#endif
#else
#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     FALSE
#endif

/* I2C attributes.*/
#define STM32_I2C4_USE_BDMA                 FALSE

/*===========================================================================*/
/* STM32L4yyxx+.                                                             */
/*===========================================================================*/

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || \
    defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || \
    defined(STM32L4S7xx) || defined(STM32L4S9xx) || defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE

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
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                41
#define STM32_EXTI_IMR1_MASK                0xFF820000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFF87U

/* Flash attributes.*/
#define STM32_FLASH_NUMBER_OF_BANKS         2

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
#define STM32_I2C_SINGLE_IRQ                FALSE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      TRUE
#define STM32_HAS_I2C5                      FALSE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  TRUE
#define STM32_HAS_OCTOSPI2                  TRUE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)
#define STM32_HAS_SDMMC2                    TRUE
#else
#define STM32_HAS_SDMMC2                    FALSE
#endif

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             FALSE

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
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     TRUE
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
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   FALSE

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              TRUE

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      TRUE

#endif /* defined(STM32L4P5xx) || defined(STM32L4Q5xx) ||
          defined(STM32L4R5xx) || defined(STM32L4R7xx) ||
          defined(STM32L4R9xx) || defined(STM32L4S5xx) ||
          defined(STM32L4S7xx) || defined(STM32L4S9xx) */

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
