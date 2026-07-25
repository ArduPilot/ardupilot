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
 * @file    STM32H7xx/stm32_registry.h
 * @brief   STM32H7xx capabilities registry.
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
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZ1 |= DBGMCU_APB2FZ1_DBG_TIM1
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM2
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM3
#define STM32_DBGMCU_TIM4_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM4
#define STM32_DBGMCU_TIM5_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM5
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM6
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM7
#define STM32_DBGMCU_TIM8_STOP()            DBGMCU->APB2FZ1 |= DBGMCU_APB2FZ1_DBG_TIM8
#define STM32_DBGMCU_TIM12_STOP()           DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM12
#define STM32_DBGMCU_TIM13_STOP()           DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM13
#define STM32_DBGMCU_TIM14_STOP()           DBGMCU->APB1LFZ1 |= DBGMCU_APB1LFZ1_DBG_TIM14
#define STM32_DBGMCU_TIM15_STOP()           DBGMCU->APB2FZ1 |= DBGMCU_APB2FZ1_DBG_TIM15
#define STM32_DBGMCU_TIM16_STOP()           DBGMCU->APB2FZ1 |= DBGMCU_APB2FZ1_DBG_TIM16
#define STM32_DBGMCU_TIM17_STOP()           DBGMCU->APB2FZ1 |= DBGMCU_APB2FZ1_DBG_TIM17
#define STM32_DBGMCU_TIM23_STOP()           DBGMCU->APB1HFZ1 |= DBGMCU_APB1HFZ1_DBG_TIM23
#define STM32_DBGMCU_TIM24_STOP()           DBGMCU->APB1HFZ1 |= DBGMCU_APB1HFZ1_DBG_TIM24

/* Cores.*/
#if defined(STM32H730xx)  ||                                                \
    defined(STM32H750xx)  ||                                                \
    defined(STM32H7B0xx)  ||                                                \
    defined(STM32H723xx)  || defined(STM32H733xx)  ||                       \
    defined(STM32H725xx)  || defined(STM32H735xx)  ||                       \
    defined(STM32H730xx)  ||                                                \
    defined(STM32H742xx)  ||                                                \
    defined(STM32H743xx)  || defined(STM32H753xx)  ||                       \
    defined(STM32H7A3xx)  || defined(STM32H7B3xx)  ||                       \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ)
#define STM32_HAS_M7                        TRUE
#ifndef STM32_HAS_M4
#define STM32_HAS_M4                        FALSE
#endif
#else
#define STM32_HAS_M7                        TRUE
#ifndef STM32_HAS_M4
#define STM32_HAS_M4                        TRUE
#endif
#endif

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   TRUE

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* I2C attributes.*/
#define STM32_I2C4_USE_BDMA                 TRUE

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
#define STM32_RTC_TAMP_STAMP_EXTI           18
#define STM32_RTC_WKUP_EXTI                 19
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI17_PRIORITY);      \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI18_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI19_PRIORITY);       \
} while (false)

/*===========================================================================*/
/* STM32H730xx, STM32H750xx, STM32H7B0xx, STM32H733xx, STM32H735xx,          */
/* STM32H753xx, STM32H7B3xx, STM32H755xx, STM32H757xx                        */
/*===========================================================================*/

#if defined(STM32H730xx)  || defined(STM32H750xx)  ||                       \
    defined(STM32H7B0xx)  ||                                                \
    defined(STM32H733xx)  || defined(STM32H735xx)  ||                       \
    defined(STM32H753xx)  || defined(STM32H7B3xx)  ||                       \
                             defined(STM32H7B3xxQ) ||                       \
    defined(STM32H755xx)  || defined(STM32H757xx)  ||                       \
    defined(__DOXYGEN__)

/* HASH attributes.*/
#define STM32_HAS_HASH1                     TRUE

/* CRYP attributes.*/
#define STM32_HAS_CRYP1                     TRUE

#else

#define STM32_HAS_HASH1                     FALSE
#define STM32_HAS_CRYP1                     FALSE

#endif

/*===========================================================================*/
/* STM32H743xx, STM32H753xx, STM32H745xx, STM32H755xx, STM32H747xx,          */
/* STM32H757xx.                                                              */
/*===========================================================================*/
#if defined(STM32H743xx) || defined(STM32H753xx) ||                         \
    defined(STM32H745xx) || defined(STM32H755xx) ||                         \
    defined(STM32H747xx) || defined(STM32H757xx) ||                         \
    defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_ADC_RENAMED_REGS              FALSE
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      TRUE
#define STM32_HAS_ADC3                      TRUE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE
#define STM32_HAS_FDCAN3                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* BDMA attributes.*/
#define STM32_HAS_BDMA1                     TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* MDMA attributes.*/
#define STM32_HAS_MDMA1                     TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE

/* EXTI attributes.*/
#define STM32_EXTI_ENHANCED
#define STM32_EXTI_NUM_LINES                34
#define STM32_EXTI_IMR1_MASK                0x1F800000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFCU

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
#define STM32_GPIO_EN_MASK                  (RCC_AHB4ENR_GPIOAEN |          \
                                             RCC_AHB4ENR_GPIOBEN |          \
                                             RCC_AHB4ENR_GPIOCEN |          \
                                             RCC_AHB4ENR_GPIODEN |          \
                                             RCC_AHB4ENR_GPIOEEN |          \
                                             RCC_AHB4ENR_GPIOFEN |          \
                                             RCC_AHB4ENR_GPIOGEN |          \
                                             RCC_AHB4ENR_GPIOHEN |          \
                                             RCC_AHB4ENR_GPIOIEN |          \
                                             RCC_AHB4ENR_GPIOJEN |          \
                                             RCC_AHB4ENR_GPIOKEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      TRUE
#define STM32_HAS_I2C5                      FALSE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_HAS_QUADSPI2                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_HAS_SDMMC2                    TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

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

#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
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
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_USART10                   FALSE
#define STM32_HAS_LPUART1                   TRUE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                8

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      TRUE

/* FDCAN attrubutes */
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_FDCAN1_IT0_HANDLER            Vector8C
#define STM32_FDCAN1_IT1_HANDLER            Vector94
#define STM32_FDCAN1_IT0_NUMBER             19
#define STM32_FDCAN1_IT1_NUMBER             21

#define STM32_HAS_FDCAN2                    TRUE
#define STM32_FDCAN2_IT0_HANDLER            Vector90
#define STM32_FDCAN2_IT1_HANDLER            Vector98
#define STM32_FDCAN2_IT0_NUMBER             20
#define STM32_FDCAN2_IT1_NUMBER             22

#endif /* defined(STM32H743xx) || defined(STM32H753xx) */

/*===========================================================================*/
/* STM32H723xx, STM32H733xx, STM32H725xx, STM32H735xx, STM32H730xx.          */
/*===========================================================================*/
#if defined(STM32H723xx) || defined(STM32H733xx) ||                         \
    defined(STM32H725xx) || defined(STM32H735xx) ||                         \
    defined(STM32H730xx) ||                                                 \
    defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_ADC_RENAMED_REGS              TRUE
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      TRUE
#define STM32_HAS_ADC3                      FALSE /* NOT an error, it is a different ADC type.*/
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE
#define STM32_HAS_FDCAN3                    TRUE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* BDMA attributes.*/
#define STM32_HAS_BDMA1                     TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* MDMA attributes.*/
#define STM32_HAS_MDMA1                     TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE

/* EXTI attributes.*/
#define STM32_EXTI_ENHANCED
#define STM32_EXTI_NUM_LINES                34
#define STM32_EXTI_IMR1_MASK                0x1F800000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFCU

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
#define STM32_HAS_GPIOJ                     TRUE
#define STM32_HAS_GPIOK                     TRUE
#define STM32_GPIO_EN_MASK                  (RCC_AHB4ENR_GPIOAEN |          \
                                             RCC_AHB4ENR_GPIOBEN |          \
                                             RCC_AHB4ENR_GPIOCEN |          \
                                             RCC_AHB4ENR_GPIODEN |          \
                                             RCC_AHB4ENR_GPIOEEN |          \
                                             RCC_AHB4ENR_GPIOFEN |          \
                                             RCC_AHB4ENR_GPIOGEN |          \
                                             RCC_AHB4ENR_GPIOHEN |          \
                                             RCC_AHB4ENR_GPIOJEN |          \
                                             RCC_AHB4ENR_GPIOKEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      TRUE
#define STM32_HAS_I2C5                      TRUE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  TRUE
#define STM32_HAS_OCTOSPI2                  TRUE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE
#define STM32_HAS_QUADSPI2                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_HAS_SDMMC2                    TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

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

#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
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
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE
#define STM32_HAS_UART9                     TRUE
#define STM32_HAS_USART10                   TRUE
#define STM32_HAS_LPUART1                   TRUE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      TRUE

#endif /* defined(STM32H723xx) || defined(STM32H733xx) ||
          defined(STM32H725xx) || defined(STM32H735xx) ||
          defined(STM32H730xx) */

/*===========================================================================*/
/* STM32H7A3xx, STM32H7B3xx, STM32H7A3xxQ, STM32H7B3xxQ.                     */
/*===========================================================================*/
#if defined(STM32H7A3xx)  || defined(STM32H7B3xx)  ||                       \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) ||                       \
    defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_ADC_RENAMED_REGS              FALSE
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      TRUE
#define STM32_HAS_ADC3                      FALSE /* NOT an error, it is a different ADC type.*/
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE
#define STM32_HAS_FDCAN3                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  TRUE
#define STM32_HAS_DAC2_CH2                  FALSE

/* BDMA attributes.*/
#define STM32_HAS_BDMA1                     TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* MDMA attributes.*/
#define STM32_HAS_MDMA1                     TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       FALSE

/* EXTI attributes.*/
#define STM32_EXTI_ENHANCED
#define STM32_EXTI_NUM_LINES                34
#define STM32_EXTI_IMR1_MASK                0x1F800000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFCU

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
#define STM32_GPIO_EN_MASK                  (RCC_AHB4ENR_GPIOAEN |          \
                                             RCC_AHB4ENR_GPIOBEN |          \
                                             RCC_AHB4ENR_GPIOCEN |          \
                                             RCC_AHB4ENR_GPIODEN |          \
                                             RCC_AHB4ENR_GPIOEEN |          \
                                             RCC_AHB4ENR_GPIOFEN |          \
                                             RCC_AHB4ENR_GPIOGEN |          \
                                             RCC_AHB4ENR_GPIOHEN |          \
                                             RCC_AHB4ENR_GPIOIEN |          \
                                             RCC_AHB4ENR_GPIOJEN |          \
                                             RCC_AHB4ENR_GPIOKEN)

/* I2C attributes.*/
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
#define STM32_HAS_QUADSPI2                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_HAS_SDMMC2                    TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE

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

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

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

#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
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
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE
#define STM32_HAS_UART9                     TRUE
#define STM32_HAS_USART10                   TRUE
#define STM32_HAS_LPUART1                   TRUE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      FALSE

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      TRUE

#endif /* defined(STM32H7A3xx)  || defined(STM32H7B3xx) ||
          defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) */

/*===========================================================================*/
/* STM32H750xx.                                                              */
/*===========================================================================*/
#if defined(STM32H750xx) || defined(STM32H7B0xx) ||                         \
    defined(__DOXYGEN__)

/* ADC attributes.*/
#define STM32_ADC_RENAMED_REGS              FALSE
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      TRUE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE
#define STM32_HAS_FDCAN3                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* BDMA attributes.*/
#define STM32_HAS_BDMA1                     TRUE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE

#define STM32_HAS_DMA1                      TRUE
#define STM32_HAS_DMA2                      TRUE

/* MDMA attributes.*/
#define STM32_HAS_MDMA1                     TRUE

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE

/* EXTI attributes.*/
#define STM32_EXTI_ENHANCED
#define STM32_EXTI_NUM_LINES                34
#define STM32_EXTI_IMR1_MASK                0x1F800000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFCU

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
#define STM32_GPIO_EN_MASK                  (RCC_AHB4ENR_GPIOAEN |          \
                                             RCC_AHB4ENR_GPIOBEN |          \
                                             RCC_AHB4ENR_GPIOCEN |          \
                                             RCC_AHB4ENR_GPIODEN |          \
                                             RCC_AHB4ENR_GPIOEEN |          \
                                             RCC_AHB4ENR_GPIOFEN |          \
                                             RCC_AHB4ENR_GPIOGEN |          \
                                             RCC_AHB4ENR_GPIOHEN |          \
                                             RCC_AHB4ENR_GPIOIEN |          \
                                             RCC_AHB4ENR_GPIOJEN |          \
                                             RCC_AHB4ENR_GPIOKEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      TRUE
#define STM32_HAS_I2C4                      TRUE
#define STM32_HAS_I2C5                      FALSE

/* OCTOSPI attributes.*/
#define STM32_HAS_OCTOSPI1                  FALSE
#define STM32_HAS_OCTOSPI2                  FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  TRUE
#define STM32_HAS_QUADSPI2                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_HAS_SDMMC2                    TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             TRUE
#define STM32_SPI1_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE

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
#define STM32_HAS_TIM9                      FALSE
#define STM32_HAS_TIM10                     FALSE
#define STM32_HAS_TIM11                     FALSE
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
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_USART10                   FALSE
#define STM32_HAS_LPUART1                   TRUE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                8

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

/* DCMI attributes.*/
#define STM32_HAS_DCMI                      TRUE

#endif /* defined(STM32H750xx) */

#endif /* STM32_REGISTRY_H */

/** @} */
