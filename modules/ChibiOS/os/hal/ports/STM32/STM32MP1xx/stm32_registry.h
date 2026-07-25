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
 * @file    STM32MP1xx/stm32_registry.h
 * @brief   STM32MP1xx capabilities registry.
 *
 * @addtogroup STM32MP1xx_REGISTRY
 * @{
 */

#ifndef STM32_REGISTRY_H
#define STM32_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    STM32MP1xx capabilities
 * @{
 */

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* STM32MP157Axx, STM32MP157Cxx, STM32MP157Dxx, STM32MP157Fxx.               */
/* STM32MP153Axx, STM32MP153Cxx, STM32MP153Dxx, STM32MP153Fxx.               */
/* STM32MP151Axx, STM32MP151Cxx, STM32MP151Dxx, STM32MP151Fxx.               */
/*===========================================================================*/

#if defined(STM32MP157Axx) || defined(STM32MP157Cxx) ||                     \
    defined(STM32MP157Dxx) || defined(STM32MP157Fxx) ||                     \
    defined(STM32MP153Axx) || defined(STM32MP153Cxx) ||                     \
    defined(STM32MP153Dxx) || defined(STM32MP153Fxx) ||                     \
    defined(STM32MP151Axx) || defined(STM32MP151Cxx) ||                     \
    defined(STM32MP151Dxx) || defined(STM32MP151Fxx) ||                     \
    defined(__DOXYGEN__)

/* RCC attributes.*/
#define STM32_RCC_HAS_CSI                   TRUE
#define STM32_RCC_HAS_HSI                   TRUE
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE
#define STM32_RCC_HAS_PLL3                  TRUE
#define STM32_RCC_PLL3_HAS_P                TRUE
#define STM32_RCC_PLL3_HAS_Q                TRUE
#define STM32_RCC_PLL3_HAS_R                TRUE

/* ADC attributes.*/

/* CAN attributes.*/

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE
#define STM32_HAS_DAC3_CH1                  FALSE
#define STM32_HAS_DAC3_CH2                  FALSE
#define STM32_HAS_DAC4_CH1                  FALSE
#define STM32_HAS_DAC4_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             8
#define STM32_DMA2_NUM_CHANNELS             8

/* ETH attributes.*/

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                76
#define STM32_EXTI_IMR1_MASK                0xFFFE0000U
#define STM32_EXTI_IMR2_MASK                0xFFFFFFFFU
#define STM32_EXTI_IMR3_MASK                0xFFFFFDE9U

/* Flash attributes.*/

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
#define STM32_HAS_GPIOJ                     TRUE
#define STM32_HAS_GPIOK                     TRUE
#define STM32_HAS_GPIOZ                     TRUE
#define STM32_GPIO_EN_MASK                  (RCC_MC_AHB4ENSETR_GPIOAEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOBEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOCEN |    \
                                             RCC_MC_AHB4ENSETR_GPIODEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOEEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOFEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOGEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOHEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOIEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOJEN |    \
                                             RCC_MC_AHB4ENSETR_GPIOKEN)

/* I2C attributes.*/

/* OCTOSPI attributes.*/

/* SDMMC attributes.*/

/* SPI attributes.*/

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
#define STM32_HAS_USART1                    FALSE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     TRUE
#define STM32_HAS_LPUART1                   TRUE
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE

/* OTG/USB attributes.*/
#define STM32_HAS_OTG1                      FALSE
#define STM32_HAS_OTG2                      FALSE

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32MP157Axx) || defined(STM32MP157Cxx) ||
          defined(STM32MP157Dxx) || defined(STM32MP157Fxx) ||
          defined(STM32MP153Axx) || defined(STM32MP153Cxx) ||
          defined(STM32MP153Dxx) || defined(STM32MP153Fxx) ||
          defined(STM32MP151Axx) || defined(STM32MP151Cxx) ||
          defined(STM32MP151Dxx) || defined(STM32MP151Fxx) */

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
