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
 * @file    STM32H5xx/stm32_registry.h
 * @brief   STM32H5xx capabilities registry.
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
 * @name    STM32H5xx capabilities
 * @{
 */

/* DBGMCU helpers.*/
#define STM32_DBGMCU_TIM1_STOP()            DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM1_STOP
#define STM32_DBGMCU_TIM2_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP
#define STM32_DBGMCU_TIM3_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM3_STOP
#define STM32_DBGMCU_TIM4_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM4_STOP
#define STM32_DBGMCU_TIM5_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM5_STOP
#define STM32_DBGMCU_TIM6_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM6_STOP
#define STM32_DBGMCU_TIM7_STOP()            DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM7_STOP
#define STM32_DBGMCU_TIM8_STOP()            DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM8_STOP
#define STM32_DBGMCU_TIM12_STOP()           DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM12_STOP
#define STM32_DBGMCU_TIM13_STOP()           DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM13_STOP
#define STM32_DBGMCU_TIM14_STOP()           DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM14_STOP
#define STM32_DBGMCU_TIM15_STOP()           DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM15_STOP
#define STM32_DBGMCU_TIM16_STOP()           DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM16_STOP
#define STM32_DBGMCU_TIM17_STOP()           DBGMCU->APB2FZR |= DBGMCU_APB2FZR_DBG_TIM17_STOP

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* DAC attributes.*/
#define STM32_DAC_HAS_MCR                   TRUE

/* DMA3 attributes.*/
#define STM32_DMA3_MEMORY_PORT              0U
#define STM32_DMA3_PERIPHERAL_PORT          1U
#define STM32_DMA31_NUM_CHANNELS            8U
#define STM32_DMA31_MASK_FIFO2              0x0000000FU
#define STM32_DMA31_MASK_FIFO4              0x00000030U
#define STM32_DMA31_MASK_FIFO4_2D           0x000000C0U
#define STM32_DMA32_NUM_CHANNELS            8U
#define STM32_DMA32_MASK_FIFO2              0x00000F00U
#define STM32_DMA32_MASK_FIFO4              0x00003000U
#define STM32_DMA32_MASK_FIFO4_2D           0x0000C000U

#define STM32_DMA3_REQ_ADC1                 0U
#define STM32_DMA3_REQ_ADC2                 1U
#define STM32_DMA3_REQ_DAC1_CH1             2U
#define STM32_DMA3_REQ_DAC1_CH2             3U
#define STM32_DMA3_REQ_TIM6_UPD             4U
#define STM32_DMA3_REQ_TIM7_UPD             5U
#define STM32_DMA3_REQ_SPI1_RX              6U
#define STM32_DMA3_REQ_SPI1_TX              7U
#define STM32_DMA3_REQ_SPI2_RX              8U
#define STM32_DMA3_REQ_SPI2_TX              9U
#define STM32_DMA3_REQ_SPI3_RX              10U
#define STM32_DMA3_REQ_SPI3_TX              11U
#define STM32_DMA3_REQ_I2C1_RX              12U
#define STM32_DMA3_REQ_I2C1_TX              13U
#define STM32_DMA3_REQ_RESERVED14           14U
#define STM32_DMA3_REQ_I2C2_RX              15U
#define STM32_DMA3_REQ_I2C2_TX              16U
#define STM32_DMA3_REQ_RESERVED17           17U
#define STM32_DMA3_REQ_I2C3_RX              18U
#define STM32_DMA3_REQ_I2C3_TX              19U
#define STM32_DMA3_REQ_RESERVED20           20U
#define STM32_DMA3_REQ_USART1_RX            21U
#define STM32_DMA3_REQ_USART1_TX            22U
#define STM32_DMA3_REQ_USART2_RX            23U
#define STM32_DMA3_REQ_USART2_TX            24U
#define STM32_DMA3_REQ_USART3_RX            25U
#define STM32_DMA3_REQ_USART3_TX            26U
#define STM32_DMA3_REQ_UART4_RX             27U
#define STM32_DMA3_REQ_UART4_TX             28U
#define STM32_DMA3_REQ_UART5_RX             29U
#define STM32_DMA3_REQ_UART5_TX             30U
#define STM32_DMA3_REQ_USART6_RX            31U
#define STM32_DMA3_REQ_USART6_TX            32U
#define STM32_DMA3_REQ_UART7_RX             33U
#define STM32_DMA3_REQ_UART7_TX             34U
#define STM32_DMA3_REQ_UART8_RX             35U
#define STM32_DMA3_REQ_UART8_TX             36U
#define STM32_DMA3_REQ_UART9_RX             37U
#define STM32_DMA3_REQ_UART9_TX             38U
#define STM32_DMA3_REQ_USART10_RX           39U
#define STM32_DMA3_REQ_USART10_TX           40U
#define STM32_DMA3_REQ_USART11_RX           41U
#define STM32_DMA3_REQ_USART11_TX           42U
#define STM32_DMA3_REQ_UART12_RX            43U
#define STM32_DMA3_REQ_UART12_TX            44U
#define STM32_DMA3_REQ_LPUART1_RX           45U
#define STM32_DMA3_REQ_LPUART1_TX           46U
#define STM32_DMA3_REQ_SPI4_RX              47U
#define STM32_DMA3_REQ_SPI4_TX              48U
#define STM32_DMA3_REQ_SPI5_RX              49U
#define STM32_DMA3_REQ_SPI5_TX              50U
#define STM32_DMA3_REQ_SPI6_RX              51U
#define STM32_DMA3_REQ_SPI6_TX              52U
#define STM32_DMA3_REQ_SAI1_A               53U
#define STM32_DMA3_REQ_SAI1_B               54U
#define STM32_DMA3_REQ_SAI2_A               55U
#define STM32_DMA3_REQ_SAI2_B               56U
#define STM32_DMA3_REQ_OSPI1                57U
#define STM32_DMA3_REQ_TIM1_CC1             58U
#define STM32_DMA3_REQ_TIM1_CC2             59U
#define STM32_DMA3_REQ_TIM1_CC3             60U
#define STM32_DMA3_REQ_TIM1_CC4             61U
#define STM32_DMA3_REQ_TIM1_UPD             62U
#define STM32_DMA3_REQ_TIM1_TRG             63U
#define STM32_DMA3_REQ_TIM1_COM             64U
#define STM32_DMA3_REQ_TIM8_CC1             65U
#define STM32_DMA3_REQ_TIM8_CC2             66U
#define STM32_DMA3_REQ_TIM8_CC3             67U
#define STM32_DMA3_REQ_TIM8_CC4             68U
#define STM32_DMA3_REQ_TIM8_UPD             69U
#define STM32_DMA3_REQ_TIM8_TRG             70U
#define STM32_DMA3_REQ_TIM8_COM             71U
#define STM32_DMA3_REQ_TIM2_CC1             72U
#define STM32_DMA3_REQ_TIM2_CC2             73U
#define STM32_DMA3_REQ_TIM2_CC3             74U
#define STM32_DMA3_REQ_TIM2_CC4             75U
#define STM32_DMA3_REQ_TIM2_UPD             76U
#define STM32_DMA3_REQ_TIM3_CC1             77U
#define STM32_DMA3_REQ_TIM3_CC2             78U
#define STM32_DMA3_REQ_TIM3_CC3             79U
#define STM32_DMA3_REQ_TIM3_CC4             80U
#define STM32_DMA3_REQ_TIM3_UPD             81U
#define STM32_DMA3_REQ_TIM3_TRG             82U
#define STM32_DMA3_REQ_TIM4_CC1             83U
#define STM32_DMA3_REQ_TIM4_CC2             84U
#define STM32_DMA3_REQ_TIM4_CC3             85U
#define STM32_DMA3_REQ_TIM4_CC4             86U
#define STM32_DMA3_REQ_TIM4_UPD             87U
#define STM32_DMA3_REQ_TIM5_CC1             88U
#define STM32_DMA3_REQ_TIM5_CC2             89U
#define STM32_DMA3_REQ_TIM5_CC3             90U
#define STM32_DMA3_REQ_TIM5_CC4             91U
#define STM32_DMA3_REQ_TIM5_UPD             92U
#define STM32_DMA3_REQ_TIM5_TRG             93U
#define STM32_DMA3_REQ_TIM15_CC1            94U
#define STM32_DMA3_REQ_TIM15_UPD            95U
#define STM32_DMA3_REQ_TIM15_TRG            96U
#define STM32_DMA3_REQ_TIM15_COM            97U
#define STM32_DMA3_REQ_TIM16_CC1            98U
#define STM32_DMA3_REQ_TIM16_UPD            99U
#define STM32_DMA3_REQ_TIM17_CC1            100U
#define STM32_DMA3_REQ_TIM17_UPD            101U
#define STM32_DMA3_REQ_LPTIM1_IC1           102U
#define STM32_DMA3_REQ_LPTIM1_IC2           103U
#define STM32_DMA3_REQ_LPTIM1_UE            104U
#define STM32_DMA3_REQ_LPTIM2_IC1           105U
#define STM32_DMA3_REQ_LPTIM2_IC2           106U
#define STM32_DMA3_REQ_LPTIM2_UE            107U
#define STM32_DMA3_REQ_DCMI                 108U
#define STM32_DMA3_REQ_AES_OUT              109U
#define STM32_DMA3_REQ_AES_IN               110U
#define STM32_DMA3_REQ_HASH_IN              111U
#define STM32_DMA3_REQ_UCPD1_RX             112U
#define STM32_DMA3_REQ_UCPD1_TX             113U
#define STM32_DMA3_REQ_CORDIC_READ          114U
#define STM32_DMA3_REQ_CORDIC_WRITE         115U
#define STM32_DMA3_REQ_FMAC_READ            116U
#define STM32_DMA3_REQ_FMAC_WRITE           117U
#define STM32_DMA3_REQ_SAES_OUT             118U
#define STM32_DMA3_REQ_SAES_IN              119U
#define STM32_DMA3_REQ_I3C_RX               120U
#define STM32_DMA3_REQ_I3C_TX               121U
#define STM32_DMA3_REQ_I3C_TC               122U
#define STM32_DMA3_REQ_I3C_RS               123U
#define STM32_DMA3_REQ_I2C4_RX              124U
#define STM32_DMA3_REQ_I2C4_TX              125U
#define STM32_DMA3_REQ_RESERVED126          126U
#define STM32_DMA3_REQ_LPTIM3_IC1           127U
#define STM32_DMA3_REQ_LPTIM3_IC2           128U
#define STM32_DMA3_REQ_LPTIM3_UE            129U
#define STM32_DMA3_REQ_LPTIM5_IC1           130U
#define STM32_DMA3_REQ_LPTIM5_IC2           131U
#define STM32_DMA3_REQ_LPTIM5_UE            132U
#define STM32_DMA3_REQ_LPTIM6_IC1           133U
#define STM32_DMA3_REQ_LPTIM6_IC2           134U
#define STM32_DMA3_REQ_LPTIM6_UE            135U

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              128
#define STM32_RTC_GLOBAL_HANDLER            Vector48
#define STM32_RTC_TAMP_HANDLER              Vector50
#define STM32_RTC_GLOBAL_NUMBER             2
#define STM32_RTC_TAMP_NUMBER               4
#define STM32_RTC_GLOBAL_EXTI               17
#define STM32_RTC_TAMP_EXTI                 19
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_GLOBAL_NUMBER, STM32_IRQ_EXTI17_PRIORITY);     \
  nvicEnableVector(STM32_RTC_TAMP_NUMBER, STM32_IRQ_EXTI19_PRIORITY);       \
} while (false)

 /* Enabling RTC-related EXTI lines.*/
#define STM32_RTC_ENABLE_ALL_EXTI() do {                                    \
  extiEnableGroup1(EXTI_MASK1(STM32_RTC_GLOBAL_EXTI) |                      \
                   EXTI_MASK1(STM32_RTC_TAMP_EXTI),                         \
                   EXTI_MODE_RISING_EDGE | EXTI_MODE_ACTION_INTERRUPT);     \
} while (false)

/* Clearing EXTI interrupts. */
#define STM32_RTC_CLEAR_ALL_EXTI() do {                                     \
  extiClearGroup1(EXTI_MASK1(STM32_RTC_GLOBAL_EXTI) |                       \
                  EXTI_MASK1(STM32_RTC_TAMP_EXTI));                         \
} while (false)

/* Masks used to preserve state of RTC and TAMP register reserved bits. */
#define STM32_RTC_CR_MASK                   0xE7FFFF7F
#define STM32_RTC_PRER_MASK                 0x007F7FFF
#define STM32_TAMP_CR1_MASK                 0x003C0007
#define STM32_TAMP_CR2_MASK                 0x07070007
#define STM32_TAMP_FLTCR_MASK               0x000000FF
#define STM32_TAMP_IER_MASK                 0x003C0007

/*===========================================================================*/
/* STM32H503xx.                                                              */
/*===========================================================================*/

#if defined(STM32H503xx) || defined(__DOXYGEN__)

/* SYSTEM and PWR attributes because device differences.*/
#define STM32_SYSTEM_HAS_SEC                FALSE
#define STM32_SYSTEM_HAS_AHB4               FALSE
#define STM32_PWR_HAS_UCPDR                 FALSE
#define STM32_PWR_HAS_USBSCR                FALSE
#define STM32_PWR_HAS_SECCFGR               FALSE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      FALSE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* Cache attributes.*/
#define STM32_HAS_ICACHE                    TRUE
#define STM32_ICACHE_HAS_REGIONS            FALSE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    FALSE
#define STM32_HAS_FDCAN3                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_NUM_LINES                51
#define STM32_EXTI_IMR1_MASK                0xFFFE0000U
#define STM32_EXTI_IMR2_MASK                0x07DBBFFFU

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
#define STM32_I2C_SINGLE_IRQ                FALSE
#define STM32_HAS_I2C1                      TRUE
#define STM32_HAS_I2C2                      TRUE
#define STM32_HAS_I2C3                      FALSE
#define STM32_HAS_I2C4                      FALSE

/* RCC attributes.*/
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_CSI                   TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_HSI                   TRUE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL1                  TRUE
#define STM32_RCC_PLL1_HAS_P                TRUE
#define STM32_RCC_PLL1_HAS_Q                TRUE
#define STM32_RCC_PLL1_HAS_R                TRUE

#define STM32_RCC_HAS_PLL2                  TRUE
#define STM32_RCC_PLL2_HAS_P                TRUE
#define STM32_RCC_PLL2_HAS_Q                TRUE
#define STM32_RCC_PLL2_HAS_R                TRUE

#define STM32_RCC_HAS_PLL3                  FALSE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    FALSE
#define STM32_HAS_SDMMC2                    FALSE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_FULL_FEATURE             TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_FULL_FEATURE             TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_FULL_FEATURE             TRUE

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
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     FALSE
#define STM32_HAS_UART5                     FALSE
#define STM32_HAS_USART6                    FALSE
#define STM32_HAS_UART7                     FALSE
#define STM32_HAS_UART8                     FALSE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_USART10                   FALSE
#define STM32_HAS_USART11                   FALSE
#define STM32_HAS_UART12                    FALSE
#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USB1                      TRUE
#define STM32_USB_PMA_SIZE                  2048

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32H503xx) */


/*===========================================================================*/
/* STM32H562xx, STM32H563xx, STM32H573xx.                                    */
/*===========================================================================*/

#if defined(STM32H562xx) || defined(STM32H563xx) ||                         \
    defined(STM32H573xx) || defined(__DOXYGEN__)

/* SYSTEM and PWR attributes because device differences.*/
#define STM32_SYSTEM_HAS_SEC                TRUE
#define STM32_SYSTEM_HAS_AHB4               TRUE
#define STM32_PWR_HAS_UCPDR                 TRUE
#define STM32_PWR_HAS_USBSCR                TRUE
#define STM32_PWR_HAS_SECCFGR               TRUE

/* ADC attributes.*/
#define STM32_HAS_ADC1                      TRUE
#define STM32_HAS_ADC2                      TRUE
#define STM32_HAS_ADC3                      FALSE
#define STM32_HAS_ADC4                      FALSE

/* Cache attributes.*/
#define STM32_HAS_ICACHE                    TRUE
#define STM32_ICACHE_HAS_REGIONS            TRUE

/* CAN attributes.*/
#define STM32_HAS_FDCAN1                    TRUE
#define STM32_HAS_FDCAN2                    TRUE
#define STM32_HAS_FDCAN3                    FALSE

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE
#define STM32_HAS_DAC1_CH2                  TRUE
#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* EXTI attributes.*/
#define STM32_EXTI_HAS_CR                   TRUE
#define STM32_EXTI_SEPARATE_RF              TRUE
#define STM32_EXTI_NUM_LINES                58
#define STM32_EXTI_IMR1_MASK                0xFFFE0000U
#define STM32_EXTI_IMR2_MASK                0x03DBBFFFU

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

/* RCC attributes.*/
#define STM32_RCC_HAS_LSI                   TRUE
#define STM32_RCC_HAS_LSI_PRESCALER         FALSE
#define STM32_RCC_HAS_CSI                   TRUE
#define STM32_RCC_HAS_HSI48                 TRUE
#define STM32_RCC_HAS_HSI                   TRUE
#define STM32_RCC_HAS_LSE                   TRUE
#define STM32_RCC_HAS_HSE                   TRUE

#define STM32_RCC_HAS_PLL1                  TRUE
#define STM32_RCC_PLL1_HAS_P                TRUE
#define STM32_RCC_PLL1_HAS_Q                TRUE
#define STM32_RCC_PLL1_HAS_R                TRUE

#define STM32_RCC_HAS_PLL2                  TRUE
#define STM32_RCC_PLL2_HAS_P                TRUE
#define STM32_RCC_PLL2_HAS_Q                TRUE
#define STM32_RCC_PLL2_HAS_R                TRUE

#define STM32_RCC_HAS_PLL3                  TRUE
#define STM32_RCC_PLL3_HAS_P                TRUE
#define STM32_RCC_PLL3_HAS_Q                TRUE
#define STM32_RCC_PLL3_HAS_R                TRUE

/* SDMMC attributes.*/
#define STM32_HAS_SDMMC1                    TRUE
#define STM32_HAS_SDMMC2                    TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_FULL_FEATURE             TRUE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_FULL_FEATURE             TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_FULL_FEATURE             TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_FULL_FEATURE             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_FULL_FEATURE             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_FULL_FEATURE             FALSE

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
#define STM32_HAS_USART11                   TRUE
#define STM32_HAS_UART12                    TRUE
#define STM32_HAS_LPUART1                   TRUE

#define STM32_HAS_USB1                      TRUE
#define STM32_USB_PMA_SIZE                  2048

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              TRUE

#endif /* defined(STM32H562xx) || defined(STM32H563xx) || defined(STM32H573xx) */

/** @} */

#endif /* STM32_REGISTRY_H */

/** @} */
