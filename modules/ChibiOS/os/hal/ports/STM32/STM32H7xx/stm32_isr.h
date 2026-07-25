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
 * @file    STM32H7xx/stm32_isr.h
 * @brief   STM32H7xx ISR handler header.
 *
 * @addtogroup STM32H7xx_ISR
 * @{
 */

#ifndef STM32_ISR_H
#define STM32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISRs suppressed in standard drivers
 * @{
 */
#define STM32_TIM1_SUPPRESS_ISR
#define STM32_TIM2_SUPPRESS_ISR
#define STM32_TIM3_SUPPRESS_ISR
#define STM32_TIM4_SUPPRESS_ISR
#define STM32_TIM5_SUPPRESS_ISR
#define STM32_TIM6_SUPPRESS_ISR
#define STM32_TIM7_SUPPRESS_ISR
#define STM32_TIM8_SUPPRESS_ISR
#define STM32_TIM12_SUPPRESS_ISR
#define STM32_TIM13_SUPPRESS_ISR
#define STM32_TIM14_SUPPRESS_ISR
#define STM32_TIM15_SUPPRESS_ISR
#define STM32_TIM16_SUPPRESS_ISR
#define STM32_TIM17_SUPPRESS_ISR

#define STM32_USART1_SUPPRESS_ISR
#define STM32_USART2_SUPPRESS_ISR
#define STM32_USART3_SUPPRESS_ISR
#define STM32_UART4_SUPPRESS_ISR
#define STM32_UART5_SUPPRESS_ISR
#define STM32_USART6_SUPPRESS_ISR
#define STM32_UART7_SUPPRESS_ISR
#define STM32_UART8_SUPPRESS_ISR
#define STM32_UART9_SUPPRESS_ISR
#define STM32_USART10_SUPPRESS_ISR
#define STM32_LPUART1_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers
 * @{
 */
/*
 * ADC units.
 */
#define STM32_ADC12_HANDLER                 Vector88
#define STM32_ADC3_HANDLER                  Vector23C

#define STM32_ADC12_NUMBER                  18
#define STM32_ADC3_NUMBER                   127

/*
 * BDMA units.
 */
#define STM32_BDMA1_CH0_HANDLER             Vector244
#define STM32_BDMA1_CH1_HANDLER             Vector248
#define STM32_BDMA1_CH2_HANDLER             Vector24C
#define STM32_BDMA1_CH3_HANDLER             Vector250
#define STM32_BDMA1_CH4_HANDLER             Vector254
#define STM32_BDMA1_CH5_HANDLER             Vector258
#define STM32_BDMA1_CH6_HANDLER             Vector25C
#define STM32_BDMA1_CH7_HANDLER             Vector260

#define STM32_BDMA1_CH0_NUMBER              129
#define STM32_BDMA1_CH1_NUMBER              130
#define STM32_BDMA1_CH2_NUMBER              131
#define STM32_BDMA1_CH3_NUMBER              132
#define STM32_BDMA1_CH4_NUMBER              133
#define STM32_BDMA1_CH5_NUMBER              134
#define STM32_BDMA1_CH6_NUMBER              135
#define STM32_BDMA1_CH7_NUMBER              136

/*
 * DMA units.
 */
#define STM32_DMA1_CH0_HANDLER              Vector6C
#define STM32_DMA1_CH1_HANDLER              Vector70
#define STM32_DMA1_CH2_HANDLER              Vector74
#define STM32_DMA1_CH3_HANDLER              Vector78
#define STM32_DMA1_CH4_HANDLER              Vector7C
#define STM32_DMA1_CH5_HANDLER              Vector80
#define STM32_DMA1_CH6_HANDLER              Vector84
#define STM32_DMA1_CH7_HANDLER              VectorFC
#define STM32_DMA2_CH0_HANDLER              Vector120
#define STM32_DMA2_CH1_HANDLER              Vector124
#define STM32_DMA2_CH2_HANDLER              Vector128
#define STM32_DMA2_CH3_HANDLER              Vector12C
#define STM32_DMA2_CH4_HANDLER              Vector130
#define STM32_DMA2_CH5_HANDLER              Vector150
#define STM32_DMA2_CH6_HANDLER              Vector154
#define STM32_DMA2_CH7_HANDLER              Vector158

#define STM32_DMA1_CH0_NUMBER               11
#define STM32_DMA1_CH1_NUMBER               12
#define STM32_DMA1_CH2_NUMBER               13
#define STM32_DMA1_CH3_NUMBER               14
#define STM32_DMA1_CH4_NUMBER               15
#define STM32_DMA1_CH5_NUMBER               16
#define STM32_DMA1_CH6_NUMBER               17
#define STM32_DMA1_CH7_NUMBER               47
#define STM32_DMA2_CH0_NUMBER               56
#define STM32_DMA2_CH1_NUMBER               57
#define STM32_DMA2_CH2_NUMBER               58
#define STM32_DMA2_CH3_NUMBER               59
#define STM32_DMA2_CH4_NUMBER               60
#define STM32_DMA2_CH5_NUMBER               68
#define STM32_DMA2_CH6_NUMBER               69
#define STM32_DMA2_CH7_NUMBER               70

/*
 * MDMA units.
 */
#define STM32_MDMA_HANDLER                  Vector228

#define STM32_MDMA_NUMBER                   122

/*
 * ETH units.
 */
#define STM32_ETH_HANDLER                   Vector134

#define STM32_ETH_NUMBER                    61

/*
 * EXTI units.
 */
#define STM32_EXTI0_HANDLER                 Vector58
#define STM32_EXTI1_HANDLER                 Vector5C
#define STM32_EXTI2_HANDLER                 Vector60
#define STM32_EXTI3_HANDLER                 Vector64
#define STM32_EXTI4_HANDLER                 Vector68
#define STM32_EXTI5_9_HANDLER               Vector9C
#define STM32_EXTI10_15_HANDLER             VectorE0
#define STM32_EXTI16_HANDLER                Vector44    /* PVD              */
#define STM32_EXTI17_HANDLER                VectorE4    /* RTC ALARM        */
#define STM32_EXTI18_HANDLER                Vector48    /* RTC TAMP CSS     */
#define STM32_EXTI19_HANDLER                Vector4C    /* RTC WAKEUP       */
#define STM32_EXTI2021_HANDLER              Vector264   /* COMP1 COMP2      */

#define STM32_EXTI0_NUMBER                  6
#define STM32_EXTI1_NUMBER                  7
#define STM32_EXTI2_NUMBER                  8
#define STM32_EXTI3_NUMBER                  9
#define STM32_EXTI4_NUMBER                  10
#define STM32_EXTI5_9_NUMBER                23
#define STM32_EXTI10_15_NUMBER              40
#define STM32_EXTI16_NUMBER                 1
#define STM32_EXTI17_NUMBER                 41
#define STM32_EXTI18_NUMBER                 42
#define STM32_EXTI19_NUMBER                 3
#define STM32_EXTI2021_NUMBER               137

/*
 * FDCAN units.
 */
#define STM32_FDCAN1_IT0_HANDLER            Vector8C
#define STM32_FDCAN1_IT1_HANDLER            Vector94
#define STM32_FDCAN2_IT0_HANDLER            Vector90
#define STM32_FDCAN2_IT1_HANDLER            Vector98
#define STM32_FDCAN3_IT0_HANDLER            Vector2BC
#define STM32_FDCAN3_IT1_HANDLER            Vector2C0

#define STM32_FDCAN1_IT0_NUMBER             19
#define STM32_FDCAN1_IT1_NUMBER             21
#define STM32_FDCAN2_IT0_NUMBER             20
#define STM32_FDCAN2_IT1_NUMBER             22
#define STM32_FDCAN3_IT0_NUMBER             159
#define STM32_FDCAN3_IT1_NUMBER             160

/*
 * I2C units.
 */
#define STM32_I2C1_EVENT_HANDLER            VectorBC
#define STM32_I2C1_ERROR_HANDLER            VectorC0
#define STM32_I2C2_EVENT_HANDLER            VectorC4
#define STM32_I2C2_ERROR_HANDLER            VectorC8
#define STM32_I2C3_EVENT_HANDLER            Vector160
#define STM32_I2C3_ERROR_HANDLER            Vector164
#define STM32_I2C4_EVENT_HANDLER            Vector1BC
#define STM32_I2C4_ERROR_HANDLER            Vector1C0
#define STM32_I2C5_EVENT_HANDLER            Vector2B4
#define STM32_I2C5_ERROR_HANDLER            Vector2B8

#define STM32_I2C1_EVENT_NUMBER             31
#define STM32_I2C1_ERROR_NUMBER             32
#define STM32_I2C2_EVENT_NUMBER             33
#define STM32_I2C2_ERROR_NUMBER             34
#define STM32_I2C3_EVENT_NUMBER             72
#define STM32_I2C3_ERROR_NUMBER             73
#define STM32_I2C4_EVENT_NUMBER             95
#define STM32_I2C4_ERROR_NUMBER             96
#define STM32_I2C5_EVENT_NUMBER             157
#define STM32_I2C5_ERROR_NUMBER             158

/*
 * QUADSPI units.
 */
#define STM32_QUADSPI1_HANDLER              Vector1B0

#define STM32_QUADSPI1_NUMBER               92

/*
 * OCTOSPI units.
 */
#define STM32_OCTOSPI1_HANDLER              Vector1B0
#define STM32_OCTOSPI2_HANDLER              Vector298

#define STM32_OCTOSPI1_NUMBER               92
#define STM32_OCTOSPI2_NUMBER               150

/*
 * SDMMC units.
 */
#define STM32_SDMMC1_HANDLER                Vector104
#define STM32_SDMMC2_HANDLER                Vector230

#define STM32_SDMMC1_NUMBER                 49
#define STM32_SDMMC2_NUMBER                 124

/*
 * SPI units.
 */
#define STM32_SPI1_HANDLER                  VectorCC
#define STM32_SPI2_HANDLER                  VectorD0
#define STM32_SPI3_HANDLER                  Vector10C
#define STM32_SPI4_HANDLER                  Vector190
#define STM32_SPI5_HANDLER                  Vector194
#define STM32_SPI6_HANDLER                  Vector198

#define STM32_SPI1_NUMBER                   35
#define STM32_SPI2_NUMBER                   36
#define STM32_SPI3_NUMBER                   51
#define STM32_SPI4_NUMBER                   84
#define STM32_SPI5_NUMBER                   85
#define STM32_SPI6_NUMBER                   86

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_HANDLER              VectorA0
#define STM32_TIM1_UP_HANDLER               VectorA4
#define STM32_TIM1_TRGCO_HANDLER            VectorA8
#define STM32_TIM1_CC_HANDLER               VectorAC
#define STM32_TIM2_HANDLER                  VectorB0
#define STM32_TIM3_HANDLER                  VectorB4
#define STM32_TIM4_HANDLER                  VectorB8
#define STM32_TIM5_HANDLER                  Vector108
#define STM32_TIM6_HANDLER                  Vector118
#define STM32_TIM7_HANDLER                  Vector11C
#define STM32_TIM8_BRK_TIM12_HANDLER        VectorEC
#define STM32_TIM8_UP_TIM13_HANDLER         VectorF0
#define STM32_TIM8_TRGCO_TIM14_HANDLER      VectorF4
#define STM32_TIM8_CC_HANDLER               VectorF8
#define STM32_TIM15_HANDLER                 Vector210
#define STM32_TIM16_HANDLER                 Vector214
#define STM32_TIM17_HANDLER                 Vector218

#define STM32_TIM1_BRK_NUMBER               24
#define STM32_TIM1_UP_NUMBER                25
#define STM32_TIM1_TRGCO_NUMBER             26
#define STM32_TIM1_CC_NUMBER                27
#define STM32_TIM2_NUMBER                   28
#define STM32_TIM3_NUMBER                   29
#define STM32_TIM4_NUMBER                   30
#define STM32_TIM5_NUMBER                   50
#define STM32_TIM6_NUMBER                   54
#define STM32_TIM7_NUMBER                   55
#define STM32_TIM8_BRK_TIM12_NUMBER         43
#define STM32_TIM8_UP_TIM13_NUMBER          44
#define STM32_TIM8_TRGCO_TIM14_NUMBER       45
#define STM32_TIM8_CC_NUMBER                46
#define STM32_TIM15_NUMBER                  116
#define STM32_TIM16_NUMBER                  117
#define STM32_TIM17_NUMBER                  118

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                VectorD4
#define STM32_USART2_HANDLER                VectorD8
#define STM32_USART3_HANDLER                VectorDC
#define STM32_UART4_HANDLER                 Vector110
#define STM32_UART5_HANDLER                 Vector114
#define STM32_USART6_HANDLER                Vector15C
#define STM32_UART7_HANDLER                 Vector188
#define STM32_UART8_HANDLER                 Vector18C
#define STM32_UART9_HANDLER                 Vector2AC
#define STM32_USART10_HANDLER               Vector2B0
#define STM32_LPUART1_HANDLER               Vector278

#define STM32_USART1_NUMBER                 37
#define STM32_USART2_NUMBER                 38
#define STM32_USART3_NUMBER                 39
#define STM32_UART4_NUMBER                  52
#define STM32_UART5_NUMBER                  53
#define STM32_USART6_NUMBER                 71
#define STM32_UART7_NUMBER                  82
#define STM32_UART8_NUMBER                  83
#define STM32_UART9_NUMBER                  155
#define STM32_USART10_NUMBER                156
#define STM32_LPUART1_NUMBER                142

/*
 * USB/OTG units.
 */
#define STM32_OTG1_HANDLER                  Vector1D4
#define STM32_OTG1_EP1OUT_HANDLER           Vector1C8
#define STM32_OTG1_EP1IN_HANDLER            Vector1CC
#define STM32_OTG2_HANDLER                  Vector174
#define STM32_OTG2_EP1OUT_HANDLER           Vector168
#define STM32_OTG2_EP1IN_HANDLER            Vector16C

#define STM32_OTG1_NUMBER                   101
#define STM32_OTG1_EP1OUT_NUMBER            98
#define STM32_OTG1_EP1IN_NUMBER             99
#define STM32_OTG2_NUMBER                   77
#define STM32_OTG2_EP1OUT_NUMBER            74
#define STM32_OTG2_EP1IN_NUMBER             75

/*
 * LTDC units.
 */
#define STM32_LTDC_EV_HANDLER               Vector1A0
#define STM32_LTDC_ER_HANDLER               Vector1A4

#define STM32_LTDC_EV_NUMBER                88
#define STM32_LTDC_ER_NUMBER                89

/*
 * DMA2D units.
 */
#define STM32_DMA2D_HANDLER                 Vector1A8

#define STM32_DMA2D_NUMBER                  90

/*
 * FSMC units.
 */
#define STM32_FSMC_HANDLER                  Vector100

#define STM32_FSMC_NUMBER                   48

/*
 * DCMI units.
 */
#define STM32_DCMI_HANDLER                  Vector178

#define STM32_DCMI_NUMBER                   78
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void irqInit(void);
  void irqDeinit(void);
#ifdef __cplusplus
}
#endif

#endif /* STM32_ISR_H */

/** @} */
