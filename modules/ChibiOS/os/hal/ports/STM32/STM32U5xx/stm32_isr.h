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
 * @file    STM32H5xx/stm32_isr.h
 * @brief   STM32H5xx ISR handler header.
 *
 * @addtogroup STM32H5xx_ISR
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
#define STM32_USART11_SUPPRESS_ISR
#define STM32_UART12_SUPPRESS_ISR
#define STM32_LPUART1_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers
 * @{
 */
/*
 * ADC unit.
 */
#define STM32_ADC1_HANDLER                  VectorD4
#define STM32_ADC2_HANDLER                  Vector154

#define STM32_ADC1_NUMBER                   37
#define STM32_ADC2_NUMBER                   69

/*
 * DMA unit.
 */
#define STM32_GPDMA1_CH0_HANDLER            VectorAC
#define STM32_GPDMA1_CH1_HANDLER            VectorB0
#define STM32_GPDMA1_CH2_HANDLER            VectorB4
#define STM32_GPDMA1_CH3_HANDLER            VectorB8
#define STM32_GPDMA1_CH4_HANDLER            VectorBC
#define STM32_GPDMA1_CH5_HANDLER            VectorC0
#define STM32_GPDMA1_CH6_HANDLER            VectorC4
#define STM32_GPDMA1_CH7_HANDLER            VectorC8
#define STM32_GPDMA2_CH0_HANDLER            Vector1A8
#define STM32_GPDMA2_CH1_HANDLER            Vector1AC
#define STM32_GPDMA2_CH2_HANDLER            Vector1B0
#define STM32_GPDMA2_CH3_HANDLER            Vector1B4
#define STM32_GPDMA2_CH4_HANDLER            Vector1B8
#define STM32_GPDMA2_CH5_HANDLER            Vector1BC
#define STM32_GPDMA2_CH6_HANDLER            Vector1C0
#define STM32_GPDMA2_CH7_HANDLER            Vector1C4

#define STM32_GPDMA1_CH0_NUMBER             27
#define STM32_GPDMA1_CH1_NUMBER             28
#define STM32_GPDMA1_CH2_NUMBER             29
#define STM32_GPDMA1_CH3_NUMBER             30
#define STM32_GPDMA1_CH4_NUMBER             31
#define STM32_GPDMA1_CH5_NUMBER             32
#define STM32_GPDMA1_CH6_NUMBER             33
#define STM32_GPDMA1_CH7_NUMBER             34
#define STM32_GPDMA2_CH0_NUMBER             90
#define STM32_GPDMA2_CH1_NUMBER             91
#define STM32_GPDMA2_CH2_NUMBER             92
#define STM32_GPDMA2_CH3_NUMBER             93
#define STM32_GPDMA2_CH4_NUMBER             94
#define STM32_GPDMA2_CH5_NUMBER             95
#define STM32_GPDMA2_CH6_NUMBER             96
#define STM32_GPDMA2_CH7_NUMBER             97

/*
 * EXTI unit.
 */
#define STM32_EXTI0_HANDLER                 Vector6C
#define STM32_EXTI1_HANDLER                 Vector70
#define STM32_EXTI2_HANDLER                 Vector74
#define STM32_EXTI3_HANDLER                 Vector78
#define STM32_EXTI4_HANDLER                 Vector7C
#define STM32_EXTI5_HANDLER                 Vector80
#define STM32_EXTI6_HANDLER                 Vector84
#define STM32_EXTI7_HANDLER                 Vector88
#define STM32_EXTI8_HANDLER                 Vector8C
#define STM32_EXTI9_HANDLER                 Vector90
#define STM32_EXTI10_HANDLER                Vector94
#define STM32_EXTI11_HANDLER                Vector98
#define STM32_EXTI12_HANDLER                Vector9C
#define STM32_EXTI13_HANDLER                VectorA0
#define STM32_EXTI14_HANDLER                VectorA4
#define STM32_EXTI15_HANDLER                VectorA8

#define STM32_EXTI0_NUMBER                  11
#define STM32_EXTI1_NUMBER                  12
#define STM32_EXTI2_NUMBER                  13
#define STM32_EXTI3_NUMBER                  14
#define STM32_EXTI4_NUMBER                  15
#define STM32_EXTI5_NUMBER                  16
#define STM32_EXTI6_NUMBER                  17
#define STM32_EXTI7_NUMBER                  18
#define STM32_EXTI8_NUMBER                  19
#define STM32_EXTI9_NUMBER                  20
#define STM32_EXTI10_NUMBER                 21
#define STM32_EXTI11_NUMBER                 22
#define STM32_EXTI12_NUMBER                 23
#define STM32_EXTI13_NUMBER                 24
#define STM32_EXTI14_NUMBER                 25
#define STM32_EXTI15_NUMBER                 26

/*
 * SPI units.
 */
#define STM32_SPI1_HANDLER                  Vector11C
#define STM32_SPI2_HANDLER                  Vector120
#define STM32_SPI3_HANDLER                  Vector124
#define STM32_SPI4_HANDLER                  Vector188
#define STM32_SPI5_HANDLER                  Vector18C
#define STM32_SPI6_HANDLER                  Vector190

#define STM32_SPI1_NUMBER                   55
#define STM32_SPI2_NUMBER                   56
#define STM32_SPI3_NUMBER                   57
#define STM32_SPI4_NUMBER                   82
#define STM32_SPI5_NUMBER                   83
#define STM32_SPI6_NUMBER                   84

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_TERR_IERR_HANDLER    VectorE4
#define STM32_TIM1_UP_HANDLER               VectorE8
#define STM32_TIM1_TRGCO_DIR_IDX_HANDLER    VectorEC
#define STM32_TIM1_CC_HANDLER               VectorF0
#define STM32_TIM2_HANDLER                  VectorF4
#define STM32_TIM3_HANDLER                  VectorF8
#define STM32_TIM4_HANDLER                  VectorFC
#define STM32_TIM5_HANDLER                  Vector100
#define STM32_TIM6_HANDLER                  Vector104
#define STM32_TIM7_HANDLER                  Vector108
#define STM32_TIM8_BRK_TERR_IERR_HANDLER    Vector144
#define STM32_TIM8_UP_HANDLER               Vector148
#define STM32_TIM8_TRGCO_DIR_IDX_HANDLER    Vector14C
#define STM32_TIM8_CC_HANDLER               Vector150
#define STM32_TIM12_HANDLER                 Vector208
#define STM32_TIM13_HANDLER                 Vector20C
#define STM32_TIM14_HANDLER                 Vector210
#define STM32_TIM15_HANDLER                 Vector15C
#define STM32_TIM16_HANDLER                 Vector160
#define STM32_TIM17_HANDLER                 Vector164

#define STM32_TIM1_BRK_TERR_IERR_NUMBER     41
#define STM32_TIM1_UP_TIM16_NUMBER          42
#define STM32_TIM1_TRGCO_DIR_IDX_NUMBER     43
#define STM32_TIM1_CC_NUMBER                44
#define STM32_TIM2_NUMBER                   45
#define STM32_TIM3_NUMBER                   46
#define STM32_TIM4_NUMBER                   47
#define STM32_TIM5_NUMBER                   48
#define STM32_TIM6_NUMBER                   49
#define STM32_TIM7_NUMBER                   50
#define STM32_TIM8_BRK_TERR_IERR_NUMBER     65
#define STM32_TIM8_UP_NUMBER                66
#define STM32_TIM8_TRGCO_DIR_IDX_NUMBER     67
#define STM32_TIM8_CC_NUMBER                68
#define STM32_TIM12_NUMBER                  120
#define STM32_TIM13_NUMBER                  121
#define STM32_TIM14_NUMBER                  122
#define STM32_TIM15_NUMBER                  71
#define STM32_TIM16_NUMBER                  72
#define STM32_TIM17_NUMBER                  73

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                Vector128
#define STM32_USART2_HANDLER                Vector12C
#define STM32_USART3_HANDLER                Vector130
#define STM32_UART4_HANDLER                 Vector134
#define STM32_UART5_HANDLER                 Vector138
#define STM32_USART6_HANDLER                Vector194
#define STM32_UART7_HANDLER                 Vector1C8
#define STM32_UART8_HANDLER                 Vector1CC
#define STM32_UART9_HANDLER                 Vector1D0
#define STM32_USART10_HANDLER               Vector198
#define STM32_USART11_HANDLER               Vector19C
#define STM32_UART12_HANDLER                Vector1D4
#define STM32_LPUART1_HANDLER               Vector13C

#define STM32_USART1_NUMBER                 58
#define STM32_USART2_NUMBER                 59
#define STM32_USART3_NUMBER                 60
#define STM32_UART4_NUMBER                  61
#define STM32_UART5_NUMBER                  62
#define STM32_USART6_NUMBER                 85
#define STM32_UART7_NUMBER                  98
#define STM32_UART8_NUMBER                  99
#define STM32_UART9_NUMBER                  100
#define STM32_USART10_NUMBER                86
#define STM32_USART11_NUMBER                87
#define STM32_UART12_NUMBER                 101
#define STM32_LPUART1_NUMBER                63

/*
 * USB units.
 */
#define STM32_USB1_HP_HANDLER               Vector168
#define STM32_USB1_LP_HANDLER               Vector168
#define STM32_USB1_HP_NUMBER                74
#define STM32_USB1_LP_NUMBER                74
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
