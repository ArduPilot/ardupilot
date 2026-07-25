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
/*
    Concepts and parts of this file have been contributed by Ilya Kharin.
*/

/**
 * @file    STM32WBxx/stm32_isr.h
 * @brief   STM32WBxx ISR handler header.
 *
 * @addtogroup SRM32WBxx_ISR
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
#define STM32_TIM16_SUPPRESS_ISR
#define STM32_TIM17_SUPPRESS_ISR

#define STM32_USART1_SUPPRESS_ISR
#define STM32_LPUART1_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers
 * @{
 */
/*
 * ADC unit.
 */
#define STM32_ADC1_HANDLER                  Vector88

#define STM32_ADC1_NUMBER                   18

/*
 * DMA unit.
 */
#define STM32_DMA1_CH1_HANDLER              Vector6C
#define STM32_DMA1_CH2_HANDLER              Vector70
#define STM32_DMA1_CH3_HANDLER              Vector74
#define STM32_DMA1_CH4_HANDLER              Vector78
#define STM32_DMA1_CH5_HANDLER              Vector7C
#define STM32_DMA1_CH6_HANDLER              Vector80
#define STM32_DMA1_CH7_HANDLER              Vector84
#define STM32_DMA2_CH1_HANDLER              Vector11C
#define STM32_DMA2_CH2_HANDLER              Vector120
#define STM32_DMA2_CH3_HANDLER              Vector124
#define STM32_DMA2_CH4_HANDLER              Vector128
#define STM32_DMA2_CH5_HANDLER              Vector12C
#define STM32_DMA2_CH6_HANDLER              Vector130
#define STM32_DMA2_CH7_HANDLER              Vector134

#define STM32_DMA1_CH1_NUMBER               11
#define STM32_DMA1_CH2_NUMBER               12
#define STM32_DMA1_CH3_NUMBER               13
#define STM32_DMA1_CH4_NUMBER               14
#define STM32_DMA1_CH5_NUMBER               15
#define STM32_DMA1_CH6_NUMBER               16
#define STM32_DMA1_CH7_NUMBER               17
#define STM32_DMA2_CH1_NUMBER               55
#define STM32_DMA2_CH2_NUMBER               56
#define STM32_DMA2_CH3_NUMBER               57
#define STM32_DMA2_CH4_NUMBER               58
#define STM32_DMA2_CH5_NUMBER               59
#define STM32_DMA2_CH6_NUMBER               60
#define STM32_DMA2_CH7_NUMBER               61

/*
 * EXTI unit.
 */
#define STM32_EXTI0_HANDLER                 Vector58
#define STM32_EXTI1_HANDLER                 Vector5C
#define STM32_EXTI2_HANDLER                 Vector60
#define STM32_EXTI3_HANDLER                 Vector64
#define STM32_EXTI4_HANDLER                 Vector68
#define STM32_EXTI5_9_HANDLER               Vector9C
#define STM32_EXTI10_15_HANDLER             VectorE0
#define STM32_EXTI16_31_33_HANDLER          Vector44    /* PVD PVM0 PVM2    */
#define STM32_EXTI17_HANDLER                VectorE4    /* RTC ALARM        */
#define STM32_EXTI18_HANDLER                Vector48    /* RTC TAMP CSS     */
#define STM32_EXTI19_HANDLER                Vector4C    /* RTC WAKEUP       */
#define STM32_EXTI20_21_HANDLER             Vector98    /* COMP2 COMP1      */

#define STM32_EXTI0_NUMBER                  6
#define STM32_EXTI1_NUMBER                  7
#define STM32_EXTI2_NUMBER                  8
#define STM32_EXTI3_NUMBER                  9
#define STM32_EXTI4_NUMBER                  10
#define STM32_EXTI5_9_NUMBER                23
#define STM32_EXTI10_15_NUMBER              40
#define STM32_EXTI16_31_33_NUMBER           1
#define STM32_EXTI17_NUMBER                 41
#define STM32_EXTI18_NUMBER                 2
#define STM32_EXTI19_NUMBER                 3
#define STM32_EXTI20_21_NUMBER              22

/*
 * I2C units.
 */
#define STM32_I2C1_EVENT_HANDLER            VectorB8
#define STM32_I2C1_ERROR_HANDLER            VectorBC
#define STM32_I2C3_EVENT_HANDLER            VectorC0
#define STM32_I2C3_ERROR_HANDLER            VectorC4

#define STM32_I2C1_EVENT_NUMBER             30
#define STM32_I2C1_ERROR_NUMBER             31
#define STM32_I2C3_EVENT_NUMBER             32
#define STM32_I2C3_ERROR_NUMBER             33

/*
 * QUADSPI unit.
 */
#define STM32_QUADSPI1_HANDLER              Vector108

#define STM32_QUADSPI1_NUMBER               50

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_HANDLER              VectorA0
#define STM32_TIM1_UP_TIM16_HANDLER         VectorA4
#define STM32_TIM1_TRGCO_TIM17_HANDLER      VectorA8
#define STM32_TIM1_CC_HANDLER               VectorAC
#define STM32_TIM2_HANDLER                  VectorB0

#define STM32_TIM1_BRK_NUMBER               24
#define STM32_TIM1_UP_TIM16_NUMBER          25
#define STM32_TIM1_TRGCO_TIM17_NUMBER       26
#define STM32_TIM1_CC_NUMBER                27
#define STM32_TIM2_NUMBER                   28

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                VectorD0
#define STM32_LPUART1_HANDLER               VectorD4

#define STM32_USART1_NUMBER                 36
#define STM32_LPUART1_NUMBER                37

/*
 * USB unit.
 */
#define STM32_USB1_HP_HANDLER               Vector8C
#define STM32_USB1_LP_HANDLER               Vector90

#define STM32_USB1_HP_NUMBER                19
#define STM32_USB1_LP_NUMBER                20
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
