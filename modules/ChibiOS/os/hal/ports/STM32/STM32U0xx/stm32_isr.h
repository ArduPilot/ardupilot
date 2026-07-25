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
 * @file    STM32U0xx/stm32_isr.h
 * @brief   STM32U0xx ISR handler header.
 *
 * @addtogroup STM32U0xx_ISR
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
#define STM32_TIM6_SUPPRESS_ISR
#define STM32_TIM7_SUPPRESS_ISR
#define STM32_TIM15_SUPPRESS_ISR
#define STM32_TIM16_SUPPRESS_ISR

#define STM32_USART1_SUPPRESS_ISR
#define STM32_USART2_SUPPRESS_ISR
#define STM32_USART3_SUPPRESS_ISR
#define STM32_UART4_SUPPRESS_ISR
#define STM32_LPUART1_SUPPRESS_ISR
#define STM32_LPUART2_SUPPRESS_ISR
#define STM32_LPUART3_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers
 * @{
 */
/*
 * ADC unit.
 */
#define STM32_ADC1_HANDLER                  Vector70
#define STM32_ADC1_NUMBER                   12

/*
 * DMA unit.
 */
#define STM32_DMA1_CH1_HANDLER              Vector64
#define STM32_DMA1_CH23_HANDLER             Vector68
#define STM32_DMA1_CH4567_HANDLER           Vector6C
#define STM32_DMA1_CH4567_DMA2_CH12345_HANDLER STM32_DMA1_CH4567_HANDLER

#define STM32_DMA1_CH1_NUMBER               9
#define STM32_DMA1_CH23_NUMBER              10
#define STM32_DMA1_CH2_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH3_NUMBER               STM32_DMA1_CH23_NUMBER
#define STM32_DMA1_CH4567_NUMBER            11
#define STM32_DMA1_CH4567_DMA2_CH12345_NUMBER STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH4_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH5_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH6_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA1_CH7_NUMBER               STM32_DMA1_CH4567_NUMBER
#define STM32_DMA2_CH1_NUMBER               STM32_DMA1_CH4567_DMA2_CH12345_NUMBER
#define STM32_DMA2_CH2_NUMBER               STM32_DMA1_CH4567_DMA2_CH12345_NUMBER
#define STM32_DMA2_CH3_NUMBER               STM32_DMA1_CH4567_DMA2_CH12345_NUMBER
#define STM32_DMA2_CH4_NUMBER               STM32_DMA1_CH4567_DMA2_CH12345_NUMBER
#define STM32_DMA2_CH5_NUMBER               STM32_DMA1_CH4567_DMA2_CH12345_NUMBER

/*
 * EXTI unit.
 */
#define STM32_EXTI0_1_HANDLER               Vector54
#define STM32_EXTI2_3_HANDLER               Vector58
#define STM32_EXTI4_15_HANDLER              Vector5C

#define STM32_EXTI0_1_NUMBER                5
#define STM32_EXTI2_3_NUMBER                6
#define STM32_EXTI4_15_NUMBER               7

/*
 * I2C units.
 */
#define STM32_I2C1_GLOBAL_HANDLER           Vector9C
#define STM32_I2C2_3_4_GLOBAL_HANDLER       VectorA0

#define STM32_I2C1_GLOBAL_NUMBER            23
#define STM32_I2C2_3_4_GLOBAL_NUMBER        24

/*
 * TIM units.
 */
#define STM32_TIM1_UP_HANDLER               Vector74
#define STM32_TIM1_CC_HANDLER               Vector78
#define STM32_TIM2_HANDLER                  Vector7C
#define STM32_TIM3_HANDLER                  Vector80
#define STM32_TIM6_HANDLER                  Vector84
#define STM32_TIM7_HANDLER                  Vector88
#define STM32_TIM14_HANDLER                 Vector8C
#define STM32_TIM15_HANDLER                 Vector90
#define STM32_TIM16_HANDLER                 Vector94
#define STM32_TIM17_HANDLER                 Vector98

#define STM32_TIM1_UP_NUMBER                13
#define STM32_TIM1_CC_NUMBER                14
#define STM32_TIM2_NUMBER                   15
#define STM32_TIM3_NUMBER                   16
#define STM32_TIM3_4_NUMBER                 STM32_TIM3_NUMBER
#define STM32_TIM6_NUMBER                   17
#define STM32_TIM7_NUMBER                   18
#define STM32_TIM15_NUMBER                  20
#define STM32_TIM16_NUMBER                  21

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                VectorAC
#define STM32_USART2_LP2_HANDLER            VectorB0
#define STM32_USART3_LP1_HANDLER            VectorB4
#define STM32_USART4_LP3_HANDLER            VectorB8

#define STM32_USART1_NUMBER                 27
#define STM32_USART2_LP2_NUMBER             28
#define STM32_USART3_LP1_NUMBER             29
#define STM32_USART4_LP3_NUMBER             30

/*
 * USB units.
 */
#define STM32_USB1_HANDLER                  Vector60
#define STM32_USB1_NUMBER                   8
/** @} */

/* Individual IRQ priorities are derived by the shared I2C IRQ priority,
   drivers expect individual definitions to exist.*/
#define STM32_IRQ_I2C2_PRIORITY             STM32_IRQ_I2C2_3_4_PRIORITY
#define STM32_IRQ_I2C3_PRIORITY             STM32_IRQ_I2C2_3_4_PRIORITY
#define STM32_IRQ_I2C4_PRIORITY             STM32_IRQ_I2C2_3_4_PRIORITY

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
