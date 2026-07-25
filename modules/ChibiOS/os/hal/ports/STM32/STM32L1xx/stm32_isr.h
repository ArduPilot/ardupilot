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
 * @file    STM32L1xx/stm32_isr.h
 * @brief   STM32L1xx ISR handler header.
 *
 * @addtogroup STM32L1xx_ISR
 * @{
 */

#ifndef STM32_ISR_H
#define STM32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */
/*
 * I2C units.
 */
#define STM32_I2C1_EVENT_HANDLER    VectorBC
#define STM32_I2C1_ERROR_HANDLER    VectorC0
#define STM32_I2C1_EVENT_NUMBER     31
#define STM32_I2C1_ERROR_NUMBER     32

#define STM32_I2C2_EVENT_HANDLER    VectorC4
#define STM32_I2C2_ERROR_HANDLER    VectorC8
#define STM32_I2C2_EVENT_NUMBER     33
#define STM32_I2C2_ERROR_NUMBER     34

/*
 * TIM units.
 */
#define STM32_TIM2_HANDLER          VectorB0
#define STM32_TIM3_HANDLER          VectorB4
#define STM32_TIM4_HANDLER          VectorB8
#define STM32_TIM5_HANDLER          VectorF8
#define STM32_TIM6_HANDLER          VectorEC
#define STM32_TIM7_HANDLER          VectorF0
#define STM32_TIM9_HANDLER          VectorA4
#define STM32_TIM10_HANDLER         VectorA8
#define STM32_TIM11_HANDLER         VectorAC

#define STM32_TIM2_NUMBER           28
#define STM32_TIM3_NUMBER           29
#define STM32_TIM4_NUMBER           30
#define STM32_TIM5_NUMBER           46
#define STM32_TIM6_NUMBER           43
#define STM32_TIM7_NUMBER           44
#define STM32_TIM9_NUMBER           25
#define STM32_TIM10_NUMBER          26
#define STM32_TIM11_NUMBER          27

/*
 * USART units.
 */
#define STM32_USART1_HANDLER        VectorD4
#define STM32_USART2_HANDLER        VectorD8
#define STM32_USART3_HANDLER        VectorDC
#define STM32_UART4_HANDLER         Vector100
#define STM32_UART5_HANDLER         Vector104

#define STM32_USART1_NUMBER         37
#define STM32_USART2_NUMBER         38
#define STM32_USART3_NUMBER         39
#define STM32_UART4_NUMBER          48
#define STM32_UART5_NUMBER          49
/*
 * USB units.
 */
#define STM32_USB1_HP_HANDLER       Vector8C
#define STM32_USB1_LP_HANDLER       Vector90

#define STM32_USB1_HP_NUMBER        19
#define STM32_USB1_LP_NUMBER        20
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EXTI0 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI0_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI0_PRIORITY            6
#endif

/**
 * @brief   EXTI1 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI1_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI1_PRIORITY            6
#endif

/**
 * @brief   EXTI2 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI2_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI2_PRIORITY            6
#endif

/**
 * @brief   EXTI3 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI3_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI3_PRIORITY            6
#endif

/**
 * @brief   EXTI4 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI4_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI4_PRIORITY            6
#endif

/**
 * @brief   EXTI9..5 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI5_9_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#endif

/**
 * @brief   EXTI15..10 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI10_15_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI10_15_PRIORITY        6
#endif

/**
 * @brief   EXTI16 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI16_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI16_PRIORITY           6
#endif

/**
 * @brief   EXTI17 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI17_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI17_PRIORITY           6
#endif

/**
 * @brief   EXTI18 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI18_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI18_PRIORITY           6
#endif

/**
 * @brief   EXTI19 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI19_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI19_PRIORITY           6
#endif

/**
 * @brief   EXTI20 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI20_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI20_PRIORITY           6
#endif

/**
 * @brief   EXTI21..22 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI21_22_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI21_22_PRIORITY        6
#endif

/**
 * @brief   EXTI23 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI23_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI23_PRIORITY           6
#endif
/** @} */

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
