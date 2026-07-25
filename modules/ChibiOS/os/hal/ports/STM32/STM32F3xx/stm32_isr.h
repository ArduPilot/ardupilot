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
 * @file    STM32F3xx/stm32_isr.h
 * @brief   STM32F3xx ISR handler header.
 *
 * @addtogroup STM32F3xx_ISR
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
#define STM32_TIM15_SUPPRESS_ISR
#define STM32_TIM16_SUPPRESS_ISR
#define STM32_TIM17_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers remapping
 * @{
 */
/*
 * CAN units.
 */
#define STM32_CAN1_TX_HANDLER       Vector8C
#define STM32_CAN1_RX0_HANDLER      Vector90
#define STM32_CAN1_RX1_HANDLER      Vector94
#define STM32_CAN1_SCE_HANDLER      Vector98

#define STM32_CAN1_TX_NUMBER        19
#define STM32_CAN1_RX0_NUMBER       20
#define STM32_CAN1_RX1_NUMBER       21
#define STM32_CAN1_SCE_NUMBER       22

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

#define STM32_I2C3_EVENT_HANDLER    Vector160
#define STM32_I2C3_ERROR_HANDLER    Vector164
#define STM32_I2C3_EVENT_NUMBER     72
#define STM32_I2C3_ERROR_NUMBER     73

/*
 * TIM units.
 */
#define STM32_TIM1_UP_HANDLER       VectorA4
#define STM32_TIM1_CC_HANDLER       VectorAC
#define STM32_TIM2_HANDLER          VectorB0
#define STM32_TIM3_HANDLER          VectorB4
#define STM32_TIM4_HANDLER          VectorB8
#define STM32_TIM6_HANDLER          Vector118
#define STM32_TIM7_HANDLER          Vector11C
#define STM32_TIM8_UP_HANDLER       VectorF0
#define STM32_TIM8_CC_HANDLER       VectorF8
#define STM32_TIM15_HANDLER         VectorA0 /* Note: same as STM32_TIM1_BRK */
#define STM32_TIM16_HANDLER         VectorA4 /* Note: same as STM32_TIM1_UP */
#define STM32_TIM17_HANDLER         VectorA8 /* Note: same as STM32_TIM1_TRG_COM */
#define STM32_TIM20_UP_HANDLER      Vector178
#define STM32_TIM20_CC_HANDLER      Vector180

#define STM32_TIM1_UP_NUMBER        25
#define STM32_TIM1_CC_NUMBER        27
#define STM32_TIM2_NUMBER           28
#define STM32_TIM3_NUMBER           29
#define STM32_TIM4_NUMBER           30
#define STM32_TIM6_NUMBER           54
#define STM32_TIM7_NUMBER           55
#define STM32_TIM8_UP_NUMBER        44
#define STM32_TIM8_CC_NUMBER        46
#define STM32_TIM15_NUMBER          24 /* Note: same as STM32_TIM1_BRK */
#define STM32_TIM16_NUMBER          25 /* Note: same as STM32_TIM1_UP */
#define STM32_TIM17_NUMBER          26 /* Note: same as STM32_TIM1_TRG_COM */
#define STM32_TIM20_UP_NUMBER       78
#define STM32_TIM20_CC_NUMBER       80

/*
 * HRTIM units (F334)
 */
#define STM32_HRTIM_MASTER_HANDLER  Vector14C
#define STM32_HRTIM_TIMA_HANDLER    Vector150
#define STM32_HRTIM_TIMB_HANDLER    Vector154
#define STM32_HRTIM_TIMC_HANDLER    Vector158
#define STM32_HRTIM_TIMD_HANDLER    Vector15C
#define STM32_HRTIM_TIME_HANDLER    Vector160
#define STM32_HRTIM_FLT_HANDLER     Vector164

#define STM32_HRTIM_MASTER_NUMBER   67
#define STM32_HRTIM_TIMA_NUMBER     68
#define STM32_HRTIM_TIMB_NUMBER     69
#define STM32_HRTIM_TIMC_NUMBER     70
#define STM32_HRTIM_TIMD_NUMBER     71
#define STM32_HRTIM_TIME_NUMBER     72
#define STM32_HRTIM_FLT_NUMBER      73

/*
 * USART units.
 */
#define STM32_USART1_HANDLER        VectorD4
#define STM32_USART2_HANDLER        VectorD8
#define STM32_USART3_HANDLER        VectorDC
#define STM32_UART4_HANDLER         Vector110
#define STM32_UART5_HANDLER         Vector114

#define STM32_USART1_NUMBER         37
#define STM32_USART2_NUMBER         38
#define STM32_USART3_NUMBER         39
#define STM32_UART4_NUMBER          52
#define STM32_UART5_NUMBER          53

/*
 * USB units.
 */
#define STM32_USB1_HP_HANDLER       Vector168
#define STM32_USB1_LP_HANDLER       Vector16C

#define STM32_USB1_HP_NUMBER        74
#define STM32_USB1_LP_NUMBER        75
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
 * @brief   EXTI5..9 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI5_9_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI5_9_PRIORITY          6
#endif

/**
 * @brief   EXTI10..15 interrupt priority level setting.
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
 * @brief   EXTI21,22,29 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI21_22_29_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI21_22_29_PRIORITY     6
#endif

/**
 * @brief   EXTI30..32 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI30_32_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI30_32_PRIORITY        6
#endif

/**
 * @brief   EXTI33 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_EXTI33_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_EXTI33_PRIORITY           6
#endif

/**
 * @brief   TIM1-BRK, TIM15 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_TIM1_BRK_TIM15_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_TIM1_BRK_TIM15_PRIORITY   7
#endif

/**
 * @brief   TIM1-UP, TIM16 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_TIM1_UP_TIM16_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_TIM1_UP_TIM16_PRIORITY    7
#endif

/**
 * @brief   TIM1-TRG-COM, TIM17 interrupt priority level setting.
 */
#if !defined(STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY 7
#endif

/**
 * @brief   TIM1-CC interrupt priority level setting.
 */
#if !defined(STM32_IRQ_TIM1_CC_PRIORITY) || defined(__DOXYGEN__)
#define STM32_IRQ_TIM1_CC_PRIORITY          7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* IRQ priority checks.*/
#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI0_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI0_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI1_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI1_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI2_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI2_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI3_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI3_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI4_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI4_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI5_9_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI5_9_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI10_15_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI10_15_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI16_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI16_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI17_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI17_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI18_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI18_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI19_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI19_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI20_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI20_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI21_22_29_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI21_22_29_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI30_32_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI30_32_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_EXTI33_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_EXTI33_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM1_BRK_TIM15_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM1_BRK_TIM15_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM1_UP_TIM16_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM1_UP_TIM16_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM1_TRGCO_TIM17_PRIORITY"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_TIM1_CC_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_TIM1_CC_PRIORITY"
#endif

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
