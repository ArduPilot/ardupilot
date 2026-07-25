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
 * @file    STM32WLxx/stm32_irs_m4.inc
 * @brief   STM32WLxx M4 core ISR handler header.
 *
 * @addtogroup SRM32WLxx_ISR
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @name    CM4 core ISR names and numbers
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
#define STM32_DMA2_CH1_HANDLER              Vector118
#define STM32_DMA2_CH2_HANDLER              Vector11C
#define STM32_DMA2_CH3_HANDLER              Vector120
#define STM32_DMA2_CH4_HANDLER              Vector124
#define STM32_DMA2_CH5_HANDLER              Vector128
#define STM32_DMA2_CH6_HANDLER              Vector12C
#define STM32_DMA2_CH7_HANDLER              Vector130

#define STM32_DMA1_CH1_NUMBER               11
#define STM32_DMA1_CH2_NUMBER               12
#define STM32_DMA1_CH3_NUMBER               13
#define STM32_DMA1_CH4_NUMBER               14
#define STM32_DMA1_CH5_NUMBER               15
#define STM32_DMA1_CH6_NUMBER               16
#define STM32_DMA1_CH7_NUMBER               17
#define STM32_DMA2_CH1_NUMBER               54
#define STM32_DMA2_CH2_NUMBER               55
#define STM32_DMA2_CH3_NUMBER               56
#define STM32_DMA2_CH4_NUMBER               57
#define STM32_DMA2_CH5_NUMBER               58
#define STM32_DMA2_CH6_NUMBER               59
#define STM32_DMA2_CH7_NUMBER               60

/*
 * EXTI unit.
 */
#define STM32_EXTI0_HANDLER                 Vector58
#define STM32_EXTI1_HANDLER                 Vector5C
#define STM32_EXTI2_HANDLER                 Vector60
#define STM32_EXTI3_HANDLER                 Vector64
#define STM32_EXTI4_HANDLER                 Vector68
#define STM32_EXTI5_9_HANDLER               Vector98
#define STM32_EXTI10_15_HANDLER             VectorE4
#define STM32_EXTI16_34_HANDLER             Vector44    /* PVD PVM3         */
#define STM32_EXTI21_22_HANDLER             Vector140   /* COMP1..2         */
#define STM32_EXTI45_HANDLER                Vector108   /* SG               */

#define STM32_EXTI0_NUMBER                  6
#define STM32_EXTI1_NUMBER                  7
#define STM32_EXTI2_NUMBER                  8
#define STM32_EXTI3_NUMBER                  9
#define STM32_EXTI4_NUMBER                  10
#define STM32_EXTI5_9_NUMBER                22
#define STM32_EXTI10_15_NUMBER              41
#define STM32_EXTI16_34_NUMBER              1
#define STM32_EXTI21_22_NUMBER              21
#define STM32_EXTI45_NUMBER                 50

/*
 * I2C units.
 */
#define STM32_I2C1_EVENT_HANDLER            VectorB8
#define STM32_I2C1_ERROR_HANDLER            VectorBC
#define STM32_I2C2_EVENT_HANDLER            VectorC0
#define STM32_I2C2_ERROR_HANDLER            VectorC4
#define STM32_I2C3_EVENT_HANDLER            Vector100
#define STM32_I2C3_ERROR_HANDLER            Vector104

#define STM32_I2C1_EVENT_NUMBER             30
#define STM32_I2C1_ERROR_NUMBER             31
#define STM32_I2C2_EVENT_NUMBER             32
#define STM32_I2C2_ERROR_NUMBER             33
#define STM32_I2C3_EVENT_NUMBER             48
#define STM32_I2C3_ERROR_NUMBER             49

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_HANDLER              Vector9C
#define STM32_TIM1_UP_HANDLER               VectorA0
#define STM32_TIM1_TRGCO_HANDLER            VectorA4
#define STM32_TIM1_CC_HANDLER               VectorA8
#define STM32_TIM2_HANDLER                  VectorAC
#define STM32_TIM16_HANDLER                 VectorB0
#define STM32_TIM17_HANDLER                 VectorB4

#define STM32_TIM1_BRK_NUMBER               23
#define STM32_TIM1_UP_NUMBER                24
#define STM32_TIM1_TRGCO_TIM17_NUMBER       25
#define STM32_TIM1_CC_NUMBER                26
#define STM32_TIM2_NUMBER                   27
#define STM32_TIM16_NUMBER                  28
#define STM32_TIM17_NUMBER                  29

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                VectorD0
#define STM32_USART2_HANDLER                VectorD4
#define STM32_LPUART1_HANDLER               VectorD8

#define STM32_USART1_NUMBER                 36
#define STM32_USART2_NUMBER                 37
#define STM32_LPUART1_NUMBER                38

/** @} */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
