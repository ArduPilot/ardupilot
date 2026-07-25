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
 * @file    STM32WLxx/stm32_isr_m0.inc
 * @brief   STM32WLxx M0 core ISR handler header.
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
 * @name    CM0+ core ISR names and numbers
 * @{
 */

/*
 * ADC unit.
 */
#define STM32_ADC1_HANDLER                  Vector5C
#define STM32_ADC1_NUMBER                   7

/*
 * DMA unit.
 */
#define STM32_DMA1_CH123_HANDLER            Vector60
#define STM32_DMA1_CH4567_HANDLER           Vector64
#define STM32_DMA2_CH1_7_HANDLER            Vector68

#define STM32_DMA1_CH123_NUMBER             8
#define STM32_DMA1_CH4567_NUMBER            9
#define STM32_DMA2_CH1_7_NUMBER             10

/*
 * EXTI unit.
 */
#define STM32_EXTI0_1_HANDLER               Vector50
#define STM32_EXTI2_3_HANDLER               Vector54
#define STM32_EXTI4_15_HANDLER              Vector58
#define STM32_EXTI16_34_HANDLER             Vector44    /* PVD PVM3         */
#define STM32_EXTI21_22_HANDLER             Vector5C    /* COMP1..2         */
#define STM32_EXTI45_HANDLER                VectorBC    /* SG               */

#define STM32_EXTI0_1_NUMBER                4
#define STM32_EXTI2_3_NUMBER                5
#define STM32_EXTI4_15_NUMBER               6
#define STM32_EXTI16_34_NUMBER              1
#define STM32_EXTI21_22_NUMBER              7
#define STM32_EXTI45_NUMBER                 31

/*
 * I2C units.
 */
#define STM32_I2C1_GLOBAL_HANDLER           Vector98
#define STM32_I2C2_GLOBAL_HANDLER           Vector9C
#define STM32_I2C3_GLOBAL_HANDLER           VectorA0

#define STM32_I2C1_GLOBAL_NUMBER            22
#define STM32_I2C2_GLOBAL_NUMBER            23
#define STM32_I2C3_GLOBAL_NUMBER            24

/*
 * TIM units.
 */
#define STM32_TIM1_SHARED_HANDLER           Vector78
#define STM32_TIM2_HANDLER                  Vector7C
#define STM32_TIM16_HANDLER                 Vector80
#define STM32_TIM17_HANDLER                 Vector84

#define STM32_TIM1_NUMBER                   14
#define STM32_TIM2_NUMBER                   15
#define STM32_TIM16_NUMBER                  16
#define STM32_TIM17_NUMBER                  17

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                VectorAC
#define STM32_USART2_HANDLER                VectorB0
#define STM32_LPUART1_HANDLER               VectorB4

#define STM32_USART1_NUMBER                 27
#define STM32_USART2_NUMBER                 28
#define STM32_LPUART1_NUMBER                29

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
