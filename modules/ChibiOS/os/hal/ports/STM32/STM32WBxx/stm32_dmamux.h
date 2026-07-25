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
 * @file    STM32WBxx/stm32_dmamux.h
 * @brief   STM32WBxx DMAMUX handler header.
 *
 * @addtogroup STM32WBxxp_DMAMUX
 * @{
 */

#ifndef STM32_DMAMUX_H
#define STM32_DMAMUX_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    DMAMUX1 request sources
 * @{
 */
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_ADC1          5
#define STM32_DMAMUX1_SPI1_RX       6
#define STM32_DMAMUX1_SPI1_TX       7
#define STM32_DMAMUX1_SPI2_RX       8
#define STM32_DMAMUX1_SPI2_TX       9
#define STM32_DMAMUX1_I2C1_RX       10
#define STM32_DMAMUX1_I2C1_TX       11
#define STM32_DMAMUX1_I2C3_RX       12
#define STM32_DMAMUX1_I2C3_TX       13
#define STM32_DMAMUX1_USART1_RX     14
#define STM32_DMAMUX1_USART1_TX     15
#define STM32_DMAMUX1_LPUART1_RX    16
#define STM32_DMAMUX1_LPUART1_TX    17
#define STM32_DMAMUX1_SAI1_A        18
#define STM32_DMAMUX1_SAI1_B        19
#define STM32_DMAMUX1_QUADSPI       20
#define STM32_DMAMUX1_TIM1_CH1      21
#define STM32_DMAMUX1_TIM1_CH2      22
#define STM32_DMAMUX1_TIM1_CH3      23
#define STM32_DMAMUX1_TIM1_CH4      24
#define STM32_DMAMUX1_TIM1_UP       25
#define STM32_DMAMUX1_TIM1_TRIG     26
#define STM32_DMAMUX1_TIM1_COM      27
#define STM32_DMAMUX1_TIM2_CH1      28
#define STM32_DMAMUX1_TIM2_CH2      29
#define STM32_DMAMUX1_TIM2_CH3      30
#define STM32_DMAMUX1_TIM2_CH4      31
#define STM32_DMAMUX1_TIM2_UP       32
#define STM32_DMAMUX1_TIM16_CH1     33
#define STM32_DMAMUX1_TIM16_UP      34
#define STM32_DMAMUX1_TIM17_CH1     35
#define STM32_DMAMUX1_TIM17_UP      36
#define STM32_DMAMUX1_CRYP_IN       37
#define STM32_DMAMUX1_CRYP_OUT      38
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

#ifdef __cplusplus
}
#endif

#endif /* STM32_DMAMUX_H */

/** @} */
