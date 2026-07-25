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
 * @file    STM32C0xx/stm32_dmamux.h
 * @brief   STM32C0xx DMAMUX handler header.
 *
 * @addtogroup STM32C0xx_DMAMUX
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
#define STM32_DMAMUX1_I2C1_RX       10
#define STM32_DMAMUX1_I2C1_TX       11
#define STM32_DMAMUX1_SPI1_RX       16
#define STM32_DMAMUX1_SPI1_TX       17
#define STM32_DMAMUX1_TIM1_CH1      20
#define STM32_DMAMUX1_TIM1_CH2      21
#define STM32_DMAMUX1_TIM1_CH3      22
#define STM32_DMAMUX1_TIM1_CH4      23
#define STM32_DMAMUX1_TIM1_COM      24
#define STM32_DMAMUX1_TIM1_UP       25
#define STM32_DMAMUX1_TIM3_CH1      32
#define STM32_DMAMUX1_TIM3_CH2      33
#define STM32_DMAMUX1_TIM3_CH3      34
#define STM32_DMAMUX1_TIM3_CH4      35
#define STM32_DMAMUX1_TIM3_TRIG     36
#define STM32_DMAMUX1_TIM3_UP       37
#define STM32_DMAMUX1_TIM16_CH1     44
#define STM32_DMAMUX1_TIM16_COM     45
#define STM32_DMAMUX1_TIM16_UP      46
#define STM32_DMAMUX1_TIM17_CH1     47
#define STM32_DMAMUX1_TIM17_COM     48
#define STM32_DMAMUX1_TIM17_UP      49
#define STM32_DMAMUX1_USART1_RX     50
#define STM32_DMAMUX1_USART1_TX     51
#define STM32_DMAMUX1_USART2_RX     52
#define STM32_DMAMUX1_USART2_TX     53
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
