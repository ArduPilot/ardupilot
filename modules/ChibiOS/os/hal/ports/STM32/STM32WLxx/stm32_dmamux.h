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
 * @file    STM32WLxx/stm32_dmamux.h
 * @brief   STM32WLxx DMAMUX handler header.
 *
 * @addtogroup STM32WLxx_DMAMUX
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
#define STM32_DMAMUX1_DAC1_CH1      6
#define STM32_DMAMUX1_SPI1_RX       7
#define STM32_DMAMUX1_SPI1_TX       8
#define STM32_DMAMUX1_SPI2_RX       9
#define STM32_DMAMUX1_SPI2_TX       10
#define STM32_DMAMUX1_I2C1_RX       11
#define STM32_DMAMUX1_I2C1_TX       12
#define STM32_DMAMUX1_I2C2_RX       13
#define STM32_DMAMUX1_I2C2_TX       14
#define STM32_DMAMUX1_I2C3_RX       15
#define STM32_DMAMUX1_I2C3_TX       16
#define STM32_DMAMUX1_USART1_RX     17
#define STM32_DMAMUX1_USART1_TX     18
#define STM32_DMAMUX1_USART2_RX     19
#define STM32_DMAMUX1_USART2_TX     20
#define STM32_DMAMUX1_LPUART1_RX    21
#define STM32_DMAMUX1_LPUART1_TX    22
#define STM32_DMAMUX1_TIM1_CH1      23
#define STM32_DMAMUX1_TIM1_CH2      24
#define STM32_DMAMUX1_TIM1_CH3      25
#define STM32_DMAMUX1_TIM1_CH4      26
#define STM32_DMAMUX1_TIM1_UP       27
#define STM32_DMAMUX1_TIM1_TRIG     28
#define STM32_DMAMUX1_TIM1_COM      29
#define STM32_DMAMUX1_TIM2_CH1      30
#define STM32_DMAMUX1_TIM2_CH2      31
#define STM32_DMAMUX1_TIM2_CH3      32
#define STM32_DMAMUX1_TIM2_CH4      33
#define STM32_DMAMUX1_TIM2_UP       34
#define STM32_DMAMUX1_TIM16_CH1     35
#define STM32_DMAMUX1_TIM16_UP      36
#define STM32_DMAMUX1_TIM17_CH1     37
#define STM32_DMAMUX1_TIM17_UP      38
#define STM32_DMAMUX1_AES_IN        39
#define STM32_DMAMUX1_AES_OUT       40
#define STM32_DMAMUX1_SPI3_RX       41
#define STM32_DMAMUX1_SPI3_TX       42
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
