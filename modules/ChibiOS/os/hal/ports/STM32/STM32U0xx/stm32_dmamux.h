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
 * @file    STM32U0xx/stm32_dmamux.h
 * @brief   STM32U0xx DMAMUX handler header.
 *
 * @addtogroup STM32U0xx_DMAMUX
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
#define STM32_DMAMUX1_AES_IN        6
#define STM32_DMAMUX1_AES_OUT       7
#define STM32_DMAMUX1_DAC1_CH1      8
#define STM32_DMAMUX1_I2C1_RX       9
#define STM32_DMAMUX1_I2C1_TX       10
#define STM32_DMAMUX1_I2C2_RX       11
#define STM32_DMAMUX1_I2C2_TX       12
#define STM32_DMAMUX1_I2C3_RX       13
#define STM32_DMAMUX1_I2C3_TX       14
#define STM32_DMAMUX1_I2C4_RX       15
#define STM32_DMAMUX1_I2C4_TX       16
#define STM32_DMAMUX1_LPTIM1_IC1    17
#define STM32_DMAMUX1_LPTIM1_IC2    18
#define STM32_DMAMUX1_LPTIM1_IC3    19
#define STM32_DMAMUX1_LPTIM1_IC4    20
#define STM32_DMAMUX1_LPTIM1_UE     21
#define STM32_DMAMUX1_LPTIM2_IC1    22
#define STM32_DMAMUX1_LPTIM2_IC2    23
#define STM32_DMAMUX1_LPTIM2_UE     24
#define STM32_DMAMUX1_LPTIM3_IC1    25
#define STM32_DMAMUX1_LPTIM3_IC2    26
#define STM32_DMAMUX1_LPTIM3_IC3    27
#define STM32_DMAMUX1_LPTIM3_IC4    28
#define STM32_DMAMUX1_LPTIM3_UE     29
#define STM32_DMAMUX1_LPUART1_RX    30
#define STM32_DMAMUX1_LPUART1_TX    31
#define STM32_DMAMUX1_LPUART2_RX    32
#define STM32_DMAMUX1_LPUART2_TX    33
#define STM32_DMAMUX1_LPUART3_RX    34
#define STM32_DMAMUX1_LPUART3_TX    35
#define STM32_DMAMUX1_SPI1_RX       36
#define STM32_DMAMUX1_SPI1_TX       37
#define STM32_DMAMUX1_SPI2_RX       38
#define STM32_DMAMUX1_SPI2_TX       39
#define STM32_DMAMUX1_SPI3_RX       40
#define STM32_DMAMUX1_SPI3_TX       41
#define STM32_DMAMUX1_TIM1_CH1      42
#define STM32_DMAMUX1_TIM1_CH2      43
#define STM32_DMAMUX1_TIM1_CH3      44
#define STM32_DMAMUX1_TIM1_CH4      45
#define STM32_DMAMUX1_TIM1_COM      46
#define STM32_DMAMUX1_TIM1_UP       47
#define STM32_DMAMUX1_TIM2_CH1      48
#define STM32_DMAMUX1_TIM2_CH2      49
#define STM32_DMAMUX1_TIM2_CH3      50
#define STM32_DMAMUX1_TIM2_CH4      51
#define STM32_DMAMUX1_TIM2_TRIG     52
#define STM32_DMAMUX1_TIM2_UP       53
#define STM32_DMAMUX1_TIM3_CH1      54
#define STM32_DMAMUX1_TIM3_CH2      55
#define STM32_DMAMUX1_TIM3_CH3      56
#define STM32_DMAMUX1_TIM3_CH4      57
#define STM32_DMAMUX1_TIM3_TRIG     58
#define STM32_DMAMUX1_TIM3_UP       59
#define STM32_DMAMUX1_TIM6_UP       60
#define STM32_DMAMUX1_TIM7_UP       61
#define STM32_DMAMUX1_TIM15_CH1     62
#define STM32_DMAMUX1_TIM15_CH2     63
#define STM32_DMAMUX1_TIM15_COM     64
#define STM32_DMAMUX1_TIM15_UP      65
#define STM32_DMAMUX1_TIM16_CH1     66
#define STM32_DMAMUX1_TIM16_COM     67
#define STM32_DMAMUX1_TIM16_UP      68
#define STM32_DMAMUX1_USART1_RX     69
#define STM32_DMAMUX1_USART1_TX     70
#define STM32_DMAMUX1_USART2_RX     71
#define STM32_DMAMUX1_USART2_TX     72
#define STM32_DMAMUX1_USART3_RX     73
#define STM32_DMAMUX1_USART3_TX     74
#define STM32_DMAMUX1_USART4_RX     75
#define STM32_DMAMUX1_USART4_TX     76
#define STM32_DMAMUX1_UART4_RX      STM32_DMAMUX1_USART4_RX /* Legacy.      */
#define STM32_DMAMUX1_UART4_TX      STM32_DMAMUX1_USART4_TX /* Legacy.      */
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
