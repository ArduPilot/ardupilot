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
 * @file    STM32L4xx+/stm32_dmamux.h
 * @brief   STM32L4xx+ DMAMUX handler header.
 *
 * @addtogroup STM32L4xxp_DMAMUX
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
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_ADC1          5
#define STM32_DMAMUX1_ADC2          6
#define STM32_DMAMUX1_DAC1_CH1      7
#define STM32_DMAMUX1_DAC1_CH2      8
#define STM32_DMAMUX1_TIM6_UP       9
#define STM32_DMAMUX1_TIM7_UP       10
#define STM32_DMAMUX1_SPI1_RX       11
#define STM32_DMAMUX1_SPI1_TX       12
#define STM32_DMAMUX1_SPI2_RX       13
#define STM32_DMAMUX1_SPI2_TX       14
#define STM32_DMAMUX1_SPI3_RX       15
#define STM32_DMAMUX1_SPI3_TX       16
#define STM32_DMAMUX1_I2C1_RX       17
#define STM32_DMAMUX1_I2C1_TX       18
#define STM32_DMAMUX1_I2C2_RX       19
#define STM32_DMAMUX1_I2C2_TX       20
#define STM32_DMAMUX1_I2C3_RX       21
#define STM32_DMAMUX1_I2C3_TX       22
#define STM32_DMAMUX1_I2C4_RX       23
#define STM32_DMAMUX1_I2C4_TX       24
#define STM32_DMAMUX1_USART1_RX     25
#define STM32_DMAMUX1_USART1_TX     26
#define STM32_DMAMUX1_USART2_RX     27
#define STM32_DMAMUX1_USART2_TX     28
#define STM32_DMAMUX1_USART3_RX     29
#define STM32_DMAMUX1_USART3_TX     30
#define STM32_DMAMUX1_UART4_RX      31
#define STM32_DMAMUX1_UART4_TX      32
#define STM32_DMAMUX1_UART5_RX      33
#define STM32_DMAMUX1_UART5_TX      34
#define STM32_DMAMUX1_LPUART1_RX    35
#define STM32_DMAMUX1_LPUART1_TX    36
#define STM32_DMAMUX1_SAI1_A        37
#define STM32_DMAMUX1_SAI1_B        38
#define STM32_DMAMUX1_SAI2_A        39
#define STM32_DMAMUX1_SAI2_B        40
#define STM32_DMAMUX1_OCTOSPI1      41
#define STM32_DMAMUX1_OCTOSPI2      42
#define STM32_DMAMUX1_TIM1_CH1      43
#define STM32_DMAMUX1_TIM1_CH2      44
#define STM32_DMAMUX1_TIM1_CH3      45
#define STM32_DMAMUX1_TIM1_CH4      46
#define STM32_DMAMUX1_TIM1_UP       47
#define STM32_DMAMUX1_TIM1_TRIG     48
#define STM32_DMAMUX1_TIM1_COM      49
#define STM32_DMAMUX1_TIM8_CH1      50
#define STM32_DMAMUX1_TIM8_CH2      51
#define STM32_DMAMUX1_TIM8_CH3      52
#define STM32_DMAMUX1_TIM8_CH4      53
#define STM32_DMAMUX1_TIM8_UP       54
#define STM32_DMAMUX1_TIM8_TRIG     55
#define STM32_DMAMUX1_TIM8_COM      56
#define STM32_DMAMUX1_TIM2_CH1      57
#define STM32_DMAMUX1_TIM2_CH2      58
#define STM32_DMAMUX1_TIM2_CH3      59
#define STM32_DMAMUX1_TIM2_CH4      60
#define STM32_DMAMUX1_TIM2_UP       61
#define STM32_DMAMUX1_TIM3_CH1      62
#define STM32_DMAMUX1_TIM3_CH2      63
#define STM32_DMAMUX1_TIM3_CH3      64
#define STM32_DMAMUX1_TIM3_CH4      65
#define STM32_DMAMUX1_TIM3_UP       66
#define STM32_DMAMUX1_TIM3_TRIG     67
#define STM32_DMAMUX1_TIM4_CH1      68
#define STM32_DMAMUX1_TIM4_CH2      69
#define STM32_DMAMUX1_TIM4_CH3      70
#define STM32_DMAMUX1_TIM4_CH4      71
#define STM32_DMAMUX1_TIM4_UP       72
#define STM32_DMAMUX1_TIM5_CH1      73
#define STM32_DMAMUX1_TIM5_CH2      74
#define STM32_DMAMUX1_TIM5_CH3      75
#define STM32_DMAMUX1_TIM5_CH4      76
#define STM32_DMAMUX1_TIM5_UP       77
#define STM32_DMAMUX1_TIM5_TRIG     78
#define STM32_DMAMUX1_TIM15_CH1     79
#define STM32_DMAMUX1_TIM15_UP      80
#define STM32_DMAMUX1_TIM15_TRIG    81
#define STM32_DMAMUX1_TIM15_COM     82
#define STM32_DMAMUX1_TIM16_CH1     83
#define STM32_DMAMUX1_TIM16_UP      84
#define STM32_DMAMUX1_TIM17_CH1     85
#define STM32_DMAMUX1_TIM17_UP      86
#define STM32_DMAMUX1_DFSDM1_FLT0   87
#define STM32_DMAMUX1_DFSDM1_FLT1   88
#define STM32_DMAMUX1_DFSDM1_FLT2   89
#define STM32_DMAMUX1_DFSDM1_FLT3   90
#define STM32_DMAMUX1_DCMI          91
#define STM32_DMAMUX1_AES_IN        92
#define STM32_DMAMUX1_AES_OUT       93
#define STM32_DMAMUX1_HASH_IN       94
#else
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_ADC1          5
#define STM32_DMAMUX1_DAC1_CH1      6
#define STM32_DMAMUX1_DAC1_CH2      7
#define STM32_DMAMUX1_TIM6_UP       8
#define STM32_DMAMUX1_TIM7_UP       9
#define STM32_DMAMUX1_SPI1_RX       10
#define STM32_DMAMUX1_SPI1_TX       11
#define STM32_DMAMUX1_SPI2_RX       12
#define STM32_DMAMUX1_SPI2_TX       13
#define STM32_DMAMUX1_SPI3_RX       14
#define STM32_DMAMUX1_SPI3_TX       15
#define STM32_DMAMUX1_I2C1_RX       16
#define STM32_DMAMUX1_I2C1_TX       17
#define STM32_DMAMUX1_I2C2_RX       18
#define STM32_DMAMUX1_I2C2_TX       19
#define STM32_DMAMUX1_I2C3_RX       20
#define STM32_DMAMUX1_I2C3_TX       21
#define STM32_DMAMUX1_I2C4_RX       22
#define STM32_DMAMUX1_I2C4_TX       23
#define STM32_DMAMUX1_USART1_RX     24
#define STM32_DMAMUX1_USART1_TX     25
#define STM32_DMAMUX1_USART2_RX     26
#define STM32_DMAMUX1_USART2_TX     27
#define STM32_DMAMUX1_USART3_RX     28
#define STM32_DMAMUX1_USART3_TX     29
#define STM32_DMAMUX1_UART4_RX      30
#define STM32_DMAMUX1_UART4_TX      31
#define STM32_DMAMUX1_UART5_RX      32
#define STM32_DMAMUX1_UART5_TX      33
#define STM32_DMAMUX1_LPUART1_RX    34
#define STM32_DMAMUX1_LPUART1_TX    35
#define STM32_DMAMUX1_SAI1_A        36
#define STM32_DMAMUX1_SAI1_B        37
#define STM32_DMAMUX1_SAI2_A        38
#define STM32_DMAMUX1_SAI2_B        39
#define STM32_DMAMUX1_OCTOSPI1      40
#define STM32_DMAMUX1_OCTOSPI2      41
#define STM32_DMAMUX1_TIM1_CH1      42
#define STM32_DMAMUX1_TIM1_CH2      43
#define STM32_DMAMUX1_TIM1_CH3      44
#define STM32_DMAMUX1_TIM1_CH4      45
#define STM32_DMAMUX1_TIM1_UP       46
#define STM32_DMAMUX1_TIM1_TRIG     47
#define STM32_DMAMUX1_TIM1_COM      48
#define STM32_DMAMUX1_TIM8_CH1      49
#define STM32_DMAMUX1_TIM8_CH2      50
#define STM32_DMAMUX1_TIM8_CH3      51
#define STM32_DMAMUX1_TIM8_CH4      52
#define STM32_DMAMUX1_TIM8_UP       53
#define STM32_DMAMUX1_TIM8_TRIG     54
#define STM32_DMAMUX1_TIM8_COM      55
#define STM32_DMAMUX1_TIM2_CH1      56
#define STM32_DMAMUX1_TIM2_CH2      57
#define STM32_DMAMUX1_TIM2_CH3      58
#define STM32_DMAMUX1_TIM2_CH4      59
#define STM32_DMAMUX1_TIM2_UP       60
#define STM32_DMAMUX1_TIM3_CH1      61
#define STM32_DMAMUX1_TIM3_CH2      62
#define STM32_DMAMUX1_TIM3_CH3      63
#define STM32_DMAMUX1_TIM3_CH4      64
#define STM32_DMAMUX1_TIM3_UP       65
#define STM32_DMAMUX1_TIM3_TRIG     66
#define STM32_DMAMUX1_TIM4_CH1      67
#define STM32_DMAMUX1_TIM4_CH2      68
#define STM32_DMAMUX1_TIM4_CH3      69
#define STM32_DMAMUX1_TIM4_CH4      70
#define STM32_DMAMUX1_TIM4_UP       71
#define STM32_DMAMUX1_TIM5_CH1      72
#define STM32_DMAMUX1_TIM5_CH2      73
#define STM32_DMAMUX1_TIM5_CH3      74
#define STM32_DMAMUX1_TIM5_CH4      75
#define STM32_DMAMUX1_TIM5_UP       76
#define STM32_DMAMUX1_TIM5_TRIG     77
#define STM32_DMAMUX1_TIM15_CH1     78
#define STM32_DMAMUX1_TIM15_UP      79
#define STM32_DMAMUX1_TIM15_TRIG    80
#define STM32_DMAMUX1_TIM15_COM     81
#define STM32_DMAMUX1_TIM16_CH1     82
#define STM32_DMAMUX1_TIM16_UP      83
#define STM32_DMAMUX1_TIM17_CH1     84
#define STM32_DMAMUX1_TIM17_UP      85
#define STM32_DMAMUX1_DFSDM1_FLT0   86
#define STM32_DMAMUX1_DFSDM1_FLT1   87
#define STM32_DMAMUX1_DFSDM1_FLT2   88
#define STM32_DMAMUX1_DFSDM1_FLT3   89
#define STM32_DMAMUX1_DCMI          90
#define STM32_DMAMUX1_AES_IN        91
#define STM32_DMAMUX1_AES_OUT       92
#define STM32_DMAMUX1_HASH_IN       93
#endif /* defined(STM32L4P5xx) || defined(STM32L4Q5xx) */
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
