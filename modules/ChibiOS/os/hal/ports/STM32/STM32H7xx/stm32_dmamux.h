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
 * @file    STM32H7xx/stm32_dmamux.h
 * @brief   STM32H7xx DMAMUX handler header.
 *
 * @addtogroup STM32H7xx_DMAMUX
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
#define STM32_DMAMUX1_REQ_GEN4      5
#define STM32_DMAMUX1_REQ_GEN5      6
#define STM32_DMAMUX1_REQ_GEN6      7
#define STM32_DMAMUX1_REQ_GEN7      8
#define STM32_DMAMUX1_ADC1          9
#define STM32_DMAMUX1_ADC2          10
#define STM32_DMAMUX1_TIM1_CH1      11
#define STM32_DMAMUX1_TIM1_CH2      12
#define STM32_DMAMUX1_TIM1_CH3      13
#define STM32_DMAMUX1_TIM1_CH4      14
#define STM32_DMAMUX1_TIM1_UP       15
#define STM32_DMAMUX1_TIM1_TRIG     16
#define STM32_DMAMUX1_TIM1_COM      17
#define STM32_DMAMUX1_TIM2_CH1      18
#define STM32_DMAMUX1_TIM2_CH2      19
#define STM32_DMAMUX1_TIM2_CH3      20
#define STM32_DMAMUX1_TIM2_CH4      21
#define STM32_DMAMUX1_TIM2_UP       22
#define STM32_DMAMUX1_TIM3_CH1      23
#define STM32_DMAMUX1_TIM3_CH2      24
#define STM32_DMAMUX1_TIM3_CH3      25
#define STM32_DMAMUX1_TIM3_CH4      26
#define STM32_DMAMUX1_TIM3_UP       27
#define STM32_DMAMUX1_TIM3_TRIG     28
#define STM32_DMAMUX1_TIM4_CH1      29
#define STM32_DMAMUX1_TIM4_CH2      30
#define STM32_DMAMUX1_TIM4_CH3      31
#define STM32_DMAMUX1_TIM4_UP       32
#define STM32_DMAMUX1_I2C1_RX       33
#define STM32_DMAMUX1_I2C1_TX       34
#define STM32_DMAMUX1_I2C2_RX       35
#define STM32_DMAMUX1_I2C2_TX       36
#define STM32_DMAMUX1_SPI1_RX       37
#define STM32_DMAMUX1_SPI1_TX       38
#define STM32_DMAMUX1_SPI2_RX       39
#define STM32_DMAMUX1_SPI2_TX       40
#define STM32_DMAMUX1_USART1_RX     41
#define STM32_DMAMUX1_USART1_TX     42
#define STM32_DMAMUX1_USART2_RX     43
#define STM32_DMAMUX1_USART2_TX     44
#define STM32_DMAMUX1_USART3_RX     45
#define STM32_DMAMUX1_USART3_TX     46
#define STM32_DMAMUX1_TIM8_CH1      47
#define STM32_DMAMUX1_TIM8_CH2      48
#define STM32_DMAMUX1_TIM8_CH3      49
#define STM32_DMAMUX1_TIM8_CH4      50
#define STM32_DMAMUX1_TIM8_UP       51
#define STM32_DMAMUX1_TIM8_TRIG     52
#define STM32_DMAMUX1_TIM8_COM      53
#define STM32_DMAMUX1_RESERVED54    54
#define STM32_DMAMUX1_TIM5_CH1      55
#define STM32_DMAMUX1_TIM5_CH2      56
#define STM32_DMAMUX1_TIM5_CH3      57
#define STM32_DMAMUX1_TIM5_CH4      58
#define STM32_DMAMUX1_TIM5_UP       59
#define STM32_DMAMUX1_TIM5_TRIG     60
#define STM32_DMAMUX1_SPI3_RX       61
#define STM32_DMAMUX1_SPI3_TX       62
#define STM32_DMAMUX1_UART4_RX      63
#define STM32_DMAMUX1_UART4_TX      64
#define STM32_DMAMUX1_UART5_RX      65
#define STM32_DMAMUX1_UART5_TX      66
#define STM32_DMAMUX1_DAC1_CH1      67  /* Renamed to L4 name.*/
#define STM32_DMAMUX1_DAC1_CH2      68  /* Renamed to L4 name.*/
#define STM32_DMAMUX1_TIM6_UP       69
#define STM32_DMAMUX1_TIM7_UP       70
#define STM32_DMAMUX1_USART6_RX     71
#define STM32_DMAMUX1_USART6_TX     72
#define STM32_DMAMUX1_I2C3_RX       73
#define STM32_DMAMUX1_I2C3_TX       74
#define STM32_DMAMUX1_DCMI          75
#define STM32_DMAMUX1_CRYP_IN       76
#define STM32_DMAMUX1_CRYP_OUT      77
#define STM32_DMAMUX1_HASH_IN       78
#define STM32_DMAMUX1_UART7_RX      79
#define STM32_DMAMUX1_UART7_TX      80
#define STM32_DMAMUX1_UART8_RX      81
#define STM32_DMAMUX1_UART8_TX      82
#define STM32_DMAMUX1_SPI4_RX       83
#define STM32_DMAMUX1_SPI4_TX       84
#define STM32_DMAMUX1_SPI5_RX       85
#define STM32_DMAMUX1_SPI5_TX       86
#define STM32_DMAMUX1_SAI1_A        87
#define STM32_DMAMUX1_SAI1_B        88
#define STM32_DMAMUX1_SAI2_A        89
#define STM32_DMAMUX1_SAI2_B        90
#define STM32_DMAMUX1_SWPMI_RX      91
#define STM32_DMAMUX1_SQPMI_TX      92
#define STM32_DMAMUX1_SPDIFRX_DT    93
#define STM32_DMAMUX1_SPDIFRX_CS    94
#define STM32_DMAMUX1_HR_REQ1       95
#define STM32_DMAMUX1_HR_REQ2       96
#define STM32_DMAMUX1_HR_REQ3       97
#define STM32_DMAMUX1_HR_REQ4       98
#define STM32_DMAMUX1_HR_REQ5       99
#define STM32_DMAMUX1_HR_REQ6       100
#define STM32_DMAMUX1_DFSDM1_DMA0   101
#define STM32_DMAMUX1_DFSDM1_DMA1   102
#define STM32_DMAMUX1_DFSDM1_DMA2   103
#define STM32_DMAMUX1_DFSDM1_DMA3   104
#define STM32_DMAMUX1_TIM15_CH1     105
#define STM32_DMAMUX1_TIM15_UP      106
#define STM32_DMAMUX1_TIM15_TRIG    107
#define STM32_DMAMUX1_TIM15_COM     108
#define STM32_DMAMUX1_TIM16_CH1     109
#define STM32_DMAMUX1_TIM16_UP      110
#define STM32_DMAMUX1_TIM17_CH1     111
#define STM32_DMAMUX1_TIM17_UP      112
#define STM32_DMAMUX1_SAI3_A        113
#define STM32_DMAMUX1_SAI3_B        114
#define STM32_DMAMUX1_ADC3          115
#define STM32_DMAMUX1_I2C5_RX       124
#define STM32_DMAMUX1_I2C5_TX       125
/** @} */

/**
 * @name    DMAMUX2 request sources
 * @{
 */
#define STM32_DMAMUX2_REQ_GEN0      1
#define STM32_DMAMUX2_REQ_GEN1      2
#define STM32_DMAMUX2_REQ_GEN2      3
#define STM32_DMAMUX2_REQ_GEN3      4
#define STM32_DMAMUX2_REQ_GEN4      5
#define STM32_DMAMUX2_REQ_GEN5      6
#define STM32_DMAMUX2_REQ_GEN6      7
#define STM32_DMAMUX2_REQ_GEN7      8
#define STM32_DMAMUX2_LP_UART1_RX   9
#define STM32_DMAMUX2_LP_UART1_TX   10
#define STM32_DMAMUX2_SPI6_RX       11
#define STM32_DMAMUX2_SPI6_TX       12
#define STM32_DMAMUX2_I2C4_RX       13
#define STM32_DMAMUX2_I2C4_TX       14
#define STM32_DMAMUX2_SAI4_A        15
#define STM32_DMAMUX2_SAI4_B        16
#define STM32_DMAMUX2_ADC3_REQ      17
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
