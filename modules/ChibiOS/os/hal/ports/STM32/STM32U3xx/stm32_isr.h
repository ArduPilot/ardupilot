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
 * @file    STM32U3xx/stm32_isr.h
 * @brief   STM32U3xx ISR handler header.
 *
 * @addtogroup STM32U3xx_ISR
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
#define STM32_TIM2_SUPPRESS_ISR
#define STM32_TIM3_SUPPRESS_ISR
#define STM32_TIM4_SUPPRESS_ISR
#define STM32_TIM6_SUPPRESS_ISR
#define STM32_TIM7_SUPPRESS_ISR
#define STM32_TIM15_SUPPRESS_ISR
#define STM32_TIM16_SUPPRESS_ISR
#define STM32_TIM17_SUPPRESS_ISR

#define STM32_USART1_SUPPRESS_ISR
#define STM32_USART3_SUPPRESS_ISR
#define STM32_UART4_SUPPRESS_ISR
#define STM32_UART5_SUPPRESS_ISR
#define STM32_LPUART1_SUPPRESS_ISR
/** @} */

/**
 * @name    ISR names and numbers
 * @{
 */
/*
 * ADC unit.
 */
#define STM32_ADC1_HANDLER                  VectorD4
#define STM32_ADC2_HANDLER                  Vector204

#define STM32_ADC1_NUMBER                   37
#define STM32_ADC2_NUMBER                   113

/*
 * DAC unit.
 */
#define STM32_DAC1_HANDLER                  VectorD8

#define STM32_DAC1_NUMBER                   38

/*
 * DMA unit.
 */
#define STM32_DMA31_CH0_HANDLER             VectorB4
#define STM32_DMA31_CH1_HANDLER             VectorB8
#define STM32_DMA31_CH2_HANDLER             VectorBC
#define STM32_DMA31_CH3_HANDLER             VectorC0
#define STM32_DMA31_CH4_HANDLER             VectorC4
#define STM32_DMA31_CH5_HANDLER             VectorC8
#define STM32_DMA31_CH6_HANDLER             VectorCC
#define STM32_DMA31_CH7_HANDLER             VectorD0
#define STM32_DMA31_CH8_HANDLER             Vector180
#define STM32_DMA31_CH9_HANDLER             Vector184
#define STM32_DMA31_CH10_HANDLER            Vector188
#define STM32_DMA31_CH11_HANDLER            Vector18C

#define STM32_DMA31_CH0_NUMBER              29
#define STM32_DMA31_CH1_NUMBER              30
#define STM32_DMA31_CH2_NUMBER              31
#define STM32_DMA31_CH3_NUMBER              32
#define STM32_DMA31_CH4_NUMBER              33
#define STM32_DMA31_CH5_NUMBER              34
#define STM32_DMA31_CH6_NUMBER              35
#define STM32_DMA31_CH7_NUMBER              36
#define STM32_DMA31_CH8_NUMBER              80
#define STM32_DMA31_CH9_NUMBER              81
#define STM32_DMA31_CH10_NUMBER             82
#define STM32_DMA31_CH11_NUMBER             83

/*
 * EXTI unit.
 */
#define STM32_EXTI0_HANDLER                 Vector6C
#define STM32_EXTI1_HANDLER                 Vector70
#define STM32_EXTI2_HANDLER                 Vector74
#define STM32_EXTI3_HANDLER                 Vector78
#define STM32_EXTI4_HANDLER                 Vector7C
#define STM32_EXTI5_HANDLER                 Vector80
#define STM32_EXTI6_HANDLER                 Vector84
#define STM32_EXTI7_HANDLER                 Vector88
#define STM32_EXTI8_HANDLER                 Vector8C
#define STM32_EXTI9_HANDLER                 Vector90
#define STM32_EXTI10_HANDLER                Vector94
#define STM32_EXTI11_HANDLER                Vector98
#define STM32_EXTI12_HANDLER                Vector9C
#define STM32_EXTI13_HANDLER                VectorA0
#define STM32_EXTI14_HANDLER                VectorA4
#define STM32_EXTI15_HANDLER                VectorA8

#define STM32_EXTI0_NUMBER                  11
#define STM32_EXTI1_NUMBER                  12
#define STM32_EXTI2_NUMBER                  13
#define STM32_EXTI3_NUMBER                  14
#define STM32_EXTI4_NUMBER                  15
#define STM32_EXTI5_NUMBER                  16
#define STM32_EXTI6_NUMBER                  17
#define STM32_EXTI7_NUMBER                  18
#define STM32_EXTI8_NUMBER                  19
#define STM32_EXTI9_NUMBER                  20
#define STM32_EXTI10_NUMBER                 21
#define STM32_EXTI11_NUMBER                 22
#define STM32_EXTI12_NUMBER                 23
#define STM32_EXTI13_NUMBER                 24
#define STM32_EXTI14_NUMBER                 25
#define STM32_EXTI15_NUMBER                 26

/*
 * FDCAN units.
 */
#define STM32_FDCAN1_IT0_HANDLER            VectorDC
#define STM32_FDCAN1_IT1_HANDLER            VectorE0

#define STM32_FDCAN1_IT0_NUMBER             39
#define STM32_FDCAN1_IT1_NUMBER             40

/*
 * I2C units.
 */
#define STM32_I2C1_EV_HANDLER               Vector11C
#define STM32_I2C1_ER_HANDLER               Vector120
#define STM32_I2C2_EV_HANDLER               Vector124
#define STM32_I2C2_ER_HANDLER               Vector128
#define STM32_I2C3_EV_HANDLER               Vector1A0
#define STM32_I2C3_ER_HANDLER               Vector1A4

#define STM32_I2C1_EV_NUMBER                55
#define STM32_I2C1_ER_NUMBER                56
#define STM32_I2C2_EV_NUMBER                57
#define STM32_I2C2_ER_NUMBER                58
#define STM32_I2C3_EV_NUMBER                88
#define STM32_I2C3_ER_NUMBER                89

/*
 * OCTOSPI units.
 */
#define STM32_OCTOSPI1_HANDLER              Vector170

#define STM32_OCTOSPI1_NUMBER               76

/*
 * SDMMC units.
 */
#define STM32_SDMMC1_HANDLER                Vector178

#define STM32_SDMMC1_NUMBER                 78

/*
 * SPI units.
 */
#define STM32_SPI1_HANDLER                  Vector12C
#define STM32_SPI2_HANDLER                  Vector130
#define STM32_SPI3_HANDLER                  Vector1CC

#define STM32_SPI1_NUMBER                   59
#define STM32_SPI2_NUMBER                   60
#define STM32_SPI3_NUMBER                   99

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_TERR_IERR_HANDLER    VectorE4
#define STM32_TIM1_UP_HANDLER               VectorE8
#define STM32_TIM1_TRGCO_DIR_IDX_HANDLER    VectorEC
#define STM32_TIM1_CC_HANDLER               VectorF0
#define STM32_TIM2_HANDLER                  VectorF4
#define STM32_TIM3_HANDLER                  VectorF8
#define STM32_TIM4_HANDLER                  VectorFC
#define STM32_TIM6_HANDLER                  Vector104
#define STM32_TIM7_HANDLER                  Vector108

#define STM32_TIM1_BRK_TERR_IERR_NUMBER     41
#define STM32_TIM1_UP_NUMBER                42
#define STM32_TIM1_TRGCO_DIR_IDX_NUMBER     43
#define STM32_TIM1_CC_NUMBER                44
#define STM32_TIM2_NUMBER                   45
#define STM32_TIM3_NUMBER                   46
#define STM32_TIM4_NUMBER                   47
#define STM32_TIM6_NUMBER                   49
#define STM32_TIM7_NUMBER                   50

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                Vector134
#define STM32_USART3_HANDLER                Vector13C
#define STM32_UART4_HANDLER                 Vector140
#define STM32_UART5_HANDLER                 Vector144
#define STM32_LPUART1_HANDLER               Vector148

#define STM32_USART1_NUMBER                 61
#define STM32_USART3_NUMBER                 63
#define STM32_UART4_NUMBER                  64
#define STM32_UART5_NUMBER                  65
#define STM32_LPUART1_NUMBER                66

/*
 * USB units.
 */
#define STM32_USB1_HANDLER                  Vector164
#define STM32_USB1_NUMBER                   73
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
  void irqInit(void);
  void irqDeinit(void);
#ifdef __cplusplus
}
#endif

#endif /* STM32_ISR_H */

/** @} */
