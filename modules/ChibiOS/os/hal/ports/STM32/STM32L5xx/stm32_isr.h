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
 * @file    STM32L5xx/stm32_isr.h
 * @brief   STM32L5xx ISR handler header.
 *
 * @addtogroup STM32L5xx_ISR
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
#define STM32_TIM5_SUPPRESS_ISR
#define STM32_TIM6_SUPPRESS_ISR
#define STM32_TIM7_SUPPRESS_ISR
#define STM32_TIM8_SUPPRESS_ISR
#define STM32_TIM15_SUPPRESS_ISR
#define STM32_TIM16_SUPPRESS_ISR
#define STM32_TIM17_SUPPRESS_ISR

#define STM32_USART1_SUPPRESS_ISR
#define STM32_USART2_SUPPRESS_ISR
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
#define STM32_ADC12_HANDLER                 VectorD4

#define STM32_ADC12_NUMBER                  37

/*
 * FDCAN unit.
 */
#define STM32_FDCAN1_IT0_HANDLER            VectorDC
#define STM32_FDCAN1_IT1_HANDLER            VectorE0

#define STM32_FDCAN1_IT0_NUMBER             39
#define STM32_FDCAN1_IT1_NUMBER             40

/*
 * DMA unit.
 */
#define STM32_DMA1_CH1_HANDLER              VectorB4
#define STM32_DMA1_CH2_HANDLER              VectorB8
#define STM32_DMA1_CH3_HANDLER              VectorBC
#define STM32_DMA1_CH4_HANDLER              VectorC0
#define STM32_DMA1_CH5_HANDLER              VectorC4
#define STM32_DMA1_CH6_HANDLER              VectorC8
#define STM32_DMA1_CH7_HANDLER              VectorCC
#define STM32_DMA1_CH8_HANDLER              VectorD0
#define STM32_DMA2_CH1_HANDLER              Vector180
#define STM32_DMA2_CH2_HANDLER              Vector184
#define STM32_DMA2_CH3_HANDLER              Vector188
#define STM32_DMA2_CH4_HANDLER              Vector18C
#define STM32_DMA2_CH5_HANDLER              Vector190
#define STM32_DMA2_CH6_HANDLER              Vector194
#define STM32_DMA2_CH7_HANDLER              Vector198
#define STM32_DMA2_CH8_HANDLER              Vector19C

#define STM32_DMA1_CH1_NUMBER               29
#define STM32_DMA1_CH2_NUMBER               30
#define STM32_DMA1_CH3_NUMBER               31
#define STM32_DMA1_CH4_NUMBER               32
#define STM32_DMA1_CH5_NUMBER               33
#define STM32_DMA1_CH6_NUMBER               34
#define STM32_DMA1_CH7_NUMBER               35
#define STM32_DMA1_CH8_NUMBER               36
#define STM32_DMA2_CH1_NUMBER               80
#define STM32_DMA2_CH2_NUMBER               81
#define STM32_DMA2_CH3_NUMBER               82
#define STM32_DMA2_CH4_NUMBER               83
#define STM32_DMA2_CH5_NUMBER               84
#define STM32_DMA2_CH6_NUMBER               85
#define STM32_DMA2_CH7_NUMBER               86
#define STM32_DMA2_CH8_NUMBER               87

/*
 * EXTI unit.
 */
#define STM32_EXTI0_HANDLER                 Vector58
#define STM32_EXTI1_HANDLER                 Vector5C
#define STM32_EXTI2_HANDLER                 Vector60
#define STM32_EXTI3_HANDLER                 Vector64
#define STM32_EXTI4_HANDLER                 Vector68
#define STM32_EXTI5_HANDLER                 Vector9C
#define STM32_EXTI6_HANDLER                 Vector9C
#define STM32_EXTI7_HANDLER                 Vector9C
#define STM32_EXTI8_HANDLER                 Vector9C
#define STM32_EXTI9_HANDLER                 Vector9C
#define STM32_EXTI10_HANDLER                VectorE0
#define STM32_EXTI11_HANDLER                VectorE0
#define STM32_EXTI12_HANDLER                VectorE0
#define STM32_EXTI13_HANDLER                VectorE0
#define STM32_EXTI14_HANDLER                VectorE0
#define STM32_EXTI15_HANDLER                VectorE0
#define STM32_EXTI1635_38_HANDLER           Vector44    /* PVD PVM1..PVM4   */
#define STM32_EXTI17_HANDLER                Vector48    /* RTC              */
#define STM32_EXTI18_HANDLER                Vector4C    /* RTC (secure)     */
#define STM32_EXTI19_HANDLER                Vector50    /* TAMP             */
#define STM32_EXTI20_HANDLER                Vector54    /* TAMP (secure)    */
#define STM32_EXTI21_22_HANDLER             Vector160   /* COMP1..2         */

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
#define STM32_EXTI1635_38_NUMBER            1
#define STM32_EXTI17_NUMBER                 2
#define STM32_EXTI18_NUMBER                 3
#define STM32_EXTI19_NUMBER                 4
#define STM32_EXTI20_NUMBER                 5
#define STM32_EXTI21_22_NUMBER              72

/*
 * I2C units.
 */
#define STM32_I2C1_EVENT_HANDLER            Vector11C
#define STM32_I2C1_ERROR_HANDLER            Vector120
#define STM32_I2C2_EVENT_HANDLER            Vector124
#define STM32_I2C2_ERROR_HANDLER            Vector138
#define STM32_I2C3_EVENT_HANDLER            Vector188
#define STM32_I2C3_ERROR_HANDLER            Vector18C
#define STM32_I2C4_EVENT_HANDLER            Vector1B8
#define STM32_I2C4_ERROR_HANDLER            Vector1BC

#define STM32_I2C1_EVENT_NUMBER             55
#define STM32_I2C1_ERROR_NUMBER             56
#define STM32_I2C2_EVENT_NUMBER             57
#define STM32_I2C2_ERROR_NUMBER             58
#define STM32_I2C3_EVENT_NUMBER             88
#define STM32_I2C3_ERROR_NUMBER             89
#define STM32_I2C4_EVENT_NUMBER             100
#define STM32_I2C4_ERROR_NUMBER             101

/*
 * OCTOSPI unit.
 */
#define STM32_OCTOSPI1_HANDLER              Vector170

#define STM32_OCTOSPI1_NUMBER               76

/*
 * SDMMC unit.
 */
#define STM32_SDMMC1_HANDLER                Vector178

#define STM32_SDMMC1_NUMBER                 78

/*
 * TIM units.
 */
#define STM32_TIM1_BRK_HANDLER              VectorE4
#define STM32_TIM1_UP_HANDLER               VectorE8
#define STM32_TIM1_TRGCO_HANDLER            VectorEC
#define STM32_TIM1_CC_HANDLER               VectorF0
#define STM32_TIM2_HANDLER                  VectorF4
#define STM32_TIM3_HANDLER                  VectorF8
#define STM32_TIM4_HANDLER                  VectorFC
#define STM32_TIM5_HANDLER                  Vector100
#define STM32_TIM6_HANDLER                  Vector104
#define STM32_TIM7_HANDLER                  Vector108
#define STM32_TIM8_BRK_HANDLER              Vector10C
#define STM32_TIM8_UP_HANDLER               Vector110
#define STM32_TIM8_TRGCO_HANDLER            Vector114
#define STM32_TIM8_CC_HANDLER               Vector118
#define STM32_TIM15_HANDLER                 Vector154
#define STM32_TIM16_HANDLER                 Vector158
#define STM32_TIM17_HANDLER                 Vector15C

#define STM32_TIM1_BRK_NUMBER               41
#define STM32_TIM1_UP_NUMBER                42
#define STM32_TIM1_TRGCO_NUMBER             43
#define STM32_TIM1_CC_NUMBER                44
#define STM32_TIM2_NUMBER                   45
#define STM32_TIM3_NUMBER                   46
#define STM32_TIM4_NUMBER                   47
#define STM32_TIM5_NUMBER                   48
#define STM32_TIM6_NUMBER                   49
#define STM32_TIM7_NUMBER                   50
#define STM32_TIM8_BRK_NUMBER               51
#define STM32_TIM8_UP_NUMBER                52
#define STM32_TIM8_TRGCO_NUMBER             53
#define STM32_TIM8_CC_NUMBER                54
#define STM32_TIM15_NUMBER                  69
#define STM32_TIM16_NUMBER                  70
#define STM32_TIM17_NUMBER                  71

/*
 * USART/UART units.
 */
#define STM32_USART1_HANDLER                Vector134
#define STM32_USART2_HANDLER                Vector138
#define STM32_USART3_HANDLER                Vector13C
#define STM32_UART4_HANDLER                 Vector140
#define STM32_UART5_HANDLER                 Vector144
#define STM32_LPUART1_HANDLER               Vector148

#define STM32_USART1_NUMBER                 61
#define STM32_USART2_NUMBER                 62
#define STM32_USART3_NUMBER                 63
#define STM32_UART4_NUMBER                  64
#define STM32_UART5_NUMBER                  65
#define STM32_LPUART1_NUMBER                66

/*
 * USB/OTG units.
 */
#define STM32_USB_FS_HANDLER                Vector164

#define STM32_USB_FS_NUMBER                 73

/*
 * FSMC unit.
 */
#define STM32_FSMC_HANDLER                  Vector16C

#define STM32_FSMC_NUMBER                   75
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
