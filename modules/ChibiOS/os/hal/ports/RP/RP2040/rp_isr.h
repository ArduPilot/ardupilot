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
 * @file    RP2040/rp_isr.h
 * @brief   RP2040 ISR handler header.
 *
 * @addtogroup RP2040_ISR
 * @{
 */

#ifndef RP_ISR_H
#define RP_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers
 * @{
 */
#define RP_TIMER_IRQ0_HANDLER               Vector40
#define RP_TIMER_IRQ1_HANDLER               Vector44
#define RP_TIMER_IRQ2_HANDLER               Vector48
#define RP_TIMER_IRQ3_HANDLER               Vector4C
#define RP_PWM_IRQ_WRAP_HANDLER             Vector50
#define RP_USBCTRL_IRQ_HANDLER              Vector54
#define RP_XIP_IRQ_HANDLER                  Vector58
#define RP_PIO0_IRQ_0_HANDLER               Vector5C
#define RP_PIO0_IRQ_1_HANDLER               Vector60
#define RP_PIO1_IRQ_0_HANDLER               Vector64
#define RP_PIO1_IRQ_1_HANDLER               Vector68
#define RP_DMA_IRQ_0_HANDLER                Vector6C
#define RP_DMA_IRQ_1_HANDLER                Vector70
#define RP_IO_IRQ_BANK0_HANDLER             Vector74
#define RP_IO_IRQ_QSPI_HANDLER              Vector78
#define RP_SIO_IRQ_PROC0_HANDLER            Vector7C
#define RP_SIO_IRQ_PROC1_HANDLER            Vector80
#define RP_CLOCKS_IRQ_HANDLER               Vector84
#define RP_SPI0_IRQ_HANDLER                 Vector88
#define RP_SPI1_IRQ_HANDLER                 Vector8C
#define RP_UART0_IRQ_HANDLER                Vector90
#define RP_UART1_IRQ_HANDLER                Vector94
#define RP_ADC_IRQ_FIFO_HANDLER             Vector98
#define RP_I2C0_IRQ_HANDLER                 Vector9C
#define RP_I2C1_IRQ_HANDLER                 VectorA0
#define RP_RTC_IRQ_HANDLER                  VectorA4

#define RP_TIMER_IRQ0_NUMBER                0
#define RP_TIMER_IRQ1_NUMBER                1
#define RP_TIMER_IRQ2_NUMBER                2
#define RP_TIMER_IRQ3_NUMBER                3
#define RP_PWM_IRQ_WRAP_NUMBER              4
#define RP_USBCTRL_IRQ_NUMBER               5
#define RP_XIP_IRQ_NUMBER                   6
#define RP_PIO0_IRQ_0_NUMBER                7
#define RP_PIO0_IRQ_1_NUMBER                8
#define RP_PIO1_IRQ_0_NUMBER                9
#define RP_PIO1_IRQ_1_NUMBER                10
#define RP_DMA_IRQ_0_NUMBER                 11
#define RP_DMA_IRQ_1_NUMBER                 12
#define RP_IO_IRQ_BANK0_NUMBER              13
#define RP_IO_IRQ_QSPI_NUMBER               14
#define RP_SIO_IRQ_PROC0_NUMBER             15
#define RP_SIO_IRQ_PROC1_NUMBER             16
#define RP_CLOCKS_IRQ_NUMBER                17
#define RP_SPI0_IRQ_NUMBER                  18
#define RP_SPI1_IRQ_NUMBER                  19
#define RP_UART0_IRQ_NUMBER                 20
#define RP_UART1_IRQ_NUMBER                 21
#define RP_ADC_IRQ_FIFO_NUMBER              22
#define RP_I2C0_IRQ_NUMBER                  23
#define RP_I2C1_IRQ_NUMBER                  24
#define RP_RTC_IRQ_NUMBER                   25
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

#endif /* RP_ISR_H */

/** @} */
