/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/aducm_isr.h
 * @brief   ADUCM36x ISR handler header.
 *
 * @addtogroup ADUCM36x_ISR
 * @{
 */

#ifndef ADUCM_ISR_H
#define ADUCM_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */

/*
 * SPI units.
 */
#define ADUCM_SPI0_HANDLER          Vector88
#define ADUCM_SPI1_HANDLER          Vector8C

#define ADUCM_SPI0_NUMBER           18
#define ADUCM_SPI1_NUMBER           19
 
/*
 * TIM units.
 */
#define ADUCM_TIMER0_HANDLER        Vector6C
#define ADUCM_TIMER1_HANDLER        Vector70
#define ADUCM_TIMER2_HANDLER        Vector40
#define ADUCM_TIMER3_HANDLER        Vector64

#define ADUCM_TIMER0_NUMBER         11
#define ADUCM_TIMER1_NUMBER         12
#define ADUCM_TIMER2_NUMBER         0
#define ADUCM_TIMER3_NUMBER         9

/*
 * UART units.
 */
#define ADUCM_UART0_HANDLER         Vector84

#define ADUCM_UART0_NUMBER          17
/** @} */

/*
 * Wake Up timer.
 */
#define ADUCM_WUT_HANDLER           Vector40

#define ADUCM_WUT_NUMBER            0
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

#endif /* ADUCM_ISR_H */

/** @} */
