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
 * @file    MAX32625/aducm_isr.h
 * @brief   MAX32625 ISR handler header.
 *
 * @addtogroup MAX32625_ISR
 * @{
 */

#ifndef MAX32_ISR_H
#define MAX32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */
 
/*
 * TMR units.
 */
#define MAX32_TMR0_INT0_HANDLER     Vector98
#define MAX32_TMR0_INT1_HANDLER     Vector9C
#define MAX32_TMR1_INT0_HANDLER     VectorA0
#define MAX32_TMR1_INT1_HANDLER     VectorA4
#define MAX32_TMR2_INT0_HANDLER     VectorA8
#define MAX32_TMR2_INT1_HANDLER     VectorAC

#define MAX32_TMR0_INT0_NUMBER      22
#define MAX32_TMR0_INT1_NUMBER      23
#define MAX32_TMR1_INT0_NUMBER      24
#define MAX32_TMR1_INT1_NUMBER      25
#define MAX32_TMR2_INT0_NUMBER      26
#define MAX32_TMR2_INT1_NUMBER      27
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

#endif /* MAX32_ISR_H */

/** @} */
