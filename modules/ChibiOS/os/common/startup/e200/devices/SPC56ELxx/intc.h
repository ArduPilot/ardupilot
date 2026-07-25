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
 * @file    SPC56ELxx/intc.h
 * @brief   SPC56ELxx INTC module header.
 *
 * @addtogroup INTC
 * @{
 */

#ifndef INTC_H
#define INTC_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    INTC addresses
 * @{
 */
#define INTC_BASE           0xFFF48000
#define INTC_IACKR_ADDR     (INTC_BASE + 0x10)
#define INTC_EOIR_ADDR      (INTC_BASE + 0x18)
/** @} */

/**
 * @brief   INTC priority levels.
 */
#define INTC_PRIORITY_LEVELS 16U

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @name    INTC-related macros
 * @{
 */
#define INTC_BCR            (*((volatile uint32_t *)(INTC_BASE + 0)))
#define INTC_CPR(n)         (*((volatile uint32_t *)(INTC_BASE + 8 + ((n) * sizeof (uint32_t)))))
#define INTC_IACKR(n)       (*((volatile uint32_t *)(INTC_BASE + 0x10 + ((n) * sizeof (uint32_t)))))
#define INTC_EOIR(n)        (*((volatile uint32_t *)(INTC_BASE + 0x18 + ((n) * sizeof (uint32_t)))))
#define INTC_PSR(n)         (*((volatile uint8_t *)(INTC_BASE + 0x40 + ((n) * sizeof (uint8_t)))))
/** @} */

/**
 * @brief   Core selection macros for PSR register.
 */
#define INTC_PSR_CORE0      0x00

/**
 * @brief   PSR register content helper
 */
#define INTC_PSR_ENABLE(cores, prio) ((uint32_t)(cores) | (uint32_t)(prio))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* INTC_H */

/** @} */
