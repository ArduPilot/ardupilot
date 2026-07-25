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
 * @file    boot.h
 * @brief   Boot parameters for the SPC560Pxx.
 * @{
 */

#ifndef BOOT_H
#define BOOT_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    BUCSR registers definitions
 * @{
 */
#define BUCSR_BPEN              0x00000001
#define BUCSR_BALLOC_BFI        0x00000200
/** @} */

/**
 * @name   MSR register definitions
 * @{
 */
#define MSR_WE                  0x00040000
#define MSR_CE                  0x00020000
#define MSR_EE                  0x00008000
#define MSR_PR                  0x00004000
#define MSR_ME                  0x00001000
#define MSR_DE                  0x00000200
#define MSR_IS                  0x00000020
#define MSR_DS                  0x00000010
#define MSR_RI                  0x00000002
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*
 * BUCSR default settings.
 */
#if !defined(BOOT_BUCSR_DEFAULT) || defined(__DOXYGEN__)
#define BOOT_BUCSR_DEFAULT      (BUCSR_BPEN | BUCSR_BALLOC_BFI)
#endif

/*
 * MSR default settings.
 */
#if !defined(BOOT_MSR_DEFAULT) || defined(__DOXYGEN__)
#define BOOT_MSR_DEFAULT        (MSR_WE | MSR_CE | MSR_ME)
#endif

/*
 * Boot default settings.
 */
#if !defined(BOOT_PERFORM_CORE_INIT) || defined(__DOXYGEN__)
#define BOOT_PERFORM_CORE_INIT  1
#endif

/*
 * VLE mode default settings.
 */
#if !defined(BOOT_USE_VLE) || defined(__DOXYGEN__)
#define BOOT_USE_VLE            1
#endif

/*
 * RAM relocation flag.
 */
#if !defined(BOOT_RELOCATE_IN_RAM) || defined(__DOXYGEN__)
#define BOOT_RELOCATE_IN_RAM    0
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* BOOT_H */

/** @} */
