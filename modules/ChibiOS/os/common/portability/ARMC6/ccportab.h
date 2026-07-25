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
 * @file    ARMC6/ccportab.h
 * @brief   Compiler portability layer.
 *
 * @defgroup CC_PORTAB Compiler portability.
 * @{
 */

#ifndef CCPORTAB_H
#define CCPORTAB_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

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
 * @name    Compiler abstraction macros
 * @{
 */
/**
 * @brief   Allocates a variable or function to a specific section.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_SECTION(s)       __attribute__((section(s)))

/**
 * @brief   Marks a function or variable as a weak symbol.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_WEAK             __attribute__((weak))

/**
 * @brief   Marks a function or variable as used.
 * @details The compiler or linker shall not remove the marked function or
 *          variable regardless if it is referred or not in the code.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_USED             __attribute__((used))

/**
 * @brief   Enforces alignment of the variable declared afterward.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_ALIGN_DATA(n)    __attribute__((aligned(n)))

/**
 * @brief   Enforces alignment of a function declared afterward.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_ALIGN_CODE(n)    __attribute__((aligned(n)))

/**
 * @brief   Enforces packing of the structure declared afterward.
 * @note    If the compiler does not support such a feature then this macro
 *          must not be defined or it could originate errors.
 */
#define CC_PACK             __attribute__((packed))

/**
 * @brief   Marks a function as not inlineable.
 * @note    Can be implemented as an empty macro if not supported by the
 *          compiler.
 */
#define CC_NO_INLINE        __attribute__((noinline))

/**
 * @brief   Enforces a function inline.
 * @note    Can be implemented as an empty macro if not supported by the
 *          compiler.
 */
#define CC_FORCE_INLINE     __attribute__((always_inline))

/**
 * @brief   Marks a function as non-returning.
 * @note    Can be implemented as an empty macro if not supported by the
 *          compiler.
 */
#define CC_NO_RETURN        __attribute__((noreturn))

/**
 * @brief   Enforces a variable in a ROM area.
 * @note    Can be implemented as an empty macro if not supported by the
 *          compiler.
 */
#define CC_ROMCONST         const

/**
 * @brief   Marks a boolean expression as likely true.
 *
 * @param[in] x         a valid expression
 */
#define CC_LIKELY(x)        __builtin_expect(!!(x), 1)

/**
 * @brief   Marks a boolean expression as likely false.
 *
 * @param[in] x         a valid expression
 */
#define CC_UNLIKELY(x)      __builtin_expect(!!(x), 0)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CCPORTAB_H */

/** @} */
