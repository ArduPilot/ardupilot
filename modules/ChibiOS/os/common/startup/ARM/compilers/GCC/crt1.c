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
 * @file    ARMCMx/compilers/GCC/crt1.c
 * @brief   Startup stub functions.
 *
 * @addtogroup ARMCMx_GCC_STARTUP
 * @{
 */

#include <stdbool.h>

/**
 * @brief   Architecture-dependent core initialization.
 * @details This hook is invoked immediately after the stack initialization
 *          and before the DATA and BSS segments initialization.
 * @note    This function is a weak symbol.
 */
#if !defined(__DOXYGEN__)
__attribute__((weak))
#endif
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void __cpu_init(void) {}

/**
 * @brief   Early initialization.
 * @details This hook is invoked immediately after the stack initialization
 *          and before the DATA and BSS segments initialization. The
 *          default behavior is to do nothing.
 * @note    This function is a weak symbol.
 */
#if !defined(__DOXYGEN__)
__attribute__((weak))
#endif
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void __early_init(void) {}
/*lint -restore*/

/**
 * @brief   Late initialization.
 * @details This hook is invoked after the DATA and BSS segments
 *          initialization and before any static constructor. The
 *          default behavior is to do nothing.
 * @note    This function is a weak symbol.
 */
#if !defined(__DOXYGEN__)
__attribute__((weak))
#endif
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void __late_init(void) {}
/*lint -restore*/

/**
 * @brief   Default @p main() function exit handler.
 * @details This handler is invoked or the @p main() function exit. The
 *          default behavior is to enter an infinite loop.
 * @note    This function is a weak symbol.
 */
#if !defined(__DOXYGEN__)
__attribute__((noreturn, weak))
#endif
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void __default_exit(void) {
/*lint -restore*/

  while (true) {
  }
}

/** @} */
