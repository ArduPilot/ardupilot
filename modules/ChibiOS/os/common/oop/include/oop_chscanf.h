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

/*
    This file was contributed by Alex Lewontin.
 */

/**
 * @file    oop_chscanf.h
 * @brief   Mini scanf-like functionality.
 *
 * @addtogroup OOP_CHSCANF
 * @{
 */

#ifndef OOP_CHSCANF_H
#define OOP_CHSCANF_H

#include <stdarg.h>

/**
 * @brief   Float type support.
 */
#if !defined(CHSCANF_USE_FLOAT) || defined(__DOXYGEN__)
#define CHSCANF_USE_FLOAT FALSE
#endif

#if CHSCANF_USE_FLOAT
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
  int chvscanf(sequential_stream_i *stmp, const char *fmt, va_list ap);
  int chscanf(sequential_stream_i *stmp, const char *fmt, ...);
  int chsnscanf(char *str, size_t size, const char *fmt, ...);
  int chvsnscanf(char *str, size_t size, const char *fmt, va_list ap);
#ifdef __cplusplus
}
#endif

#endif /* OOP_CHSCANF_H */

/** @} */
