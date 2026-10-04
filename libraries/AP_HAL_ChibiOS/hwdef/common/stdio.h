/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
  asm labels make callers reference our __wrap_ implementations
  directly, so LTO keeps them. --wrap still catches any other callers.
  Avoid libc includes as this is force-included before feature macros
 */
int snprintf(char *str, size_t size, const char *fmt, ...) __asm__("__wrap_snprintf");
int vsnprintf(char *str, size_t size, const char *fmt, va_list ap) __asm__("__wrap_vsnprintf");
int vasprintf(char **strp, const char *fmt, va_list ap) __asm__("__wrap_vasprintf");
int asprintf(char **strp, const char *fmt, ...) __asm__("__wrap_asprintf");
int vprintf(const char *fmt, va_list arg) __asm__("__wrap_vprintf");
int printf(const char *fmt, ...) __asm__("__wrap_printf");
int scanf(const char *fmt, ...) __asm__("__wrap_scanf");
int sscanf(const char *buf, const char *fmt, ...) __asm__("__wrap_sscanf");
struct __sFILE;
int fprintf(struct __sFILE *f, const char *fmt, ...) __asm__("__wrap_fprintf");

void *malloc(size_t size);
void *calloc(size_t nmemb, size_t size);
void free(void *ptr);
extern int (*vprintf_console_hook)(const char *fmt, va_list arg);
void malloc_check(const void *ptr);

#ifdef __cplusplus
}
#endif
