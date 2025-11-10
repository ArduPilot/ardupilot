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
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int __wrap_snprintf(char *str, size_t size, const char *fmt, ...);
int __wrap_vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
int __wrap_vasprintf(char **strp, const char *fmt, va_list ap);
int __wrap_asprintf(char **strp, const char *fmt, ...);
int __wrap_vprintf(const char *fmt, va_list arg);
int __wrap_printf(const char *fmt, ...);
int __wrap_scanf(const char *fmt, ...);
int __wrap_sscanf(const char *buf, const char *fmt, ...);
int __wrap_fprintf(void *f, const char *fmt, ...);

int vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
int snprintf(char *str, size_t size, const char *fmt, ...); //undefined, only used as a placeholder, its replaced by wrap method at link time
int vasprintf(char **strp, const char *fmt, va_list ap);
int asprintf(char **strp, const char *fmt, ...);
int vprintf(const char *fmt, va_list arg);
int printf(const char *fmt, ...);

void *malloc(size_t size);
void *calloc(size_t nmemb, size_t size);
void free(void *ptr);
extern int (*vprintf_console_hook)(const char *fmt, va_list arg);
void malloc_check(const void *ptr);

#ifdef __cplusplus
}
#endif
