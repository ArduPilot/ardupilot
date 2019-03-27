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

#include "posix.h"
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

int vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
int __wrap_snprintf(char *str, size_t size, const char *fmt, ...);
int snprintf(char *str, size_t size, const char *fmt, ...); //undefined, only used as a placeholder, its replaced by wrap method at link time
int vasprintf(char **strp, const char *fmt, va_list ap);
int asprintf(char **strp, const char *fmt, ...);
int vprintf(const char *fmt, va_list arg);
int printf(const char *fmt, ...);
#if defined(USE_FATFS) || (defined(HAL_OS_FATFS_IO) && HAL_OS_FATFS_IO)
int fscanf ( FILE * stream, const char * format, ... );
#endif

int scanf (const char *fmt, ...);
int __wrap_sscanf (const char *buf, const char *fmt, ...);
int sscanf (const char *buf, const char *fmt, ...); //undefined, only used as a placeholder, its replaced by wrap method at link time
int vsscanf (const char *buf, const char *s, va_list ap);
void *malloc(size_t size);
void *calloc(size_t nmemb, size_t size);
void free(void *ptr);

extern int (*vprintf_console_hook)(const char *fmt, va_list arg);

#define L_tmpnam 32

#ifdef __cplusplus
}
#endif
