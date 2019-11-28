/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * **** This file incorporates work covered by the following copyright and ****
 * **** permission notice:                                                 ****
 *
 * Copyright (C) 2006-2017 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

#ifndef WOLFSSL_HELPERS_H
#define WOLFSSL_HELPERS_H
#include <stdint.h>
#include <stddef.h>

/* HW RNG support */
///@brief make sure we use our strerror_r function
#define HAVE_PK_CALLBACKS
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define XMALLOC_OVERRIDE
#define NO_CRYPT_BENCHMARK
#define NO_CRYPT_TEST

#define CUSTOM_RAND_GENERATE chibios_rand_generate
#define CUSTOM_RAND_TYPE uint32_t

#ifdef __cplusplus
extern "C" {
#endif
unsigned int chibios_rand_generate(void);
int custom_rand_generate_block(unsigned char* output, unsigned int sz);
void *malloc(size_t size);
void *calloc(size_t nmemb, size_t size);
void *realloc (void *addr, size_t size);
void free(void *addr);
void show_gen_printf(const char* chr);
#ifdef __cplusplus
}
#endif

/* Realloc (to use without USE_FAST_MATH) */

#define XREALLOC(p,n,h,t) realloc( (p) , (n) )
#define XMALLOC(s,h,t) malloc(s)
#define XFREE(p,h,t)   free(p)
#define SHOW_GEN        1
#endif