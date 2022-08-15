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

#include "hwdef.h"
#include "stdio.h"

/* HW RNG support */
///@brief make sure we use our strerror_r function
#define HAVE_PK_CALLBACKS
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define XMALLOC_OVERRIDE
#define NO_CRYPT_BENCHMARK
#define NO_CRYPT_TEST
#define WOLFCRYPT_ONLY
#define NO_HMAC
#define NO_MD5
#define NO_OLD_TLS

#ifdef HAL_BOOTLOADER_BUILD
#define NO_ERROR_STRINGS
#define NO_RSA
#define WOLFSSL_SMALL_STACK
#else
#define SHOW_GEN
#endif

#define NO_AES_CBC
#define NO_WOLFSSL_SERVER

#define CUSTOM_RAND_GENERATE wolfssl_rand_get
#define CUSTOM_RAND_TYPE uint32_t

#define NO_STM32_CRYPTO
#define NO_STM32_RNG


#define HASH_AlgoSelection_SHA1     0UL
#define HASH_AlgoSelection_MD5      HASH_CR_ALGO_0
#define HASH_AlgoSelection_SHA224   HASH_CR_ALGO_1
#define HASH_AlgoSelection_SHA256   HASH_CR_ALGO


/* Realloc (to use without USE_FAST_MATH) */

#define XREALLOC(p,n,h,t) ((void*)std_realloc( (p) , (n) )); (void)h; (void)t
#define XMALLOC(s,h,t) ((void*)malloc(s)); (void)h; (void)t
#define XFREE(p,h,t)   free(p)
#endif