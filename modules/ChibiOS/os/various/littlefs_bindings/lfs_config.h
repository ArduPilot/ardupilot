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
 * Parts of this file are:
 *
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    lfs_wspi.h
 * @brief   LittleFS bindings header.
 *
 * @addtogroup LITTLEFS_BINDINGS
 * @{
 */

#ifndef LFS_CONFIG_H
#define LFS_CONFIG_H

#include <stdint.h>
#include <string.h>

#include "hal.h"

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

/* Logging functions.*/
#define LFS_TRACE(...)
#define LFS_DEBUG(...)
#define LFS_WARN(...)
#define LFS_ERROR(...)

/* Runtime assertions.*/
#ifndef LFS_ASSERT
#define LFS_ASSERT(test) osalDbgAssert(test, "LittleFS")
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint32_t lfs_crc(uint32_t crc, const void *buffer, size_t size);
  void *lfs_malloc(size_t size);
  void lfs_free(void *p);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline uint32_t lfs_max(uint32_t a, uint32_t b) {
  return (a > b) ? a : b;
}

static inline uint32_t lfs_min(uint32_t a, uint32_t b) {
  return (a < b) ? a : b;
}

static inline uint32_t lfs_aligndown(uint32_t a, uint32_t alignment) {
  return a - (a % alignment);
}

static inline uint32_t lfs_alignup(uint32_t a, uint32_t alignment) {
  return lfs_aligndown(a + alignment - 1, alignment);
}

static inline uint32_t lfs_npw2(uint32_t a) {
#if !defined(LFS_NO_INTRINSICS) && (defined(__GNUC__) || defined(__CC_ARM))
  return 32 - __builtin_clz(a - 1);
#else
  uint32_t r = 0;
  uint32_t s;
  a -= 1;
  s = (a > 0xffff) << 4; a >>= s; r |= s;
  s = (a > 0xff  ) << 3; a >>= s; r |= s;
  s = (a > 0xf   ) << 2; a >>= s; r |= s;
  s = (a > 0x3   ) << 1; a >>= s; r |= s;
  return (r | (a >> 1)) + 1;
#endif
}

static inline uint32_t lfs_ctz(uint32_t a) {
#if !defined(LFS_NO_INTRINSICS) && defined(__GNUC__)
  return __builtin_ctz(a);
#else
  return lfs_npw2((a & -a) + 1) - 1;
#endif
}

static inline uint32_t lfs_popc(uint32_t a) {
#if !defined(LFS_NO_INTRINSICS) && (defined(__GNUC__) || defined(__CC_ARM))
  return __builtin_popcount(a);
#else
  a = a - ((a >> 1) & 0x55555555);
  a = (a & 0x33333333) + ((a >> 2) & 0x33333333);
  return (((a + (a >> 4)) & 0xf0f0f0f) * 0x1010101) >> 24;
#endif
}

static inline int lfs_scmp(uint32_t a, uint32_t b) {
  return (int)(unsigned)(a - b);
}

static inline uint32_t lfs_fromle32(uint32_t a) {
#if !defined(LFS_NO_INTRINSICS) && ( \
    (defined(  BYTE_ORDER  ) && defined(  ORDER_LITTLE_ENDIAN  ) &&   BYTE_ORDER   ==   ORDER_LITTLE_ENDIAN  ) || \
    (defined(__BYTE_ORDER  ) && defined(__ORDER_LITTLE_ENDIAN  ) && __BYTE_ORDER   == __ORDER_LITTLE_ENDIAN  ) || \
    (defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__))
  return a;
#elif !defined(LFS_NO_INTRINSICS) && ( \
    (defined(  BYTE_ORDER  ) && defined(  ORDER_BIG_ENDIAN  ) &&   BYTE_ORDER   ==   ORDER_BIG_ENDIAN  ) || \
    (defined(__BYTE_ORDER  ) && defined(__ORDER_BIG_ENDIAN  ) && __BYTE_ORDER   == __ORDER_BIG_ENDIAN  ) || \
    (defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__))
  return __builtin_bswap32(a);
#else
  return (((uint8_t*)&a)[0] <<  0) |
         (((uint8_t*)&a)[1] <<  8) |
         (((uint8_t*)&a)[2] << 16) |
         (((uint8_t*)&a)[3] << 24);
#endif
}

static inline uint32_t lfs_tole32(uint32_t a) {
  return lfs_fromle32(a);
}

static inline uint32_t lfs_frombe32(uint32_t a) {
#if !defined(LFS_NO_INTRINSICS) && ( \
    (defined(  BYTE_ORDER  ) && defined(  ORDER_LITTLE_ENDIAN  ) &&   BYTE_ORDER   ==   ORDER_LITTLE_ENDIAN  ) || \
    (defined(__BYTE_ORDER  ) && defined(__ORDER_LITTLE_ENDIAN  ) && __BYTE_ORDER   == __ORDER_LITTLE_ENDIAN  ) || \
    (defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__))
  return __builtin_bswap32(a);
#elif !defined(LFS_NO_INTRINSICS) && ( \
    (defined(  BYTE_ORDER  ) && defined(  ORDER_BIG_ENDIAN  ) &&   BYTE_ORDER   ==   ORDER_BIG_ENDIAN  ) || \
    (defined(__BYTE_ORDER  ) && defined(__ORDER_BIG_ENDIAN  ) && __BYTE_ORDER   == __ORDER_BIG_ENDIAN  ) || \
    (defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__))
  return a;
#else
  return (((uint8_t*)&a)[0] << 24) |
         (((uint8_t*)&a)[1] << 16) |
         (((uint8_t*)&a)[2] <<  8) |
         (((uint8_t*)&a)[3] <<  0);
#endif
}

static inline uint32_t lfs_tobe32(uint32_t a) {
  return lfs_frombe32(a);
}

#endif /* LFS_CONFIG_H */

/** @} */
