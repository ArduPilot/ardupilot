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
 * @file    common/ARMCMx/cache.h
 * @brief   Cortex-Mx cache support macros and structures.
 *
 * @addtogroup COMMON_ARMCMx_CACHE
 * @{
 */

#ifndef CACHE_H
#define CACHE_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#if defined(__DCACHE_PRESENT) || defined(__DOXYGEN__)
/**
 * @brief   Data cache line size, zero if there is no data cache.
 */
#define CACHE_LINE_SIZE                     32U
#else
#define CACHE_LINE_SIZE                     0U
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#if defined(__DCACHE_PRESENT) || defined(__DOXYGEN__)
#if (__DCACHE_PRESENT != 0) || defined(__DOXYGEN__)
/**
 * @brief   Aligns the specified size to a multiple of cache line size.
 * @note    This macros assumes that the size of the type @p t is a power of
 *          two and not greater than @p CACHE_LINE_SIZE.
 *
 * @param[in] t         type of the buffer element
 * @param[in] n         number of buffer elements
 */
#define CACHE_SIZE_ALIGN(t, n)                                              \
  ((((((n) * sizeof (t)) - 1U) | (CACHE_LINE_SIZE - 1U)) + 1U) / sizeof (t))

/**
 * @brief   Invalidates the data cache lines overlapping a memory buffer.
 * @details This function is meant to make sure that data written in
 *          data cache is invalidated.
 * @note    On devices without data cache this function does nothing.
 * @note    The function does not consider the lower 5 bits of addresses,
 *          the buffers are meant to be aligned to a 32 bytes boundary or
 *          adjacent data can be invalidated as side effect.
 *
 * @param[in] saddr     start address of the DMA buffer
 * @param[in] n         size of the DMA buffer in bytes
 *
 * @api
 */
#define cacheBufferInvalidate(saddr, n) {                                   \
  uint8_t *start = (uint8_t *)(saddr);                                      \
  uint8_t *end = start + (size_t)(n);                                       \
  __DSB();                                                                  \
  while (start < end) {                                                     \
    SCB->DCIMVAC = (uint32_t)start;                                         \
    start += CACHE_LINE_SIZE;                                               \
  }                                                                         \
  __DSB();                                                                  \
  __ISB();                                                                  \
}

/**
 * @brief   Flushes the data cache lines overlapping a DMA buffer.
 * @details This function is meant to make sure that data written in
 *          data cache is flushed to RAM.
 * @note    On devices without data cache this function does nothing.
 * @note    The function does not consider the lower 5 bits of addresses,
 *          the buffers are meant to be aligned to a 32 bytes boundary or
 *          adjacent data can be flushed as side effect.
 *
 * @param[in] saddr     start address of the DMA buffer
 * @param[in] n         size of the DMA buffer in bytes
 *
 * @api
 */
#define cacheBufferFlush(saddr, n) {                                        \
  uint8_t *start = (uint8_t *)(saddr);                                      \
  uint8_t *end = start + (size_t)(n);                                       \
  __DSB();                                                                  \
  while (start < end) {                                                     \
    SCB->DCCIMVAC = (uint32_t)start;                                        \
    start += CACHE_LINE_SIZE;                                               \
  }                                                                         \
  __DSB();                                                                  \
  __ISB();                                                                  \
}

#else /* __DCACHE_PRESENT == 0 */
#define cacheBufferInvalidate(addr, size) {                                 \
  (void)(addr);                                                             \
  (void)(size);                                                             \
}
#define cacheBufferFlush(addr, size) {                                      \
  (void)(addr);                                                             \
  (void)(size);                                                             \
}
#endif

#else /* !defined(__DCACHE_PRESENT) */
#define CACHE_SIZE_ALIGN(t, n) (n)

#define cacheBufferInvalidate(addr, size) {                                 \
  (void)(addr);                                                             \
  (void)(size);                                                             \
}
#define cacheBufferFlush(addr, size) {                                      \
  (void)(addr);                                                             \
  (void)(size);                                                             \
}
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* CACHE_H */

/** @} */
