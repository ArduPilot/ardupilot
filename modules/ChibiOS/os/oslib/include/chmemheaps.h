/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    oslib/include/chmemheaps.h
 * @brief   Memory heaps macros and structures.
 *
 * @addtogroup oslib_memheaps
 * @{
 */

#ifndef CHMEMHEAPS_H
#define CHMEMHEAPS_H

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Minimum alignment used for heap.
 * @note    Cannot use the sizeof operator in this macro.
 */
#if (SIZEOF_PTR == 4) || defined(__DOXYGEN__)
#define CH_HEAP_ALIGNMENT   8U
#elif (SIZEOF_PTR == 2)
#define CH_HEAP_ALIGNMENT   4U
#else
#error "unsupported pointer size"
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_USE_MEMCORE == FALSE
#error "CH_CFG_USE_HEAP requires CH_CFG_USE_MEMCORE"
#endif

#if (CH_CFG_USE_MUTEXES == FALSE) && (CH_CFG_USE_SEMAPHORES == FALSE)
#error "CH_CFG_USE_HEAP requires CH_CFG_USE_MUTEXES and/or CH_CFG_USE_SEMAPHORES"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a memory heap.
 */
typedef struct memory_heap memory_heap_t;

/**
 * @brief   Type of a memory heap header.
 */
typedef union heap_header heap_header_t;

/**
 * @brief   Memory heap block header.
 */
union heap_header {
  struct {
    heap_header_t       *next;      /**< @brief Next block in free list.    */
    size_t              pages;      /**< @brief Size of the area in pages.  */
  } free;
  struct {
    memory_heap_t       *heap;      /**< @brief Block owner heap.           */
    size_t              size;       /**< @brief Size of the area in bytes.  */
  } used;
};

/**
 * @brief   Structure describing a memory heap.
 */
struct memory_heap {
  memgetfunc2_t         provider;   /**< @brief Memory blocks provider for
                                                this heap.                  */
  heap_header_t         header;     /**< @brief Free blocks list header.    */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  mutex_t               mtx;        /**< @brief Heap access mutex.          */
#else
  semaphore_t           sem;        /**< @brief Heap access semaphore.      */
#endif
};

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Allocation of an aligned static heap buffer.
 */
#define CH_HEAP_AREA(name, size)                                            \
  ALIGNED_VAR(CH_HEAP_ALIGNMENT)                                            \
  uint8_t name[MEM_ALIGN_NEXT((size), CH_HEAP_ALIGNMENT)]

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void __heap_init(void);
  void chHeapObjectInit(memory_heap_t *heapp, void *buf, size_t size);
  void *chHeapAllocAligned(memory_heap_t *heapp, size_t size, unsigned align);
  void chHeapFree(void *p);
  size_t chHeapStatus(memory_heap_t *heapp, size_t *totalp, size_t *largestp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Allocates a block of memory from the heap by using the first-fit
 *          algorithm.
 * @details The allocated block is guaranteed to be properly aligned for a
 *          pointer data type.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] size      the size of the block to be allocated. Note that the
 *                      allocated block may be a bit bigger than the requested
 *                      size for alignment and fragmentation reasons.
 * @return              A pointer to the allocated block.
 * @retval NULL         if the block cannot be allocated.
 *
 * @api
 */
static inline void *chHeapAlloc(memory_heap_t *heapp, size_t size) {

  return chHeapAllocAligned(heapp, size, CH_HEAP_ALIGNMENT);
}

/**
 * @brief   Returns the size of an allocated block.
 * @note    The returned value is the requested size, the real size is the
 *          same value aligned to the next @p CH_HEAP_ALIGNMENT multiple.
 *
 * @param[in] p         pointer to the memory block
 * @return              Size of the block.
 *
 * @api
 */
static inline size_t chHeapGetSize(const void *p) {

  return ((heap_header_t *)p - 1U)->used.size;
}

#endif /* CH_CFG_USE_HEAP == TRUE */

#endif /* CHMEMHEAPS_H */

/** @} */
