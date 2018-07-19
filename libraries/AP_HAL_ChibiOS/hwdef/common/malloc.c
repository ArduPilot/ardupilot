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
/*
  wrappers for allocation functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */

#include <stdio.h>
#include <string.h>
#include <hal.h>
#include <chheap.h>
#include <stdarg.h>
#include "stm32_util.h"

#if CH_CFG_USE_HEAP == TRUE

#define MIN_ALIGNMENT 8

#ifdef HAL_NO_CCM
#undef CCM_RAM_SIZE_KB
#endif

#if defined(CCM_RAM_SIZE_KB)
static memory_heap_t ccm_heap;
#endif

#if defined(DTCM_RAM_SIZE_KB)
static memory_heap_t dtcm_heap;
#endif

// size of memory reserved for dma-capable alloc
#ifndef DMA_RESERVE_SIZE
#define DMA_RESERVE_SIZE 4096
#endif

static memory_heap_t dma_reserve_heap;

static void *malloc_dtcm(size_t size);

/*
  initialise memory handling
 */
void malloc_init(void)
{
#if defined(CCM_RAM_SIZE_KB)
    chHeapObjectInit(&ccm_heap, (void *)CCM_BASE_ADDRESS, CCM_RAM_SIZE_KB*1024);
#endif

#if defined(DTCM_RAM_SIZE_KB)
    chHeapObjectInit(&dtcm_heap, (void *)DTCM_BASE_ADDRESS, DTCM_RAM_SIZE_KB*1024);
#endif

    /*
      create a DMA reserve heap, to ensure we keep some memory for DMA
      safe memory allocations
     */
    void *dma_reserve = malloc_dtcm(DMA_RESERVE_SIZE);
    if (!dma_reserve) {
        dma_reserve = chHeapAllocAligned(NULL, DMA_RESERVE_SIZE, MIN_ALIGNMENT);
    }
    chHeapObjectInit(&dma_reserve_heap, dma_reserve, DMA_RESERVE_SIZE);
}

void *malloc_ccm(size_t size)
{
    void *p = NULL;
#if defined(CCM_RAM_SIZE_KB)
    p = chHeapAllocAligned(&ccm_heap, size, CH_HEAP_ALIGNMENT);
    if (p != NULL) {
        memset(p, 0, size);
    }
#else
    (void)size;
#endif
    return p;
}

static void *malloc_dtcm(size_t size)
{
    void *p = NULL;
#if defined(DTCM_RAM_SIZE_KB)
    p = chHeapAllocAligned(&dtcm_heap, size, CH_HEAP_ALIGNMENT);
#else
    (void)size;
#endif
    if (p != NULL) {
        memset(p, 0, size);
    }
    return p;
}

void *malloc(size_t size)
{
    if (size == 0) {
        return NULL;
    }
    void *p = chHeapAllocAligned(NULL, size, MIN_ALIGNMENT);
    if (p) {
        memset(p, 0, size);
        return p;
    }
    if (!p) {
        // fall back to CCM memory
        p = malloc_ccm(size);
        if (p) {
            return p;
        }
    }
    if (!p) {
        // fall back to DTCM memory
        p = malloc_dtcm(size);
        if (p) {
            return p;
        }
    }

    // fall back to DMA reserve
    p = chHeapAllocAligned(&dma_reserve_heap, size, MIN_ALIGNMENT);
    if (p) {
        memset(p, 0, size);
        return p;
    }

    return NULL;
}

/*
  allocate DMA-safe memory
 */
void *malloc_dma(size_t size)
{
    void *p;
#if defined(DTCM_RAM_SIZE_KB)
    p = malloc_dtcm(size);
#else
    // if we don't have DTCM memory then assume that main heap is DMA-safe
    p = chHeapAllocAligned(NULL, size, MIN_ALIGNMENT);
#endif
    if (p == NULL) {
        p = chHeapAllocAligned(&dma_reserve_heap, size, MIN_ALIGNMENT);
    }
    if (p) {
        memset(p, 0, size);
    }
    return p;
}

void *calloc(size_t nmemb, size_t size)
{
    return malloc(nmemb * size);
}

void free(void *ptr)
{
    if(ptr != NULL) {
        chHeapFree(ptr);
    }
}

/*
  return total available memory in bytes
 */
size_t mem_available(void)
{
    size_t totalp = 0;
    // get memory available on main heap
    chHeapStatus(NULL, &totalp, NULL);

    // we also need to add in memory that is not yet allocated to the heap
    totalp += chCoreGetStatusX();

#if defined(CCM_RAM_SIZE_KB)
    size_t ccm_available = 0;
    chHeapStatus(&ccm_heap, &ccm_available, NULL);
    totalp += ccm_available;
#endif

#if defined(DTCM_RAM_SIZE_KB)
    size_t dtcm_available = 0;
    chHeapStatus(&dtcm_heap, &dtcm_available, NULL);
    totalp += dtcm_available;
#endif

    size_t reserve_available = 0;
    chHeapStatus(&dma_reserve_heap, &reserve_available, NULL);
    totalp += reserve_available;

    return totalp;
}

#endif // CH_CFG_USE_HEAP
