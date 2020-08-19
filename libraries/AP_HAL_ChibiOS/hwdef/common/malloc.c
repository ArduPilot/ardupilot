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
#include <ch.h>
#include <stdarg.h>
#include "stm32_util.h"

#define MEM_REGION_FLAG_DMA_OK 1
#define MEM_REGION_FLAG_FAST   2
#define MEM_REGION_FLAG_SDCARD 4

static const struct memory_region {
    void *address;
    uint32_t size;
    uint32_t flags;
} memory_regions[] = { HAL_MEMORY_REGIONS };

// the first memory region is already setup as the ChibiOS
// default heap, so we will index from 1 in the allocators
#define NUM_MEMORY_REGIONS (sizeof(memory_regions)/sizeof(memory_regions[0]))

#if CH_CFG_USE_HEAP == TRUE

static memory_heap_t heaps[NUM_MEMORY_REGIONS];

#define MIN_ALIGNMENT 8

#if defined(STM32H7)
#define DMA_ALIGNMENT 32
#else
#define DMA_ALIGNMENT 8
#endif

// size of memory reserved for dma-capable alloc
#ifndef DMA_RESERVE_SIZE
#define DMA_RESERVE_SIZE 6144
#endif

#if DMA_RESERVE_SIZE != 0
static memory_heap_t dma_reserve_heap;
#endif

/*
  initialise memory handling
 */
void malloc_init(void)
{
    uint8_t i;
    for (i=1; i<NUM_MEMORY_REGIONS; i++) {
        chHeapObjectInit(&heaps[i], memory_regions[i].address, memory_regions[i].size);
    }

#if DMA_RESERVE_SIZE != 0
    /*
      create a DMA reserve heap, to ensure we keep some memory for DMA
      safe memory allocations
     */
    uint32_t reserve_size = DMA_RESERVE_SIZE;
    while (reserve_size > 0) {
        void *dma_reserve = malloc_dma(reserve_size);
        if (dma_reserve != NULL) {
            chHeapObjectInit(&dma_reserve_heap, dma_reserve, reserve_size);
            break;
        }
        reserve_size = (reserve_size * 7) / 8;
    }
#endif
}

static void *malloc_flags(size_t size, uint32_t flags)
{
    if (size == 0) {
        return NULL;
    }
    const uint8_t dma_flags = (MEM_REGION_FLAG_DMA_OK | MEM_REGION_FLAG_SDCARD);
    const uint8_t alignment = (flags&dma_flags?DMA_ALIGNMENT:MIN_ALIGNMENT);
    void *p = NULL;
    uint8_t i;

    if (flags & dma_flags) {
        // allocate multiple of DMA alignment
        size = (size + (DMA_ALIGNMENT-1)) & ~(DMA_ALIGNMENT-1);
    }

    // if no flags are set or this is a DMA request and default heap
    // is DMA safe then start with default heap
    if (flags == 0 || (flags == MEM_REGION_FLAG_DMA_OK &&
                       (memory_regions[0].flags & MEM_REGION_FLAG_DMA_OK))) {
        p = chHeapAllocAligned(NULL, size, alignment);
        if (p) {
            goto found;
        }
    }

    // try with matching flags
    for (i=1; i<NUM_MEMORY_REGIONS; i++) {
        if ((flags & MEM_REGION_FLAG_DMA_OK) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_DMA_OK)) {
            continue;
        }
        if ((flags & MEM_REGION_FLAG_SDCARD) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_SDCARD)) {
            continue;
        }
        if ((flags & MEM_REGION_FLAG_FAST) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_FAST)) {
            continue;
        }
        p = chHeapAllocAligned(&heaps[i], size, alignment);
        if (p) {
            goto found;
        }
    }

    // if this is a not a DMA request then we can fall back to any heap
    if (!(flags & dma_flags)) {
        for (i=1; i<NUM_MEMORY_REGIONS; i++) {
            p = chHeapAllocAligned(&heaps[i], size, alignment);
            if (p) {
                goto found;
            }
        }
        // try default heap
        p = chHeapAllocAligned(NULL, size, alignment);
        if (p) {
            goto found;
        }
    }

#if DMA_RESERVE_SIZE != 0
    // fall back to DMA reserve
    p = chHeapAllocAligned(&dma_reserve_heap, size, alignment);
    if (p) {
        memset(p, 0, size);
        return p;
    }
#endif

    // failed
    return NULL;

found:
    memset(p, 0, size);
    return p;
}
/*
  allocate normal memory
 */
void *malloc(size_t size)
{
    return malloc_flags(size, 0);
}

/*
  allocate DMA-safe memory
 */
void *malloc_dma(size_t size)
{
    return malloc_flags(size, MEM_REGION_FLAG_DMA_OK);
}

/*
  allocate DMA-safe memory for microSD transfers. This is only
  different on H7 where SDMMC IDMA can't use SRAM4
 */
void *malloc_sdcard_dma(size_t size)
{
#if defined(STM32H7)
    return malloc_flags(size, MEM_REGION_FLAG_SDCARD);
#else
    return malloc_flags(size, MEM_REGION_FLAG_DMA_OK);
#endif
}

/*
  allocate fast memory
 */
void *malloc_fastmem(size_t size)
{
    return malloc_flags(size, MEM_REGION_FLAG_FAST);
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
    uint8_t i;

    // get memory available on main heap
    chHeapStatus(NULL, &totalp, NULL);

    // we also need to add in memory that is not yet allocated to the heap
    totalp += chCoreGetStatusX();

    // now our own heaps
    for (i=1; i<NUM_MEMORY_REGIONS; i++) {
        size_t available = 0;
        chHeapStatus(&heaps[i], &available, NULL);
        totalp += available;
    }

#if DMA_RESERVE_SIZE != 0
    // and reserve DMA heap
    size_t available = 0;
    chHeapStatus(&dma_reserve_heap, &available, NULL);
    totalp += available;
#endif

    return totalp;
}

/*
  allocate a thread on any available heap
 */
thread_t *thread_create_alloc(size_t size,
                              const char *name, tprio_t prio,
                              tfunc_t pf, void *arg)
{
    thread_t *ret;
    // first try default heap
    ret = chThdCreateFromHeap(NULL, size, name, prio, pf, arg);
    if (ret != NULL) {
        return ret;
    }

    // now try other heaps
    uint8_t i;
    for (i=1; i<NUM_MEMORY_REGIONS; i++) {
        ret = chThdCreateFromHeap(&heaps[i], size, name, prio, pf, arg);
        if (ret != NULL) {
            return ret;
        }
    }
    return NULL;
}

#endif // CH_CFG_USE_HEAP


/*
  flush all memory. Used in chSysHalt()
 */
void memory_flush_all(void)
{
    uint8_t i;
    for (i=0; i<NUM_MEMORY_REGIONS; i++) {
        stm32_cacheBufferFlush(memory_regions[i].address, memory_regions[i].size);
    }
}

/*
  replacement for strdup
 */
char *strdup(const char *str)
{
    const size_t len = strlen(str);
    char *ret = malloc(len+1);
    if (!ret) {
        return NULL;
    }
    memcpy(ret, str, len);
    ret[len] = 0;
    return ret;
}
