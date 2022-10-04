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
#include <stdint.h>
#include "stm32_util.h"

#ifdef HAL_CHIBIOS_ENABLE_MALLOC_GUARD
#pragma GCC optimize("Og")
#endif

#define MEM_REGION_FLAG_DMA_OK 1
#define MEM_REGION_FLAG_FAST   2
#define MEM_REGION_FLAG_AXI_BUS 4

#ifdef HAL_CHIBIOS_ENABLE_MALLOC_GUARD
static mutex_t mem_mutex;
#endif

static const struct memory_region memory_regions[] = { HAL_MEMORY_REGIONS };
// the first memory region is already setup as the ChibiOS
// default heap, so we will index from 1 in the allocators
#define NUM_MEMORY_REGIONS (sizeof(memory_regions)/sizeof(memory_regions[0]))

#if CH_CFG_USE_HEAP == TRUE

static memory_heap_t heaps[NUM_MEMORY_REGIONS];

#define MIN_ALIGNMENT 8U

#if defined(STM32H7)
#define DMA_ALIGNMENT 32U
#else
#define DMA_ALIGNMENT 8U
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
#ifdef HAL_CHIBIOS_ENABLE_MALLOC_GUARD
    chMtxObjectInit(&mem_mutex);
#endif

#if defined(STM32H7)
    // zero first 1k of ITCM. We leave 1k free to avoid addresses
    // close to nullptr being valid. Zeroing it here means we can
    // check for changes which indicate a write to an uninitialised
    // object.  We start at address 0x1 as writing the first byte
    // causes a fault
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if defined(__GNUC__) &&  __GNUC__ >= 10
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
    memset((void*)0x00000001, 0, 1023);
#pragma GCC diagnostic pop
#endif

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

/*
  allocate memory, using flags from MEM_REGION_FLAG_* to determine
  memory type
 */
static void *malloc_flags(size_t size, uint32_t flags)
{
    if (size == 0) {
        return NULL;
    }
    const uint8_t dma_flags = (MEM_REGION_FLAG_DMA_OK | MEM_REGION_FLAG_AXI_BUS);
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
        if ((flags & MEM_REGION_FLAG_AXI_BUS) &&
            !(memory_regions[i].flags & MEM_REGION_FLAG_AXI_BUS)) {
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

#ifdef HAL_CHIBIOS_ENABLE_MALLOC_GUARD
/*
  memory guard system. We put all allocated memory in a doubly linked
  list and add canary bytes at the front and back of all
  allocations. On all free operations, plus on calls to malloc_check()
  we walk the list and check for memory corruption, flagging an
  internal error if one is found
 */
struct memguard {
    uint32_t size;
    uint32_t inv_size;
    struct memguard *next, *prev;
    uint32_t pad[4]; // pad to 32 bytes
};
static struct memguard *mg_head;

#define MALLOC_HEAD_SIZE sizeof(struct memguard)
#define MALLOC_GUARD_SIZE DMA_ALIGNMENT
#define MALLOC_GUARD1_START 73
#define MALLOC_GUARD2_START 172

/*
  optional malloc guard regions
 */
static void *malloc_flags_guard(size_t size, uint32_t flags)
{
    chMtxLock(&mem_mutex);

    if (flags & (MEM_REGION_FLAG_DMA_OK | MEM_REGION_FLAG_AXI_BUS)) {
        size = (size + (DMA_ALIGNMENT-1U)) & ~(DMA_ALIGNMENT-1U);
    } else {
        size = (size + (MIN_ALIGNMENT-1U)) & ~(MIN_ALIGNMENT-1U);
    }
    void *ret = malloc_flags(size+MALLOC_GUARD_SIZE*2+MALLOC_HEAD_SIZE, flags);
    if (!ret) {
        chMtxUnlock(&mem_mutex);
        return NULL;
    }
    struct memguard *mg = (struct memguard *)ret;
    uint8_t *b1 = (uint8_t *)&mg[1];
    uint8_t *b2 = b1 + MALLOC_GUARD_SIZE + size;
    mg->size = size;
    mg->inv_size = ~size;
    for (uint32_t i=0; i<MALLOC_GUARD_SIZE; i++) {
        b1[i] = (uint8_t)(MALLOC_GUARD1_START + i);
        b2[i] = (uint8_t)(MALLOC_GUARD2_START + i);
    }

    if (mg_head != NULL) {
        mg->next = mg_head;
        mg_head->prev = mg;
    }
    mg_head = mg;

    chMtxUnlock(&mem_mutex);
    return (void *)(b1+MALLOC_GUARD_SIZE);
}

extern void AP_memory_guard_error(uint32_t size);

/*
  check for errors in malloc memory using guard bytes
 */
void malloc_check_mg(const struct memguard *mg)
{
    if (mg->size != ~mg->inv_size) {
        AP_memory_guard_error(0);
        return;
    }
    const uint32_t size = mg->size;
    const uint8_t *b1 = (uint8_t *)&mg[1];
    const uint8_t *b2 = b1 + MALLOC_GUARD_SIZE + size;
    for (uint32_t i=0; i<MALLOC_GUARD_SIZE; i++) {
        if (b1[i] != (uint8_t)(MALLOC_GUARD1_START + i) ||
            b2[i] != (uint8_t)(MALLOC_GUARD2_START + i)) {
            AP_memory_guard_error(size);
            return;
        }
    }
}

/*
  check for errors across entire allocation list
 */
void malloc_check_all(void)
{
    for (struct memguard *mg=mg_head; mg; mg=mg->next) {
        malloc_check_mg(mg);
    }
}

/*
  check for errors in malloc memory using guard bytes
 */
void malloc_check(const void *p)
{
    if (p == NULL) {
        // allow for malloc_check(nullptr) to check all allocated memory
        chMtxLock(&mem_mutex);
        malloc_check_all();
        chMtxUnlock(&mem_mutex);
        return;
    }
    if (((uintptr_t)p) & 3) {
        // misaligned memory
        AP_memory_guard_error(0);
        return;
    }
    chMtxLock(&mem_mutex);
    struct memguard *mg = (struct memguard *)(((uint8_t *)p) - (MALLOC_GUARD_SIZE+MALLOC_HEAD_SIZE));
    malloc_check_mg(mg);
    malloc_check_all();
    chMtxUnlock(&mem_mutex);
}

static void free_guard(void *p)
{
    chMtxLock(&mem_mutex);
    malloc_check(p);
    struct memguard *mg = (struct memguard *)(((uint8_t *)p) - (MALLOC_GUARD_SIZE+MALLOC_HEAD_SIZE));
    if (mg->next) {
        mg->next->prev = mg->prev;
    }
    if (mg->prev) {
        mg->prev->next = mg->next;
    }
    if (mg == mg_head) {
        mg_head = mg->next;
    }
    chHeapFree((void*)(((uint8_t *)p) - (MALLOC_GUARD_SIZE+MALLOC_HEAD_SIZE)));
    chMtxUnlock(&mem_mutex);
}

#define malloc_flags(size, flags) malloc_flags_guard(size, flags)

#else // HAL_CHIBIOS_ENABLE_MALLOC_GUARD

void malloc_check(const void *p)
{
    (void)p;
}
#endif // HAL_CHIBIOS_ENABLE_MALLOC_GUARD



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
  allocate from memory connected to AXI Bus if available
  else just allocate dma safe memory
 */
void *malloc_axi_sram(size_t size)
{
#if defined(STM32H7)
    return malloc_flags(size, MEM_REGION_FLAG_AXI_BUS);
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
#ifdef HAL_CHIBIOS_ENABLE_MALLOC_GUARD
        free_guard(ptr);
#else
        chHeapFree(ptr);
#endif
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

#if CH_CFG_USE_DYNAMIC == TRUE
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
#endif

/*
  return heap information
 */
uint8_t malloc_get_heaps(memory_heap_t **_heaps, const struct memory_region **regions)
{
    *_heaps = &heaps[0];
    *regions = &memory_regions[0];
    return NUM_MEMORY_REGIONS;
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

/*
    is valid memory region
 */
bool is_address_in_memory(void *addr)
{
    uint8_t i;
    for (i=0; i<NUM_MEMORY_REGIONS; i++) {
        if (addr >= memory_regions[i].address &&
            addr < (memory_regions[i].address + memory_regions[i].size)) {
            return true;
        }
    }
    return false;
}

/*
  return the start of memory region that contains the address
 */
void* get_addr_mem_region_start_addr(void *addr)
{ 
    uint8_t i;
    for (i=0; i<NUM_MEMORY_REGIONS; i++) {
        if (addr >= memory_regions[i].address &&
            addr < (memory_regions[i].address + memory_regions[i].size)) {
            return memory_regions[i].address;
        }
    }
    return 0;
}

/*
  return the end of memory region that contains the address
 */
void* get_addr_mem_region_end_addr(void *addr)
{ 
    uint8_t i;
    for (i=0; i<NUM_MEMORY_REGIONS; i++) {
        if (addr >= memory_regions[i].address &&
            addr < (memory_regions[i].address + memory_regions[i].size)) {
            return memory_regions[i].address + memory_regions[i].size;
        }
    }
    return 0;
}
