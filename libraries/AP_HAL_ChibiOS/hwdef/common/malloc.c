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

#define MIN_ALIGNMENT 8

#ifdef STM32F4xx_MCUCONF
// assume all F4xx MCUs have 64k CCM 
#define CCM_RAM_ATTRIBUTE __attribute__((section(".ram4")))
#endif

#ifdef CCM_RAM_ATTRIBUTE
//CCM RAM Heap
#define CCM_REGION_SIZE 64*1024
CH_HEAP_AREA(ccm_heap_region, CCM_REGION_SIZE) CCM_RAM_ATTRIBUTE;
static memory_heap_t ccm_heap;
static bool ccm_heap_initialised = false;
#endif

void *malloc_ccm(size_t size)
{
    void *p = NULL;
#ifdef CCM_RAM_ATTRIBUTE
    if (!ccm_heap_initialised) {
        ccm_heap_initialised = true;
        chHeapObjectInit(&ccm_heap, ccm_heap_region, CCM_REGION_SIZE);
    }
    p = chHeapAllocAligned(&ccm_heap, size, CH_HEAP_ALIGNMENT);
    if (p != NULL) {
        memset(p, 0, size);
    }
#endif
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
    } else {
        // fall back to CCM memory when main memory full
        p = malloc_ccm(size);
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

#ifdef CCM_RAM_ATTRIBUTE
    size_t ccm_available = 0;
    chHeapStatus(&ccm_heap, &ccm_available, NULL);
    totalp += ccm_available;
#endif
    
    return totalp;
}
