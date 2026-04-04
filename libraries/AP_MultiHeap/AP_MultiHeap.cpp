/*
  multiple heap interface, allowing for an allocator that uses
  multiple underlying heaps to cope with multiple memory regions on
  STM32 boards
 */

#include "AP_MultiHeap.h"

#include <AP_Math/AP_Math.h>
#include <stdio.h>

/*
  allow up to 10 heaps
 */
#ifndef MAX_HEAPS
#define MAX_HEAPS 10
#endif

extern const AP_HAL::HAL &hal;

/*
  create heaps with a total memory size, splitting over at most
  max_heaps
 */
bool MultiHeap::create(uint32_t total_size, uint8_t max_heaps, bool _allow_expansion, uint32_t _reserve_size)
{
    max_heaps = MIN(MAX_HEAPS, max_heaps);
    if (heaps != nullptr) {
        // don't allow double allocation
        return false;
    }
    heaps = NEW_NOTHROW Heap[max_heaps];
    if (heaps == nullptr) {
        return false;
    }
    num_heaps = max_heaps;
    for (uint8_t i=0; i<max_heaps; i++) {
        uint32_t alloc_size = total_size;
        while (alloc_size > 0) {
            heaps[i].hp = heap_create(alloc_size);
            if (heaps[i].hp != nullptr) {
                total_size -= alloc_size;
                sum_size += alloc_size;
                break;
            }
            alloc_size *= 0.9;
        }
        if (total_size == 0) {
            break;
        }
    }
    if (total_size != 0) {
        destroy();
        return false;
    }

    allow_expansion = _allow_expansion;
    reserve_size = _reserve_size;

    return true;
}

// destroy heap
void MultiHeap::destroy(void)
{
    if (!available()) {
        return;
    }
    for (uint8_t i=0; i<num_heaps; i++) {
        if (heaps[i].hp != nullptr) {
            heap_destroy(heaps[i].hp);
            heaps[i].hp = nullptr;
        }
    }
    delete[] heaps;
    heaps = nullptr;
    num_heaps = 0;
    sum_size = 0;
    expanded_to = 0;
}

// return true if heap is available for operations
bool MultiHeap::available(void) const
{
    return heaps != nullptr && heaps[0].hp != nullptr;
}

/*
  allocate memory from a heap
 */
void *MultiHeap::allocate(uint32_t size)
{
    if (!available() || size == 0) {
        return nullptr;
    }
    for (uint8_t i=0; i<num_heaps; i++) {
        if (heaps[i].hp == nullptr) {
            break;
        }
        void *newptr = heap_allocate(heaps[i].hp, size);
        if (newptr != nullptr) {
            last_failed = false;
            return newptr;
        }
    }
    if (!allow_expansion || !last_failed) {
        /*
          we only allow expansion when the last allocation
          failed. This gives the lua engine a chance to use garbage
          collection to recover memory
         */
        last_failed = true;
        return nullptr;
    }

    if (!hal.util->get_soft_armed()) {
        // only expand the available heaps when armed. When disarmed
        // user should fix their SCR_HEAP_SIZE parameter
        last_failed = true;
        return nullptr;
    }

    /*
      vehicle is armed and MultiHeap (for scripting) is out of
      memory. We will see if we can add a new heap from available
      memory if we have at least reserve_size bytes free
     */
    const uint32_t available = hal.util->available_memory();
    const uint32_t heap_overhead = 128; // conservative value, varies with HAL
    const uint32_t min_size = size + heap_overhead;
    if (available < reserve_size+min_size) {
        last_failed = true;
        return nullptr;
    }

    // round up to a minimum of 30k to allocate, and allow for heap overhead
    const uint32_t round_to = 30*1024U;
    const uint32_t alloc_size = MIN(available - reserve_size, MAX(size+heap_overhead, round_to));
    if (alloc_size < min_size) {
        last_failed = true;
        return nullptr;
    }
    for (uint8_t i=0; i<num_heaps; i++) {
        if (heaps[i].hp == nullptr) {
            heaps[i].hp = heap_create(alloc_size);
            if (heaps[i].hp == nullptr) {
                last_failed = true;
                return nullptr;
            }
            sum_size += alloc_size;
            expanded_to = sum_size;
            void *p = heap_allocate(heaps[i].hp, size);
            last_failed = p == nullptr;
            return p;
        }
    }
    last_failed = true;
    return nullptr;
}

/*
  free memory from a heap
 */
void MultiHeap::deallocate(void *ptr)
{
    if (!available() || ptr == nullptr) {
        return;
    }
    heap_free(ptr);
}

/*
  change size of an allocation, operates like realloc(), but requires
  the old_size when ptr is not NULL
 */
void *MultiHeap::change_size(void *ptr, uint32_t old_size, uint32_t new_size)
{
    if (new_size == 0) {
        deallocate(ptr);
        return nullptr;
    }
    /*
      we don't want to require the underlying allocation system to
      support realloc() and we also want to be able to handle the case
      of having to move the allocation to a new heap, so we do a
      simple alloc/copy/deallocate for reallocation
     */
    void *newp = allocate(new_size);
    if (ptr == nullptr) {
        return newp;
    }
    if (newp == nullptr) {
        if (old_size >= new_size) {
            // Lua assumes that the allocator never fails when osize >= nsize
            // the best we can do is return the old pointer
            return ptr;
        }
        return nullptr;
    }
    memcpy(newp, ptr, MIN(old_size, new_size));
    deallocate(ptr);
    return newp;
}
