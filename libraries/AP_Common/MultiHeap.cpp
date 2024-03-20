/*
  multiple heap interface, allowing for an allocator that uses
  multiple underlying heaps to cope with multiple memory regions on
  STM32 boards
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

#include "MultiHeap.h"

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
    heaps = new void*[max_heaps];
    if (heaps == nullptr) {
        return false;
    }
    num_heaps = max_heaps;
    for (uint8_t i=0; i<max_heaps; i++) {
        uint32_t alloc_size = total_size;
        while (alloc_size > 0) {
            heaps[i] = hal.util->allocate_heap_memory(alloc_size);
            if (heaps[i] != nullptr) {
                total_size -= alloc_size;
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
        if (heaps[i] != nullptr) {
            free(heaps[i]);
            heaps[i] = nullptr;
        }
    }
    delete[] heaps;
    heaps = nullptr;
    num_heaps = 0;
}

// return true if heap is available for operations
bool MultiHeap::available(void) const
{
    return heaps != nullptr && heaps[0] != nullptr;
}

/*
  allocate memory from a specific heap
 */
void *MultiHeap::allocate_from_heap(void *heap, uint32_t size)
{
    struct alloc_header *ptr = (struct alloc_header *)hal.util->heap_realloc(heap, nullptr, 0, size+sizeof(alloc_header));
    if (ptr == nullptr) {
        return nullptr;
    }
    ptr->heap = heap;
#ifdef HAL_DEBUG_BUILD
    ptr->size = size;
#endif
    return (void*)(ptr+1);
}

/*
  allocate memory from a heap
 */
void *MultiHeap::allocate(uint32_t size)
{
    if (!available()) {
        return nullptr;
    }
    for (uint8_t i=0; i<num_heaps; i++) {
        if (heaps[i] == nullptr) {
            break;
        }
        void *newptr = allocate_from_heap(heaps[i], size);
        if (newptr != nullptr) {
            return newptr;
        }
    }
    if (!allow_expansion) {
        return nullptr;
    }

    if (!hal.util->get_soft_armed()) {
        // only expand the available heaps when armed. When disarmed
        // user should fix their SCR_HEAP_SIZE parameter
        return nullptr;
    }

    /*
      vehicle is armed and MultiHeap (for scripting) is out of
      memory. We will see if we can add a new heap from available
      memory if we have at least reserve_size bytes free
     */
    if (hal.util->available_memory() < reserve_size) {
        return nullptr;
    }
    const uint32_t heap_overhead = 128; // conservative value, varies with HAL

    // round up to multiple of 30k to allocate, and allow for heap overhead
    const uint32_t alloc_size = MAX(size+heap_overhead, 30*1024U);
    for (uint8_t i=0; i<num_heaps; i++) {
        if (heaps[i] == nullptr) {
            heaps[i] = hal.util->allocate_heap_memory(alloc_size);
            if (heaps[i] == nullptr) {
                return nullptr;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Expanded scripting heap by %u bytes", unsigned(alloc_size));
            return allocate_from_heap(heaps[i], size);
        }
    }
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
    struct alloc_header *h = ((struct alloc_header *)ptr)-1U;
    hal.util->heap_realloc(h->heap, h, 0, 0);
}

/*
  change size of an allocation
 */
void *MultiHeap::change_size(void *ptr, uint32_t old_size, uint32_t new_size)
{
    if (new_size == 0) {
        deallocate(ptr);
        return nullptr;
    }
    if (old_size == 0 || ptr == nullptr) {
        return allocate(new_size);
    }
#ifdef HAL_DEBUG_BUILD
    if (ptr != nullptr) {
        struct alloc_header *h = ((struct alloc_header *)ptr)-1U;
        if (h->size != old_size) {
            INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        }
    }
#endif
    /*
      always allocate again and copy. This allows memory to move
      between heaps
     */
    void *newptr = allocate(new_size);
    if (newptr == nullptr) {
        return nullptr;
    }
    if (ptr != nullptr) {
        memcpy(newptr, ptr, MIN(old_size, new_size));
    }
    deallocate(ptr);
    return newptr;
}
