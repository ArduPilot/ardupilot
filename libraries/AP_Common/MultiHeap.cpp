/*
  multiple heap interface, allowing for an allocator that uses
  multiple underlying heaps to cope with multiple memory regions on
  STM32 boards
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>

#include "MultiHeap.h"

/*
  on ChibiOS allow up to 4 heaps. On other HALs only allow a single
  heap. This is needed as hal.util->heap_realloc() needs to have the
  property that heap_realloc(heap, ptr, 0) must not care if ptr comes
  from the given heap. This is true on ChibiOS, but is not true on
  other HALs
 */
#ifndef MAX_HEAPS
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define MAX_HEAPS 4
#else
#define MAX_HEAPS 1
#endif
#endif

extern const AP_HAL::HAL &hal;

/*
  create heaps with a total memory size, splitting over at most
  max_heaps
 */
bool MultiHeap::create(uint32_t total_size, uint8_t max_heaps)
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
        void *newptr = hal.util->heap_realloc(heaps[i], nullptr, 0, size);
        if (newptr != nullptr) {
            return newptr;
        }
    }
    return nullptr;
}

/*
  free memory from a heap
 */
void MultiHeap::deallocate(void *ptr)
{
    if (!available()) {
        return;
    }
    // NOTE! this relies on either there being a single heap or heap_realloc()
    // not using the first argument when size is zero.
    hal.util->heap_realloc(heaps[0], ptr, 0, 0);
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
    /*
      we cannot know which heap this came from, so we rely on the fact
      that on ChibiOS the underlying call doesn't use the heap
      argument and on other HALs we only have one heap. We pass
      through old_size and new_size so that we can validate the lua
      old_size value
    */
    return hal.util->heap_realloc(heaps[0], ptr, old_size, new_size);
}
