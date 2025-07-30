#include "AP_MultiHeap.h"
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS

/*
  on systems other than chibios we map the allocation to the system
  malloc/free functions
 */

#include <AP_InternalError/AP_InternalError.h>
#include <stdlib.h>

/*
  low level allocator interface using system malloc
 */
/*
  default functions for heap management for lua scripting for systems
  that don't have the ability to create separable heaps
 */

struct heap_allocation_header {
    struct heap *hp;
    uint32_t allocation_size; // size of allocated block, not including this header
};

#define HEAP_MAGIC 0x5681ef9f

struct heap {
    uint32_t magic;
    uint32_t max_heap_size;
    uint32_t current_heap_usage;
};

void *MultiHeap::heap_create(uint32_t size)
{
    struct heap *new_heap = (struct heap*)malloc(sizeof(struct heap));
    if (new_heap != nullptr) {
        new_heap->magic = HEAP_MAGIC;
        new_heap->max_heap_size = size;
    }
    return (void *)new_heap;
}

void MultiHeap::heap_destroy(void *heap_ptr)
{
    struct heap *heapp = (struct heap*)heap_ptr;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (heapp->current_heap_usage != 0) {
        // lua should guarantee that there is no memory still
        // allocated when we destroy the heap. Throw an error in SITL
        // if this proves not to be true
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
#endif

    // free the heap structure
    free(heapp);
}

/*
  allocate memory on a specific heap
 */
void *MultiHeap::heap_allocate(void *heap_ptr, uint32_t size)
{
    if (heap_ptr == nullptr || size == 0) {
        return nullptr;
    }
    struct heap *heapp = (struct heap*)heap_ptr;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return nullptr;
    }

    if (heapp->current_heap_usage + size > heapp->max_heap_size) {
        // fail the allocation as we don't have the memory. Note that
        // we don't simulate the fragmentation that we would get with
        // HAL_ChibiOS
        return nullptr;
    }

    auto *header = (heap_allocation_header *)malloc(size + sizeof(heap_allocation_header));
    if (header == nullptr) {
        // can't allocate the new memory
        return nullptr;
    }
    header->hp = heapp;
    header->allocation_size = size;

    heapp->current_heap_usage += size;

    return header + 1;
}

/*
  free memory from a previous heap_allocate call
 */
void MultiHeap::heap_free(void *ptr)
{
    // extract header
    auto *header = ((struct heap_allocation_header *)ptr)-1;
    auto *heapp = header->hp;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    const uint32_t size = header->allocation_size;
    heapp->current_heap_usage -= size;

    // free the memory
    free(header);
}

#endif // CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS
