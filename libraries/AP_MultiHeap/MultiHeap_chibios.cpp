/*
  allocation backend functions using native ChibiOS chHeap API
 */

#include "AP_MultiHeap.h"
#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <ch.h>
#include <hal.h>

/*
  heap functions used by lua scripting
 */
void *MultiHeap::heap_create(uint32_t size)
{
    memory_heap_t *heap = (memory_heap_t *)malloc(size + sizeof(memory_heap_t));
    if (heap == nullptr) {
        return nullptr;
    }
    chHeapObjectInit(heap, heap + 1U, size);
    return heap;
}

void MultiHeap::heap_destroy(void *ptr)
{
    free(ptr);
}

void *MultiHeap::heap_allocate(void *heap, uint32_t size)
{
    if (heap == nullptr) {
        return nullptr;
    }
    if (size == 0) {
        return nullptr;
    }
    return chHeapAlloc((memory_heap_t *)heap, size);
}

void MultiHeap::heap_free(void *ptr)
{
    return chHeapFree(ptr);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
